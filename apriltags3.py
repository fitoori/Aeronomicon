#!/usr/bin/env python3
"""
Python wrapper for the C implementation of AprilTags (v3).
The public API is unchanged; all improvements are internal.

Changes vs. upstream (2025‑05‑21):
• Robust shared‑library loading via ``ctypes.util.find_library``.
• Table‑driven family create/destroy to remove repetitive code.
• Correct float handling for ``decode_sharpening``.
• Guaranteed release of C resources with ``try/finally``.
• Safer raw‑pointer → NumPy conversions.
• Minor speed tweaks (contiguous input, cached restype).
• ``Detection`` now uses ``__slots__``; ``Detector`` implements the
  context‑manager protocol for deterministic cleanup.
• All I/O guarded with explicit error handling; no placeholders.

The module still exposes the two classes ``Detection`` and ``Detector``
and keeps the same constructor signature and ``detect`` method.
"""
from __future__ import division, print_function

import ctypes
import os
import sys
from ctypes.util import find_library
from typing import Dict, List, Optional, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# C‑struct wrappers (unchanged field layouts)
# ---------------------------------------------------------------------------

class _ImageU8(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("stride", ctypes.c_int),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]


class _Matd(ctypes.Structure):
    _fields_ = [
        ("nrows", ctypes.c_int),
        ("ncols", ctypes.c_int),
        ("data", ctypes.POINTER(ctypes.c_double)),
    ]


class _ZArray(ctypes.Structure):
    _fields_ = [
        ("el_sz", ctypes.c_size_t),
        ("size", ctypes.c_int),
        ("alloc", ctypes.c_int),
        ("data", ctypes.c_void_p),
    ]


class _ApriltagFamily(ctypes.Structure):
    _fields_ = [
        ("ncodes", ctypes.c_uint32),
        ("codes", ctypes.POINTER(ctypes.c_uint64)),
        ("width_at_border", ctypes.c_int),
        ("total_width", ctypes.c_int),
        ("reversed_border", ctypes.c_bool),
        ("nbits", ctypes.c_uint32),
        ("bit_x", ctypes.POINTER(ctypes.c_uint32)),
        ("bit_y", ctypes.POINTER(ctypes.c_uint32)),
        ("h", ctypes.c_int32),
        ("name", ctypes.c_char_p),
    ]


class _ApriltagDetection(ctypes.Structure):
    _fields_ = [
        ("family", ctypes.POINTER(_ApriltagFamily)),
        ("id", ctypes.c_int),
        ("hamming", ctypes.c_int),
        ("decision_margin", ctypes.c_float),
        ("H", ctypes.POINTER(_Matd)),
        ("c", ctypes.c_double * 2),
        ("p", (ctypes.c_double * 2) * 4),
    ]


class _ApriltagDetector(ctypes.Structure):
    _fields_ = [
        ("nthreads", ctypes.c_int),
        ("quad_decimate", ctypes.c_float),
        ("quad_sigma", ctypes.c_float),
        ("refine_edges", ctypes.c_int),
        ("decode_sharpening", ctypes.c_double),
        ("debug", ctypes.c_int),
    ]


class _ApriltagDetectionInfo(ctypes.Structure):
    _fields_ = [
        ("det", ctypes.POINTER(_ApriltagDetection)),
        ("tagsize", ctypes.c_double),
        ("fx", ctypes.c_double),
        ("fy", ctypes.c_double),
        ("cx", ctypes.c_double),
        ("cy", ctypes.c_double),
    ]


class _ApriltagPose(ctypes.Structure):
    _fields_ = [
        ("R", ctypes.POINTER(_Matd)),
        ("t", ctypes.POINTER(_Matd)),
    ]

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


def _ptr_to_array2d(dtype, ptr, rows: int, cols: int) -> np.ndarray:
    """Return a (rows×cols) NumPy view into a C array."""
    if not ptr:
        raise ValueError("Null pointer received from C library")
    count = rows * cols
    buf_type = dtype * count
    buf = ctypes.cast(ptr, ctypes.POINTER(buf_type)).contents
    return np.ctypeslib.as_array(buf).reshape(rows, cols)


def _image_u8_get_array(img_ptr) -> np.ndarray:
    return _ptr_to_array2d(
        ctypes.c_uint8,
        img_ptr.contents.buf,
        img_ptr.contents.height,
        img_ptr.contents.stride,
    )


def _matd_get_array(mat_ptr) -> np.ndarray:
    return _ptr_to_array2d(
        ctypes.c_double,
        mat_ptr.contents.data,
        int(mat_ptr.contents.nrows),
        int(mat_ptr.contents.ncols),
    )


def _zarray_get(za_ptr, idx: int, out_ptr):
    """Equivalent to C macro ``zarray_get`` (memcpy)."""
    za = za_ptr.contents
    ctypes.memmove(out_ptr, za.data + idx * za.el_sz, za.el_sz)

# ---------------------------------------------------------------------------
# High‑level Python objects
# ---------------------------------------------------------------------------


class Detection:  # noqa: D401 — keep simple name for API stability
    """Pythonic wrapper around *apriltag_detection* and pose data."""

    __slots__ = (
        "tag_family",
        "tag_id",
        "hamming",
        "decision_margin",
        "homography",
        "center",
        "corners",
        "pose_R",
        "pose_t",
        "pose_err",
    )

    def __init__(self):
        for attr in self.__slots__:
            setattr(self, attr, None)

    # Human‑readable representation (unchanged)
    def __str__(self):
        parts = [f"{k} = {getattr(self, k)}" for k in self.__slots__]
        return "Detection(\n  " + ",\n  ".join(parts) + "\n)"

    __repr__ = __str__


class Detector:
    """Pythonic wrapper for ``apriltag_detector`` (API unchanged)."""

    _FAMILIES: Dict[str, tuple] = {
        "tag16h5": ("tag16h5_create", "tag16h5_destroy"),
        "tag25h9": ("tag25h9_create", "tag25h9_destroy"),
        "tag36h11": ("tag36h11_create", "tag36h11_destroy"),
        "tagCircle21h7": ("tagCircle21h7_create", "tagCircle21h7_destroy"),
        "tagCircle49h12": ("tagCircle49h12_create", "tagCircle49h12_destroy"),
        "tagCustom48h12": ("tagCustom48h12_create", "tagCustom48h12_destroy"),
        "tagStandard41h12": ("tagStandard41h12_create", "tagStandard41h12_destroy"),
        "tagStandard52h13": ("tagStandard52h13_create", "tagStandard52h13_destroy"),
    }

    # ------------- construction utilities ------------------

    @staticmethod
    def _load_library() -> ctypes.CDLL:
        """Load libapriltag using the platform search path."""
        libname = find_library("apriltag")
        if not libname:
            sys.stderr.write("[apriltag] shared library not found; ensure it is in LD_LIBRARY_PATH / DYLD_LIBRARY_PATH\n")
            raise FileNotFoundError("libapriltag not found")
        return ctypes.CDLL(libname)

    # ------------- public API ------------------

    def __init__(
        self,
        *,
        families: str = "tag36h11",
        nthreads: int = 1,
        quad_decimate: float = 2.0,
        quad_sigma: float = 0.0,
        refine_edges: int = 1,
        decode_sharpening: float = 0.25,
        debug: int = 0,
    ) -> None:
        self.params = {
            "families": families.split(),
            "nthreads": int(nthreads),
            "quad_decimate": float(quad_decimate),
            "quad_sigma": float(quad_sigma),
            "refine_edges": int(refine_edges),
            "decode_sharpening": float(decode_sharpening),
            "debug": int(debug),
        }

        # Load C library
        self.libc: ctypes.CDLL = self._load_library()

        # Build detector object
        self.libc.apriltag_detector_create.restype = ctypes.POINTER(_ApriltagDetector)
        self._detector = self.libc.apriltag_detector_create()
        if not self._detector:
            raise RuntimeError("Failed to create apriltag_detector (NULL pointer)")

        # Cache restype once (speed)
        self.libc.estimate_tag_pose.restype = ctypes.c_double

        # Configure detector parameters
        det = self._detector.contents
        det.nthreads = self.params["nthreads"]
        det.quad_decimate = self.params["quad_decimate"]
        det.quad_sigma = self.params["quad_sigma"]
        det.refine_edges = self.params["refine_edges"]
        det.decode_sharpening = self.params["decode_sharpening"]
        det.debug = self.params["debug"]

        # Attach families
        self._families: Dict[str, ctypes.POINTER(_ApriltagFamily)] = {}
        for fam_name in self.params["families"]:
            try:
                create_fn, _ = self._FAMILIES[fam_name]
            except KeyError:
                raise ValueError(f"Unknown tag family: {fam_name}") from None
            creator = getattr(self.libc, create_fn)
            creator.restype = ctypes.POINTER(_ApriltagFamily)
            fam_ptr = creator()
            if not fam_ptr:
                raise RuntimeError(f"Failed to create family {fam_name}")
            self._families[fam_name] = fam_ptr
            self.libc.apriltag_detector_add_family_bits(self._detector, fam_ptr, 2)

    # Context‑manager sugar
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):  # noqa: D401
        self.__del__()
        return False  # propagate exceptions

    # ------------- destructor ------------------

    def __del__(self):
        if getattr(self, "_detector", None):
            # Destroy families first
            for name, ptr in self._families.items():
                _, destroy_fn_name = self._FAMILIES[name]
                destroy_fn = getattr(self.libc, destroy_fn_name)
                destroy_fn.restype = None
                destroy_fn(ptr)
            # Destroy detector
            self.libc.apriltag_detector_destroy.restype = None
            self.libc.apriltag_detector_destroy(self._detector)
            self._detector = None

    # ------------- main workhorse ------------------

    def detect(
        self,
        img: np.ndarray,
        estimate_tag_pose: bool = False,
        camera_params: Optional[Sequence[float]] = None,
        tag_size: Optional[float] = None,
    ) -> List[Detection]:
        """Detect tags in a grayscale uint8 image."""
        if img.ndim != 2 or img.dtype != np.uint8:
            raise TypeError("img must be a 2‑D numpy.uint8 array (grayscale)")

        # Ensure contiguous memory; cheap no‑op if already contiguous
        img_c = np.ascontiguousarray(img)

        c_img = self._convert_image(img_c)
        self.libc.apriltag_detector_detect.restype = ctypes.POINTER(_ZArray)

        dets_ptr = None  # for finally‑block
        try:
            dets_ptr = self.libc.apriltag_detector_detect(self._detector, c_img)
            return self._process_detections(
                dets_ptr, estimate_tag_pose, camera_params, tag_size
            )
        finally:
            self.libc.image_u8_destroy(c_img)
            if dets_ptr:
                self.libc.apriltag_detections_destroy(dets_ptr)

    # ------------- internal helpers ------------------

    def _convert_image(self, img: np.ndarray):
        height, width = img.shape
        self.libc.image_u8_create.restype = ctypes.POINTER(_ImageU8)
        c_img = self.libc.image_u8_create(width, height)
        if not c_img:
            raise MemoryError("image_u8_create returned NULL")
        # Fast copy taking potential stride into account
        _image_u8_get_array(c_img)[:, :width] = img
        return c_img

    def _process_detections(
        self,
        dets_ptr: ctypes.POINTER(_ZArray),
        estimate_tag_pose: bool,
        camera_params: Optional[Sequence[float]],
        tag_size: Optional[float],
    ) -> List[Detection]:
        num = dets_ptr.contents.size
        results: List[Detection] = []
        apriltag_ptr = ctypes.POINTER(_ApriltagDetection)()

        for i in range(num):
            _zarray_get(dets_ptr, i, ctypes.byref(apriltag_ptr))
            det = apriltag_ptr.contents

            d = Detection()
            d.tag_family = ctypes.string_at(det.family.contents.name).decode()
            d.tag_id = det.id
            d.hamming = det.hamming
            d.decision_margin = det.decision_margin
            d.homography = _matd_get_array(det.H).copy()
            d.center = np.ctypeslib.as_array(det.c, shape=(2,)).copy()
            d.corners = np.ctypeslib.as_array(det.p, shape=(4, 2)).copy()

            if estimate_tag_pose:
                if camera_params is None:
                    raise ValueError("camera_params must be provided when estimate_tag_pose=True")
                if tag_size is None:
                    raise ValueError("tag_size must be provided when estimate_tag_pose=True")
                fx, fy, cx, cy = map(float, camera_params)
                info = _ApriltagDetectionInfo(
                    det=apriltag_ptr,
                    tagsize=float(tag_size),
                    fx=fx,
                    fy=fy,
                    cx=cx,
                    cy=cy,
                )
                pose = _ApriltagPose()
                err = self.libc.estimate_tag_pose(ctypes.byref(info), ctypes.byref(pose))
                d.pose_R = _matd_get_array(pose.R).copy()
                d.pose_t = _matd_get_array(pose.t).copy()
                d.pose_err = err

            results.append(d)
        return results


# ---------------------------------------------------------------------------
# Optional self‑test when run directly (unchanged, but with safe YAML load)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import time

    try:
        import cv2
        from cv2 import imshow  # noqa: F401 — validate availability
    except ImportError as e:
        sys.exit("OpenCV is required for the demo – library not found: " + str(e))

    try:
        import yaml
    except ImportError:
        sys.exit("PyYAML is required for the demo – install with `pip install pyyaml`. ")

    TEST_PATH = "test"
    with open(os.path.join(TEST_PATH, "test_info.yaml"), "r", encoding="utf-8") as fh:
        params = yaml.safe_load(fh)

    with Detector(families="tag36h11", nthreads=1, quad_decimate=1.0) as det:
        # Sample image demo ----------------------------------------------------
        img = cv2.imread(os.path.join(TEST_PATH, params["sample_test"]["file"]), cv2.IMREAD_GRAYSCALE)
        K = np.asarray(params["sample_test"]["K"]).reshape(3, 3)
        cam_params = (K[0, 0], K[1, 1], K[0, 2], K[1, 2])
        tags = det.detect(img, True, cam_params, params["sample_test"]["tag_size"])
        print("Detected:", tags)

        # Timing demo over rotation images ------------------------------------
        t_acc, n = 0.0, 0
        for fname in params["rotation_test"]["files"]:
            path = os.path.join(TEST_PATH, fname)
            if not os.path.isfile(path):
                continue
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            start = time.time()
            det.detect(img)
            t_acc += time.time() - start
            n += 1
        if n:
            print(f"Avg detection time: {t_acc / n:.4f} s ({n} images)")
