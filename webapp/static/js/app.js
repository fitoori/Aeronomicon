const engageToggleBtn = document.getElementById("engage-toggle-btn");
const clearBtn = document.getElementById("clear-btn");
const logView = document.getElementById("log-view");
const telemetryLogToggle = document.getElementById("telemetry-log-toggle");
const loadingScreen = document.getElementById("loading-screen");
const offlineScreen = document.getElementById("offline-screen");
const rebootScreen = document.getElementById("reboot-screen");
const connectionPill = document.getElementById("connection-pill");
const logoMenuButton = document.getElementById("logo-menu-button");
const logoMenu = document.getElementById("logo-menu");
const rebootBtn = document.getElementById("reboot-btn");
const monochromeToggle = document.getElementById("monochrome-toggle");
const lteBanner = document.getElementById("lte-banner");
const onicsState = document.querySelector("#onics-state .status-strip__value");
const onicsRuntime = document.getElementById("onics-runtime");
const loginModal = document.getElementById("login-modal");
const loginMessage = document.getElementById("login-message");
const loginCommand = document.getElementById("login-command");
const loginMoshCommandWrap = document.getElementById("login-mosh-command-wrap");
const loginMoshLabel = document.getElementById("login-mosh-label");
const loginMoshCommand = document.getElementById("login-mosh-command");
const loginSetupCommand = document.getElementById("login-setup-command");
const loginKeygenCommand = document.getElementById("login-keygen-command");
const loginOpenBtn = document.getElementById("login-open-btn");
const loginMoshCopyBtn = document.getElementById("login-mosh-copy-btn");
const loginDismissBtn = document.getElementById("login-dismiss-btn");
const serviceRestartButtons = document.querySelectorAll("[data-service-restart]");
const serviceLogCards = document.querySelectorAll("[data-service-log]");

const tailscaleStatus = document.getElementById("tailscale-status");
const tailscaleBackend = document.getElementById("tailscale-backend");
const tailscaleMeta = document.getElementById("tailscale-meta");
const tailscaleError = document.getElementById("tailscale-error");
const dnsStatus = document.getElementById("dns-status");
const dnsMeta = document.getElementById("dns-meta");
const tcpStatus = document.getElementById("tcp-status");
const tcpMeta = document.getElementById("tcp-meta");
const trackingConfidence = document.getElementById("tracking-confidence");
const trackingMeta = document.getElementById("tracking-meta");
const startupStage = document.getElementById("startup-stage");
const startupMeta = document.getElementById("startup-meta");
const startupFails = document.getElementById("startup-fails");
const autopilotAlert = document.getElementById("autopilot-alert");
const autopilotMeta = document.getElementById("autopilot-meta");

const arducopterStatus = document.getElementById("arducopter-status");
const arducopterMeta = document.getElementById("arducopter-meta");
const mavproxyStatus = document.getElementById("mavproxy-status");
const mavproxyMeta = document.getElementById("mavproxy-meta");
const uplinkStatus = document.getElementById("uplink-status");
const uplinkMeta = document.getElementById("uplink-meta");
const uplinkSignalValue = document.getElementById("uplink-signal-value");
const systemLoad = document.getElementById("system-load");
const systemLoadMeta = document.getElementById("system-load-meta");
const systemLoadGraph = document.getElementById("system-load-graph");
const systemMemory = document.getElementById("system-memory");
const systemMemoryMeta = document.getElementById("system-memory-meta");
const systemMemoryGraph = document.getElementById("system-memory-graph");
const systemDisk = document.getElementById("system-disk");
const systemDiskMeta = document.getElementById("system-disk-meta");
const headerLoad = document.getElementById("header-load");
const headerLoadGraph = document.getElementById("header-load-graph");
const headerMemory = document.getElementById("header-memory");
const headerMemoryGraph = document.getElementById("header-memory-graph");
const headerDisk = document.getElementById("header-disk");

const urlParams = new URLSearchParams(window.location.search);
const dryRunParam = (urlParams.get("dry_run") || "").toLowerCase();
const loadOnlyParam = (urlParams.get("load_only") || "").toLowerCase();
const isDryRun =
  document.body?.dataset.dryRun === "true" ||
  ["1", "true", "yes", "on"].includes(dryRunParam);
const isLoadOnly =
  document.body?.dataset.loadOnly === "true" ||
  ["1", "true", "yes", "on"].includes(loadOnlyParam);

const MIN_TELEMETRY_LOG_HEIGHT = 0;

let engageToggleAction = null;
let rebootPending = false;
let rebootAwaitingLoss = false;
let isMonochromeMode = false;
let connectionStatus = "ok";
let latestSnapshot = null;
let telemetryLogExpanded = false;

function setLoadingState(enabled) {
  document.body?.classList.toggle("is-loading", enabled);
}

function dismissLoadingScreen() {
  if (isLoadOnly) {
    return;
  }
  if (loadingScreen && !loadingScreen.classList.contains("hidden")) {
    loadingScreen.classList.add("hidden");
    setLoadingState(false);
    setTimeout(() => loadingScreen.remove(), 600);
  }
}

function setLogoMenuOpen(open) {
  if (!logoMenuButton || !logoMenu) {
    return;
  }
  logoMenuButton.setAttribute("aria-expanded", open ? "true" : "false");
  logoMenu.setAttribute("aria-hidden", open ? "false" : "true");
  logoMenu.classList.toggle("is-open", open);
}

function setRebootOverlay(visible) {
  if (!rebootScreen) {
    return;
  }
  rebootScreen.classList.toggle("hidden", !visible);
  rebootScreen.setAttribute("aria-hidden", visible ? "false" : "true");
  document.body.classList.toggle("is-rebooting", visible);
}

function setLteBanner(active) {
  if (!lteBanner) {
    return;
  }
  lteBanner.classList.toggle("is-visible", active);
  lteBanner.setAttribute("aria-hidden", active ? "false" : "true");
}

function updateTelemetryLogHeight() {
  if (!telemetryLogExpanded || !logView) {
    return;
  }
  const documentElement = document.documentElement;
  const applyHeight = () => {
    const rect = logView.getBoundingClientRect();
    const slack = documentElement.clientHeight - documentElement.scrollHeight;
    document.body?.classList.toggle("is-telemetry-log-compact", slack < 0);
    const height = Math.max(MIN_TELEMETRY_LOG_HEIGHT, Math.floor(rect.height + slack));
    documentElement.style.setProperty("--telemetry-log-height", `${height}px`);
  };
  applyHeight();
  requestAnimationFrame(() => {
    if (!telemetryLogExpanded || !logView) {
      return;
    }
    const slack = documentElement.clientHeight - documentElement.scrollHeight;
    if (Math.abs(slack) > 1) {
      applyHeight();
    }
  });
}

function setTelemetryLogExpanded(expanded) {
  telemetryLogExpanded = expanded;
  document.body?.classList.toggle("is-telemetry-log-expanded", expanded);
  telemetryLogToggle?.setAttribute("aria-expanded", expanded ? "true" : "false");
  if (!expanded) {
    document.documentElement.style.removeProperty("--telemetry-log-height");
    document.body?.classList.remove("is-telemetry-log-compact");
  }
  updateTelemetryLogHeight();
}

function setMonochromeMode(enabled) {
  isMonochromeMode = enabled;
  document.body.classList.toggle("is-monochrome", enabled);
  if (monochromeToggle) {
    monochromeToggle.setAttribute("aria-pressed", enabled ? "true" : "false");
    monochromeToggle.textContent = enabled ? "Disable monochrome mode" : "Enable monochrome mode";
  }
  if (connectionPill) {
    const pillStyles = getConnectionPillStyles(connectionStatus);
    connectionPill.style.borderColor = pillStyles.borderColor;
    connectionPill.style.color = pillStyles.color;
  }
  try {
    window.localStorage.setItem("monochrome-mode", enabled ? "true" : "false");
  } catch (err) {
    console.warn("Unable to store monochrome preference", err);
  }
  drawAllLoadGraphs();
  drawAllMemoryGraphs();
}

function getGraphColor(token, fallback) {
  const value = getComputedStyle(document.body).getPropertyValue(token).trim();
  return value || fallback;
}

function getGraphLineWidth() {
  return isMonochromeMode ? 3 : 2;
}

function getConnectionPillStyles(status) {
  if (isMonochromeMode) {
    const accent = getGraphColor("--accent", "#000000");
    const muted = getGraphColor("--muted", "#1a1a1a");
    if (status === "ok") {
      return { borderColor: accent, color: accent };
    }
    if (status === "degraded") {
      return { borderColor: muted, color: muted };
    }
    return { borderColor: "#4d4d4d", color: "#4d4d4d" };
    if (status === "ok") {
      return { borderColor: "#ffffff", color: "#ffffff" };
    }
    if (status === "degraded") {
      return { borderColor: "#b5b5b5", color: "#b5b5b5" };
    }
    return { borderColor: "#666666", color: "#e0e0e0" };
  }
  if (status === "ok") {
    return { borderColor: "rgba(74,222,128,0.5)", color: "#4ade80" };
  }
  if (status === "degraded") {
    return { borderColor: "rgba(245,158,11,0.6)", color: "#f59e0b" };
  }
  return { borderColor: "rgba(249,115,22,0.6)", color: "#f97316" };
}

const cards = {
  tailscale: document.getElementById("tailscale-card"),
  dns: document.getElementById("dns-card"),
  tcp: document.getElementById("tcp-card"),
  tracking: document.getElementById("tracking-card"),
  startup: document.getElementById("startup-card"),
  autopilot: document.getElementById("autopilot-card"),
  arducopter: document.getElementById("arducopter-card"),
  mavproxy: document.getElementById("mavproxy-card"),
  uplink: document.getElementById("uplink-card"),
  systemLoad: document.getElementById("system-load-card"),
  systemMemory: document.getElementById("system-memory-card"),
  systemDisk: document.getElementById("system-disk-card"),
};

function setOfflineState(isOffline) {
  if (isDryRun) {
    document.body.classList.remove("is-offline");
    if (offlineScreen) {
      offlineScreen.setAttribute("aria-hidden", "true");
    }
    return;
  }
  document.body.classList.toggle("is-offline", isOffline);
  if (offlineScreen) {
    offlineScreen.setAttribute("aria-hidden", isOffline ? "false" : "true");
  }
}

function setCardState(card, state) {
  if (!card) {
    return;
  }
  card.classList.remove("health-card--ok", "health-card--warn", "health-card--danger");
  if (state === "ok") {
    card.classList.add("health-card--ok");
  } else if (state === "warn") {
    card.classList.add("health-card--warn");
  } else if (state === "danger") {
    card.classList.add("health-card--danger");
  }
}

function pulseCard(card) {
  if (!card) {
    return;
  }
  card.classList.remove("health-card--pulse");
  void card.offsetWidth;
  card.classList.add("health-card--pulse");
}

function formatAge(value) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) {
    return "n/a";
  }
  const seconds = Math.max(0, Number(value));
  const rounded = seconds < 10 ? seconds.toFixed(1) : Math.round(seconds).toString();
  return `${rounded} seconds ago`;
}

function resolveAgeSeconds(ageValue, timestampIso) {
  if (timestampIso) {
    const computed = ageSecondsFromTimestamp(timestampIso);
    if (computed !== null) {
      return computed;
    }
  }
  if (ageValue !== null && ageValue !== undefined && Number.isFinite(Number(ageValue))) {
    return Number(ageValue);
  }
  return null;
}

function formatSeconds(value) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) {
    return "n/a";
  }
  const seconds = Math.max(0, Math.floor(Number(value)));
  return `${seconds} seconds`;
}

function renderOnicsRuntime(onics) {
  if (!onicsRuntime || !onics) {
    return;
  }
  const lastOutputAge = resolveAgeSeconds(onics.last_output_age_s, onics.last_output_iso);
  onicsRuntime.textContent = `SSH ${onics.ssh_connected ? "connected" : "offline"} · last output ${formatAge(
    lastOutputAge
  )}`;
  setAgeSeverity(onicsRuntime, lastOutputAge);
}

function normalizeTimestamp(ts) {
  if (!ts) {
    return null;
  }
  const trimmed = String(ts).trim();
  if (!trimmed) {
    return null;
  }
  return trimmed.replace(/([+-]\d{2})(\d{2})$/, "$1:$2");
}

function timestampToMs(ts) {
  const normalized = normalizeTimestamp(ts);
  if (!normalized) {
    return null;
  }
  const parsed = Date.parse(normalized);
  if (Number.isNaN(parsed)) {
    return null;
  }
  return parsed;
}

function ageSecondsFromTimestamp(ts) {
  const ms = timestampToMs(ts);
  if (ms === null) {
    return null;
  }
  return Math.max(0, (Date.now() - ms) / 1000);
}

function setAgeSeverity(element, ageSeconds) {
  if (!element) {
    return;
  }
  element.classList.remove("age--yellow", "age--orange", "age--red");
  if (ageSeconds === null || ageSeconds === undefined || !Number.isFinite(Number(ageSeconds))) {
    return;
  }
  const seconds = Number(ageSeconds);
  if (seconds >= 30) {
    element.classList.add("age--red");
  } else if (seconds >= 20) {
    element.classList.add("age--orange");
  } else if (seconds >= 10) {
    element.classList.add("age--yellow");
  }
}

function formatServiceStatus(status) {
  if (!status) {
    return "UNKNOWN";
  }
  return String(status).toUpperCase();
}

function serviceStateToCard(status) {
  if (!status) {
    return null;
  }
  const normalized = String(status).toLowerCase();
  if (normalized === "active") {
    return "ok";
  }
  if (["activating", "reloading", "deactivating"].includes(normalized)) {
    return "warn";
  }
  if (["failed", "inactive"].includes(normalized)) {
    return "danger";
  }
  if (["missing", "unknown"].includes(normalized)) {
    return "warn";
  }
  return null;
}

function formatBytes(value) {
  if (value === null || value === undefined) {
    return "n/a";
  }
  const units = ["B", "KB", "MB", "GB", "TB"];
  let idx = 0;
  let num = value;
  while (num >= 1024 && idx < units.length - 1) {
    num /= 1024;
    idx += 1;
  }
  return `${num.toFixed(num >= 100 || idx === 0 ? 0 : 1)} ${units[idx]}`;
}

function formatUptime(seconds) {
  return formatSeconds(seconds);
}

const loadHistory = [];
const loadHistoryWindowMs = 5 * 60 * 1000;
let loadHistoryCpuCount = null;
const loadGraphs = [systemLoadGraph, headerLoadGraph]
  .filter(Boolean)
  .map((canvas) => ({
    canvas,
    context: canvas.getContext("2d"),
  }))
  .filter((graph) => graph.context);

function pruneLoadHistory(nowMs) {
  const cutoff = nowMs - loadHistoryWindowMs;
  while (loadHistory.length && loadHistory[0].ts < cutoff) {
    loadHistory.shift();
  }
}

const memoryHistory = [];
const memoryHistoryWindowMs = 5 * 60 * 1000;
let memoryHistoryTotal = null;
const memoryGraphs = [systemMemoryGraph, headerMemoryGraph]
  .filter(Boolean)
  .map((canvas) => ({
    canvas,
    context: canvas.getContext("2d"),
  }))
  .filter((graph) => graph.context);

function pruneMemoryHistory(nowMs) {
  const cutoff = nowMs - memoryHistoryWindowMs;
  while (memoryHistory.length && memoryHistory[0].ts < cutoff) {
    memoryHistory.shift();
  }
}

function resizeGraph(graph) {
  if (!graph?.canvas || !graph?.context) {
    return;
  }
  const { clientWidth, clientHeight } = graph.canvas;
  if (!clientWidth || !clientHeight) {
    return;
  }
  const dpr = window.devicePixelRatio || 1;
  graph.canvas.width = Math.round(clientWidth * dpr);
  graph.canvas.height = Math.round(clientHeight * dpr);
  graph.context.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function drawLoadGraph(graph) {
  if (!graph?.canvas || !graph?.context) {
    return;
  }
  const now = Date.now();
  pruneLoadHistory(now);
  resizeGraph(graph);

  const width = graph.canvas.clientWidth;
  const height = graph.canvas.clientHeight;
  if (!width || !height) {
    return;
  }

  graph.context.clearRect(0, 0, width, height);

  if (!loadHistory.length) {
    return;
  }

  const maxLoad = Math.max(
    1,
    loadHistoryCpuCount || 0,
    ...loadHistory.map((entry) => entry.value)
  );
  const paddedMax = maxLoad * 1.1;
  const start = now - loadHistoryWindowMs;

  graph.context.strokeStyle = getGraphColor("--graph-load", "rgba(74, 222, 128, 0.85)");
  graph.context.lineWidth = getGraphLineWidth();
  graph.context.beginPath();

  loadHistory.forEach((entry, index) => {
    const x = ((entry.ts - start) / loadHistoryWindowMs) * width;
    const y = height - Math.min(entry.value / paddedMax, 1) * height;
    if (index === 0) {
      graph.context.moveTo(x, y);
    } else {
      graph.context.lineTo(x, y);
    }
  });

  graph.context.stroke();
}

function drawAllLoadGraphs() {
  loadGraphs.forEach((graph) => {
    drawLoadGraph(graph);
  });
}

function drawMemoryGraph(graph) {
  if (!graph?.canvas || !graph?.context) {
    return;
  }
  const now = Date.now();
  pruneMemoryHistory(now);
  resizeGraph(graph);

  const width = graph.canvas.clientWidth;
  const height = graph.canvas.clientHeight;
  if (!width || !height) {
    return;
  }

  graph.context.clearRect(0, 0, width, height);

  if (!memoryHistory.length) {
    return;
  }

  const historyMax = Math.max(1, ...memoryHistory.map((entry) => entry.value));
  const totalMax = Number.isFinite(memoryHistoryTotal) ? Math.max(memoryHistoryTotal, 1) : historyMax;
  const paddedMax = totalMax * 1.05;
  const start = now - memoryHistoryWindowMs;

  graph.context.strokeStyle = getGraphColor("--graph-memory", "rgba(96, 165, 250, 0.85)");
  graph.context.lineWidth = getGraphLineWidth();
  graph.context.beginPath();

  memoryHistory.forEach((entry, index) => {
    const x = ((entry.ts - start) / memoryHistoryWindowMs) * width;
    const y = height - Math.min(entry.value / paddedMax, 1) * height;
    if (index === 0) {
      graph.context.moveTo(x, y);
    } else {
      graph.context.lineTo(x, y);
    }
  });

  graph.context.stroke();
}

function drawAllMemoryGraphs() {
  memoryGraphs.forEach((graph) => {
    drawMemoryGraph(graph);
  });
}

function recordLoadPoint(value, cpuCount) {
  if (!Number.isFinite(value)) {
    return;
  }
  if (Number.isFinite(cpuCount)) {
    loadHistoryCpuCount = cpuCount;
  }
  loadHistory.push({ ts: Date.now(), value: Number(value) });
  pruneLoadHistory(Date.now());
  drawAllLoadGraphs();
}

function recordMemoryPoint(value, total) {
  if (!Number.isFinite(value)) {
    return;
  }
  if (Number.isFinite(total)) {
    memoryHistoryTotal = Number(total);
  }
  memoryHistory.push({ ts: Date.now(), value: Number(value) });
  pruneMemoryHistory(Date.now());
  drawAllMemoryGraphs();
}


function formatSystemSummary(system) {
  if (!system) {
    return "System telemetry unavailable.";
  }
  const load =
    system.load_1 !== null && system.load_5 !== null && system.load_15 !== null
      ? `${system.load_1.toFixed(2)} / ${system.load_5.toFixed(2)} / ${system.load_15.toFixed(2)}`
      : "n/a";
  const mem =
    system.mem_available_bytes !== null && system.mem_available_bytes !== undefined
      ? `${formatBytes(system.mem_available_bytes)} free`
      : "n/a";
  const disk =
    system.disk_free_bytes !== null && system.disk_free_bytes !== undefined
      ? `${formatBytes(system.disk_free_bytes)} free`
      : "n/a";
  return `Load ${load} · Mem ${mem} · Disk ${disk}`;
}

function formatHeaderReadout(meta, system) {
  const host = meta?.hostname ? `Host ${meta.hostname}` : "Host unknown";
  const sshUser = meta?.ssh_user ? meta.ssh_user : "user";
  const sshPort = meta?.ssh_port ? meta.ssh_port : "22";
  const sshTarget = meta?.hostname ? `${sshUser}@${meta.hostname}:${sshPort}` : `${sshUser}@host:${sshPort}`;
  const serverAge = meta?.server_time_iso ? ageSecondsFromTimestamp(meta.server_time_iso) : null;
  const serverTime = serverAge !== null ? `Server ${formatAge(serverAge)}` : null;
  const systemSummary = formatSystemSummary(system);
  return [host, `SSH ${sshTarget}`, serverTime, systemSummary].filter(Boolean).join(" · ");
}

function updateEngageToggle(onics) {
  if (!engageToggleBtn) {
    return;
  }
  const canEngage = onics.state === "IDLE" || onics.state === "ERROR" || onics.state === "LOS";
  const canDisengage = onics.state === "RUNNING" || onics.state === "STARTING";
  let label = "ENGAGE";
  let action = "/api/engage";
  let style = "btn--engage";

  if (canDisengage) {
    label = "DISENGAGE";
    action = "/api/disengage";
    style = "btn--disengage";
  } else if (onics.state === "STOPPING") {
    label = "STOPPING";
    action = null;
    style = "btn--disengage";
  }

  engageToggleBtn.textContent = label;
  engageToggleBtn.classList.remove("btn--engage", "btn--disengage");
  engageToggleBtn.classList.add(style);
  engageToggleBtn.disabled = !(
    (action === "/api/engage" && canEngage) ||
    (action === "/api/disengage" && canDisengage)
  );
  engageToggleAction = action;
}

const telemetryState = {
  trackingConfidence: null,
  trackingTimestamp: null,
  startupStage: null,
  startupTimestamp: null,
  autopilotLevel: null,
  autopilotMessage: null,
  autopilotTimestamp: null,
  lastTelemetryTimestamp: null,
};

const TELEMETRY_HEARTBEAT_S = 10;
let lastEngagedState = null;

const startupStages = [
  { regex: /Connecting to Realsense camera/i, label: "Connecting Realsense" },
  { regex: /Realsense connected/i, label: "Realsense online" },
  { regex: /Connecting to vehicle/i, label: "Connecting vehicle" },
  { regex: /Vehicle connected/i, label: "Vehicle linked" },
  { regex: /Starting main loop/i, label: "Main loop starting" },
  { regex: /Using stereo fisheye cameras/i, label: "Stereo cameras online" },
  { regex: /Set EKF home/i, label: "EKF home set" },
];

function formatTimestamp(ts) {
  const ageSeconds = ageSecondsFromTimestamp(ts);
  if (ageSeconds === null) {
    return { text: "Awaiting telemetry.", ageSeconds: null };
  }
  return { text: `Last update ${formatAge(ageSeconds)}`, ageSeconds };
}

function formatLogTimestamp(line) {
  const timestamp = parseLogTimestamp(line);
  const ageSeconds = ageSecondsFromTimestamp(timestamp);
  if (ageSeconds === null) {
    return line;
  }
  return line.replace(/^\[[^\]]+\]/, `[${formatAge(ageSeconds)}]`);
}

function updateTelemetryCards() {
  const telemetryAgeSeconds = ageSecondsFromTimestamp(telemetryState.lastTelemetryTimestamp);
  const telemetryFresh =
    telemetryAgeSeconds !== null && telemetryAgeSeconds <= TELEMETRY_HEARTBEAT_S;

  if (trackingConfidence && trackingMeta) {
    trackingConfidence.textContent =
      telemetryState.trackingConfidence || "Awaiting telemetry";
    if (telemetryState.trackingTimestamp) {
      const trackingAge = formatTimestamp(telemetryState.trackingTimestamp);
      trackingMeta.textContent = trackingAge.text;
      setAgeSeverity(trackingMeta, trackingAge.ageSeconds);
    } else {
      trackingMeta.textContent = "No tracking updates yet.";
      setAgeSeverity(trackingMeta, null);
    }
    setCardState(cards.tracking, telemetryState.trackingConfidence ? "ok" : null);
  }

  if (startupStage && startupMeta) {
    const startupAgeSeconds = ageSecondsFromTimestamp(telemetryState.startupTimestamp);
    const startupDisplay =
      telemetryFresh && (!telemetryState.startupStage || startupAgeSeconds > TELEMETRY_HEARTBEAT_S)
        ? "RUNNING"
        : telemetryState.startupStage;
    const startupTimestamp = telemetryFresh
      ? telemetryState.lastTelemetryTimestamp
      : telemetryState.startupTimestamp;
    startupStage.textContent = startupDisplay || "Awaiting telemetry";
    if (startupTimestamp) {
      const startupAge = formatTimestamp(startupTimestamp);
      startupMeta.textContent = startupAge.text;
      setAgeSeverity(startupMeta, startupAge.ageSeconds);
    } else {
      startupMeta.textContent = "No startup milestones yet.";
      setAgeSeverity(startupMeta, null);
    }
    setCardState(cards.startup, startupDisplay ? "ok" : null);
  }

  if (autopilotAlert && autopilotMeta) {
    const autopilotAgeSeconds = ageSecondsFromTimestamp(telemetryState.autopilotTimestamp);
    const autopilotStale = autopilotAgeSeconds === null || autopilotAgeSeconds > TELEMETRY_HEARTBEAT_S;
    const autopilotDisplay =
      telemetryFresh && autopilotStale ? "NO ALERTS" : telemetryState.autopilotMessage;
    const autopilotLevel = autopilotStale ? null : telemetryState.autopilotLevel;
    const autopilotTimestamp = telemetryFresh
      ? telemetryState.lastTelemetryTimestamp
      : telemetryState.autopilotTimestamp;
    autopilotAlert.textContent = autopilotDisplay || "Awaiting telemetry";
    if (autopilotTimestamp) {
      const autopilotAge = formatTimestamp(autopilotTimestamp);
      autopilotMeta.textContent = autopilotAge.text;
      setAgeSeverity(autopilotMeta, autopilotAge.ageSeconds);
    } else {
      autopilotMeta.textContent = "No autopilot alerts yet.";
      setAgeSeverity(autopilotMeta, null);
    }
    if (autopilotLevel === "CRITICAL") {
      setCardState(cards.autopilot, "danger");
    } else if (autopilotLevel === "WARNING") {
      setCardState(cards.autopilot, "warn");
    } else if (autopilotDisplay) {
      setCardState(cards.autopilot, telemetryFresh && autopilotStale ? "ok" : null);
    } else {
      setCardState(cards.autopilot, null);
    }
  }
}

function parseLogTimestamp(line) {
  const match = line.match(/^\[([^\]]+)\]/);
  return match ? match[1] : null;
}

function parseTelemetry(line) {
  const cleaned = line.replace(/^\[[^\]]+\]\s*/, "").trim();
  const timestamp = parseLogTimestamp(line);
  let trackingUpdated = false;
  let startupUpdated = false;
  let autopilotUpdated = false;

  const trackingMatch = cleaned.match(/Tracking confidence:\s*(.+)$/i);
  if (trackingMatch) {
    telemetryState.trackingConfidence = trackingMatch[1].trim();
    telemetryState.trackingTimestamp = timestamp;
    trackingUpdated = true;
  }

  for (const stage of startupStages) {
    if (stage.regex.test(cleaned)) {
      telemetryState.startupStage = stage.label;
      telemetryState.startupTimestamp = timestamp;
      startupUpdated = true;
      break;
    }
  }

  const autopilotMatch = cleaned.match(/^([A-Z]+):autopilot:(.+)$/);
  if (autopilotMatch) {
    telemetryState.autopilotLevel = autopilotMatch[1].trim();
    telemetryState.autopilotMessage = autopilotMatch[2].trim();
    telemetryState.autopilotTimestamp = timestamp;
    autopilotUpdated = true;
  }

  if (trackingUpdated || startupUpdated || autopilotUpdated) {
    telemetryState.lastTelemetryTimestamp = timestamp;
  }

  updateTelemetryCards();

  if (trackingUpdated) {
    pulseCard(cards.tracking);
  }
  if (startupUpdated) {
    pulseCard(cards.startup);
  }
  if (autopilotUpdated) {
    pulseCard(cards.autopilot);
  }
}

function resetTelemetryState() {
  telemetryState.trackingConfidence = null;
  telemetryState.trackingTimestamp = null;
  telemetryState.startupStage = null;
  telemetryState.startupTimestamp = null;
  telemetryState.autopilotLevel = null;
  telemetryState.autopilotMessage = null;
  telemetryState.autopilotTimestamp = null;
  telemetryState.lastTelemetryTimestamp = null;
  updateTelemetryCards();
}

function setLoginPrompt(meta, onics) {
  if (
    !loginModal ||
    !loginMessage ||
    !loginCommand ||
    !loginMoshCommandWrap ||
    !loginMoshLabel ||
    !loginMoshCommand ||
    !loginSetupCommand ||
    !loginKeygenCommand ||
    !loginOpenBtn ||
    !loginMoshCopyBtn
  ) {
    return;
  }
  if (onics?.login_required) {
    const sshUser = meta.ssh_user || "pi";
    const sshCommand = `ssh -p ${meta.ssh_port} ${sshUser}@${meta.hostname}`;
    const sshUri = `ssh://${sshUser}@${meta.hostname}:${meta.ssh_port}`;
    const moshCommand = `mosh --ssh="ssh -p ${meta.ssh_port}" ${sshUser}@${meta.hostname}`;
    const sshSetupCommand = `ssh-copy-id -p ${meta.ssh_port} ${sshUser}@${meta.hostname}`;
    const sshKeygenCommand = 'ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N ""';
    loginMessage.textContent =
      onics.login_message ||
      "SSH authentication is missing. Complete an interactive login to continue.";
    const showMosh = Boolean(meta.mosh_available);
    loginCommand.textContent = showMosh ? moshCommand : sshCommand;
    loginMoshLabel.textContent = showMosh ? "SSH fallback" : "Mosh";
    loginMoshCommand.textContent = showMosh ? sshCommand : moshCommand;
    loginSetupCommand.textContent = sshSetupCommand;
    loginKeygenCommand.textContent = sshKeygenCommand;
    const copyCommand = (button, command, promptLabel) => {
      const originalLabel = button.textContent;
      const finish = (label) => {
        button.textContent = label;
        window.setTimeout(() => {
          button.textContent = originalLabel;
        }, 1500);
      };
      if (navigator?.clipboard?.writeText) {
        navigator.clipboard
          .writeText(command)
          .then(() => finish("Copied!"))
          .catch(() => {
            window.prompt(promptLabel, command);
            finish("Copy command");
          });
      } else {
        window.prompt(promptLabel, command);
        finish("Copy command");
      }
    };
    loginOpenBtn.textContent = showMosh ? "Copy Mosh Command" : "Open SSH Login";
    loginOpenBtn.onclick = () => {
      if (showMosh) {
        copyCommand(loginOpenBtn, moshCommand, "Copy mosh command:");
      } else {
        window.location.href = sshUri;
      }
    };
    loginMoshCopyBtn.onclick = () => {
      copyCommand(loginMoshCopyBtn, moshCommand, "Copy mosh command:");
    };
    loginMoshCommandWrap.classList.toggle("is-hidden", !showMosh);
    loginMoshCopyBtn.classList.add("is-hidden");
    loginModal.classList.add("is-visible");
    loginModal.setAttribute("aria-hidden", "false");
  } else {
    loginMoshCommandWrap.classList.add("is-hidden");
    loginMoshCopyBtn.classList.add("is-hidden");
    loginModal.classList.remove("is-visible");
    loginModal.setAttribute("aria-hidden", "true");
  }
}

function updateSnapshot(snapshot, options = {}) {
  if (!snapshot) {
    return;
  }
  latestSnapshot = snapshot;
  if (options.fromStream) {
    setOfflineState(false);
  }

  dismissLoadingScreen();

  const { meta, health, onics, autopilot } = snapshot;
  setLoginPrompt(meta, onics);
  if (onics) {
    if (lastEngagedState === null) {
      lastEngagedState = onics.engaged;
    } else if (lastEngagedState && !onics.engaged) {
      resetTelemetryState();
    }
    lastEngagedState = onics.engaged;
  }

  const linkDegraded =
    health?.los ||
    health?.stale ||
    !health?.tailscale_ok ||
    !health?.dns_ok ||
    !health?.tcp_ok;
  const linkDown =
    !health?.tailscale_ok || !health?.dns_ok || !health?.tcp_ok || health?.los || health?.stale;
  if (rebootPending) {
    if (rebootAwaitingLoss && linkDown) {
      rebootAwaitingLoss = false;
    }
    if (!rebootAwaitingLoss && !linkDown) {
      rebootPending = false;
    }
  }
  setRebootOverlay(rebootPending);
  connectionPill.textContent = health.los
    ? "LINK: LOS"
    : linkDegraded
      ? "LINK: DEGRADED"
      : "LINK: OK";
  connectionStatus = health.los ? "los" : linkDegraded ? "degraded" : "ok";
  const pillStyles = getConnectionPillStyles(connectionStatus);
  connectionPill.style.borderColor = pillStyles.borderColor;
  connectionPill.style.color = pillStyles.color;

  if (onicsState) {
    onicsState.textContent = onics.state;
  }
  renderOnicsRuntime(onics);
  if (startupFails) {
    const restartFails = Number.isFinite(onics.restart_failures)
      ? onics.restart_failures
      : 0;
    startupFails.textContent = `${restartFails}`;
  }

  if (tailscaleStatus) {
    tailscaleStatus.textContent = health.tailscale_ok ? "RUNNING" : "OFFLINE";
  }
  if (tailscaleBackend) {
    const backend = health.tailscale_backend_state || "UNKNOWN";
    tailscaleBackend.textContent = `Backend: ${backend}`;
  }
  if (tailscaleMeta) {
    if (health.tailscale_present === false) {
      tailscaleMeta.textContent = "Tailscale not installed.";
    } else if (health.tailscale_ok) {
      tailscaleMeta.textContent = "Tailnet link healthy.";
    } else {
      tailscaleMeta.textContent = "Tailnet link unavailable.";
    }
  }
  if (tailscaleError) {
    tailscaleError.textContent = health.tailscale_error || "No errors reported.";
  }
  setCardState(cards.tailscale, health.tailscale_ok ? "ok" : "danger");

  if (dnsStatus) {
    dnsStatus.textContent = health.dns_ok ? "RESOLVED" : "UNRESOLVED";
  }
  if (dnsMeta) {
    dnsMeta.textContent = health.dns_ok
      ? `IPs: ${health.dns_ips.join(", ")}`
      : health.dns_error || "Awaiting DNS";
  }
  setCardState(cards.dns, health.dns_ok ? "ok" : "warn");

  if (tcpStatus) {
    tcpStatus.textContent = health.tcp_ok ? "CONNECTED" : "BLOCKED";
  }
  if (tcpMeta) {
    tcpMeta.textContent = health.tcp_ok
      ? `Probe ${health.tcp_ip}:${meta.ssh_port} ok`
      : health.tcp_error || "Awaiting TCP";
  }
  setCardState(cards.tcp, health.tcp_ok ? "ok" : "danger");

  if (autopilot && autopilot.services) {
    const services = autopilot.services;
    const arducopter = services.arducopter || {};
    const mavproxy = services.mavproxy || {};
    const uplink = services.uplink || {};

    if (arducopterStatus) {
      arducopterStatus.textContent = formatServiceStatus(arducopter.status);
    }
    if (arducopterMeta) {
      arducopterMeta.textContent = arducopter.detail || "Awaiting status.";
    }
    setCardState(cards.arducopter, serviceStateToCard(arducopter.status));

    if (mavproxyStatus) {
      mavproxyStatus.textContent = formatServiceStatus(mavproxy.status);
    }
    if (mavproxyMeta) {
      mavproxyMeta.textContent = mavproxy.detail || "Awaiting status.";
    }
    setCardState(cards.mavproxy, serviceStateToCard(mavproxy.status));

    if (uplinkStatus) {
      uplinkStatus.textContent = formatServiceStatus(uplink.status);
    }
    if (uplinkMeta) {
      uplinkMeta.textContent = uplink.detail || "Awaiting status.";
    }
    setCardState(cards.uplink, serviceStateToCard(uplink.status));
  }

  if (autopilot && autopilot.system) {
    const system = autopilot.system;
    setLteBanner(Boolean(system.wwan0_active));
    if (uplinkSignalValue) {
      if (Number.isFinite(system.lte_signal_percent)) {
        uplinkSignalValue.textContent = `${Math.round(system.lte_signal_percent)}%`;
      } else {
        uplinkSignalValue.textContent = "n/a";
      }
    }
    if (headerLoad) {
      headerLoad.textContent =
        system.load_1 !== null && system.load_5 !== null && system.load_15 !== null
          ? `${system.load_1.toFixed(2)} / ${system.load_5.toFixed(2)} / ${system.load_15.toFixed(2)}`
          : "n/a";
    }
    if (headerMemory) {
      headerMemory.textContent =
        system.mem_available_bytes !== null && system.mem_available_bytes !== undefined
          ? formatBytes(system.mem_available_bytes)
          : "n/a";
    }
    if (headerDisk) {
      headerDisk.textContent =
        system.disk_free_bytes !== null && system.disk_free_bytes !== undefined
          ? formatBytes(system.disk_free_bytes)
          : "n/a";
    }
    if (systemLoad) {
      if (system.load_1 !== null && system.load_5 !== null && system.load_15 !== null) {
        systemLoad.textContent = `${system.load_1.toFixed(2)} / ${system.load_5.toFixed(2)} / ${system.load_15.toFixed(
          2
        )}`;
      } else {
        systemLoad.textContent = "n/a";
      }
      systemLoadMeta.textContent =
        system.cpu_count !== null && system.cpu_count !== undefined
        ? `CPU cores: ${system.cpu_count}`
        : "CPU core count unavailable.";
    }
    if (system.load_1 !== null && system.load_1 !== undefined) {
      recordLoadPoint(system.load_1, system.cpu_count);
    }

    if (systemMemory) {
      if (system.mem_total_bytes !== null && system.mem_total_bytes !== undefined) {
        const used = system.mem_used_bytes ?? 0;
        systemMemory.textContent = `${formatBytes(used)} / ${formatBytes(system.mem_total_bytes)}`;
        systemMemoryMeta.textContent =
          system.mem_available_bytes !== null && system.mem_available_bytes !== undefined
          ? `Available ${formatBytes(system.mem_available_bytes)}`
          : "Memory availability unknown.";
      } else {
        systemMemory.textContent = "n/a";
        systemMemoryMeta.textContent = "Memory telemetry unavailable.";
      }
    }

    if (system.mem_used_bytes !== null && system.mem_used_bytes !== undefined) {
      recordMemoryPoint(system.mem_used_bytes, system.mem_total_bytes);
    }

    if (systemDisk) {
      if (system.disk_total_bytes !== null && system.disk_total_bytes !== undefined) {
        systemDisk.textContent = `${formatBytes(system.disk_used_bytes ?? 0)} / ${formatBytes(
          system.disk_total_bytes
        )}`;
        systemDiskMeta.textContent =
          system.disk_free_bytes !== null && system.disk_free_bytes !== undefined
          ? `Free ${formatBytes(system.disk_free_bytes)}`
          : "Disk availability unknown.";
      } else {
        systemDisk.textContent = "n/a";
        systemDiskMeta.textContent = "Disk telemetry unavailable.";
      }
    }

  }

  updateEngageToggle(onics);

  if (!autopilot || !autopilot.system) {
    setLteBanner(false);
  }

  const restartReady = health.tailscale_ok && health.dns_ok && health.tcp_ok && !rebootPending;
  serviceRestartButtons.forEach((button) => {
    button.disabled = !restartReady;
  });
  if (rebootBtn) {
    rebootBtn.disabled = !restartReady;
  }
}

function appendLog(line) {
  if (line.includes("blob data")) {
    return;
  }
  parseTelemetry(line);
  const renderedLine = formatLogTimestamp(line);
  const div = document.createElement("div");
  div.className = "log-line";
  div.textContent = renderedLine;
  logView.prepend(div);

  const maxLines = 400;
  while (logView.children.length > maxLines) {
    logView.removeChild(logView.lastChild);
  }

}

async function sendCommand(path) {
  const res = await fetch(path, { method: "POST" });
  if (!res.ok) {
    const data = await res.json().catch(() => ({}));
    throw new Error(data.msg || "Command failed");
  }
  return res.json();
}

async function fetchServiceLogs(service) {
  const res = await fetch(`/api/services/${service}/logs`);
  if (!res.ok) {
    const data = await res.json().catch(() => ({}));
    throw new Error(data.msg || "Log request failed");
  }
  return res.json();
}

engageToggleBtn?.addEventListener("click", async () => {
  if (!engageToggleAction) {
    return;
  }
  engageToggleBtn.disabled = true;
  try {
    const data = await sendCommand(engageToggleAction);
    updateSnapshot(data.snapshot);
  } catch (err) {
    appendLog(`COMMAND FAILED: ${err.message}`);
  }
});

clearBtn?.addEventListener("click", async () => {
  clearBtn.disabled = true;
  try {
    const data = await sendCommand("/api/clear");
    logView.innerHTML = "";
    resetTelemetryState();
    updateSnapshot(data.snapshot);
  } catch (err) {
    appendLog(`CLEAR FAILED: ${err.message}`);
  } finally {
    clearBtn.disabled = false;
  }
});

telemetryLogToggle?.addEventListener("click", () => {
  setTelemetryLogExpanded(!telemetryLogExpanded);
});

telemetryLogToggle?.addEventListener("keydown", (event) => {
  if (event.key === "Enter" || event.key === " ") {
    event.preventDefault();
    setTelemetryLogExpanded(!telemetryLogExpanded);
  }
});

serviceRestartButtons.forEach((button) => {
  button.addEventListener("click", async () => {
    const service = button.getAttribute("data-service-restart");
    if (!service) {
      return;
    }
    button.disabled = true;
    try {
      const data = await sendCommand(`/api/services/${service}/restart`);
      updateSnapshot(data.snapshot);
      appendLog(`RESTART REQUESTED: ${service}`);
    } catch (err) {
      appendLog(`RESTART FAILED (${service}): ${err.message}`);
    } finally {
      button.disabled = false;
    }
  });
});

serviceLogCards.forEach((card) => {
  const service = card.getAttribute("data-service-log");
  if (!service) {
    return;
  }
  const requestLogs = async () => {
    card.setAttribute("aria-busy", "true");
    try {
      const data = await fetchServiceLogs(service);
      const lines = Array.isArray(data.lines) ? data.lines : [];
      appendLog(`SERVICE LOG (${service}): ${lines.length} lines`);
      lines.forEach((line) => {
        appendLog(`[${service}] ${line}`);
      });
    } catch (err) {
      appendLog(`SERVICE LOG FAILED (${service}): ${err.message}`);
    } finally {
      card.removeAttribute("aria-busy");
    }
  };
  card.addEventListener("click", (event) => {
    if (event.target.closest(".btn--service")) {
      return;
    }
    requestLogs();
  });
  card.addEventListener("keydown", (event) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      requestLogs();
    }
  });
});

logoMenuButton?.addEventListener("click", (event) => {
  event.stopPropagation();
  const isOpen = logoMenu?.classList.contains("is-open");
  setLogoMenuOpen(!isOpen);
});

document.addEventListener("click", (event) => {
  if (!logoMenu || !logoMenuButton) {
    return;
  }
  if (logoMenu.contains(event.target) || logoMenuButton.contains(event.target)) {
    return;
  }
  setLogoMenuOpen(false);
});

document.addEventListener("keydown", (event) => {
  if (event.key === "Escape") {
    setLogoMenuOpen(false);
  }
});

window.addEventListener("resize", () => {
  updateTelemetryLogHeight();
});

rebootBtn?.addEventListener("click", async () => {
  const shouldRestart = window.confirm(
    "Restart the vehicle now?\n\nWarning: Do not initiate a restart if the vehicle is in flight."
  );
  if (!shouldRestart) {
    return;
  }
  rebootBtn.disabled = true;
  setLogoMenuOpen(false);
  try {
    const data = await sendCommand("/api/reboot");
    rebootPending = true;
    rebootAwaitingLoss = true;
    setRebootOverlay(true);
    updateSnapshot(data.snapshot);
    appendLog("VEHICLE RESTART REQUESTED: reboot now");
  } catch (err) {
    appendLog(`VEHICLE RESTART FAILED: ${err.message}`);
  } finally {
    if (!rebootPending) {
      rebootBtn.disabled = false;
    }
  }
});

loginDismissBtn?.addEventListener("click", () => {
  loginModal?.classList.remove("is-visible");
  loginModal?.setAttribute("aria-hidden", "true");
});

if (isDryRun && !isLoadOnly) {
  dismissLoadingScreen();
  if (connectionPill) {
    connectionPill.textContent = "LINK: DRY RUN";
    connectionStatus = "degraded";
    const pillStyles = getConnectionPillStyles(connectionStatus);
    connectionPill.style.borderColor = pillStyles.borderColor;
    connectionPill.style.color = pillStyles.color;
  }
}

if (isLoadOnly && connectionPill) {
  connectionPill.textContent = "LINK: LOAD ONLY";
  connectionStatus = "degraded";
  const pillStyles = getConnectionPillStyles(connectionStatus);
  connectionPill.style.borderColor = pillStyles.borderColor;
  connectionPill.style.color = pillStyles.color;
}

if (loadingScreen && !loadingScreen.classList.contains("hidden")) {
  setLoadingState(true);
}

if (monochromeToggle) {
  monochromeToggle.addEventListener("click", () => {
    setMonochromeMode(!isMonochromeMode);
    setLogoMenuOpen(false);
  });
}

try {
  const storedPreference = window.localStorage.getItem("monochrome-mode");
  if (storedPreference) {
    setMonochromeMode(storedPreference === "true");
  }
} catch (err) {
  console.warn("Unable to read monochrome preference", err);
}

if (!isLoadOnly) {
  const eventSource = new EventSource("/stream");

  eventSource.onopen = () => {
    setOfflineState(false);
  };

  eventSource.addEventListener("status", (event) => {
    updateSnapshot(JSON.parse(event.data), { fromStream: true });
  });

  eventSource.addEventListener("state", (event) => {
    updateSnapshot(JSON.parse(event.data), { fromStream: true });
  });

  eventSource.addEventListener("log", (event) => {
    const payload = JSON.parse(event.data);
    appendLog(payload.line);
  });

  eventSource.onerror = () => {
    if (!isDryRun) {
      setOfflineState(true);
      connectionPill.textContent = "LINK: OFFLINE";
      connectionStatus = "los";
      const pillStyles = getConnectionPillStyles(connectionStatus);
      connectionPill.style.borderColor = pillStyles.borderColor;
      connectionPill.style.color = pillStyles.color;
    }
    if (rebootPending) {
      rebootAwaitingLoss = false;
      setRebootOverlay(true);
    }
  };
}

setInterval(() => {
  if (!latestSnapshot) {
    return;
  }
  renderOnicsRuntime(latestSnapshot.onics);
}, 1000);

if (loadGraphs.length) {
  const resizeObserver = new ResizeObserver(() => {
    drawAllLoadGraphs();
    drawAllMemoryGraphs();
  });
  metricGraphs.forEach((graph) => {
    resizeObserver.observe(graph.canvas);
  });
  window.addEventListener("resize", () => {
    drawAllLoadGraphs();
    drawAllMemoryGraphs();
  });
  setInterval(() => {
    drawAllLoadGraphs();
    drawAllMemoryGraphs();
  }, 1000);
}

if (!isLoadOnly) {
  fetch("/api/snapshot")
    .then((res) => res.json())
    .then((snapshot) => {
      updateSnapshot(snapshot);
      (snapshot.logs || []).forEach((line) => {
        parseTelemetry(line);
      });
    })
    .catch(() => {
      appendLog("WARNING: Unable to fetch initial snapshot.");
    });
}
