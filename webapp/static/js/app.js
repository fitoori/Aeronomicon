const engageToggleBtn = document.getElementById("engage-toggle-btn");
const clearBtn = document.getElementById("clear-btn");
const logView = document.getElementById("log-view");
const loadingScreen = document.getElementById("loading-screen");
const connectionPill = document.getElementById("connection-pill");
const onicsState = document.querySelector("#onics-state .status-strip__value");
const onicsRuntime = document.getElementById("onics-runtime");
const loginModal = document.getElementById("login-modal");
const loginMessage = document.getElementById("login-message");
const loginCommand = document.getElementById("login-command");
const loginSetupCommand = document.getElementById("login-setup-command");
const loginKeygenCommand = document.getElementById("login-keygen-command");
const loginOpenBtn = document.getElementById("login-open-btn");
const loginDismissBtn = document.getElementById("login-dismiss-btn");
const serviceRestartButtons = document.querySelectorAll("[data-service-restart]");

const tailscaleStatus = document.getElementById("tailscale-status");
const tailscaleMeta = document.getElementById("tailscale-meta");
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
const lteSignalValue = document.getElementById("lte-signal-value");
const lteSignalStatus = document.getElementById("lte-signal-status");
const lteSignalMeta = document.getElementById("lte-signal-meta");
const systemLoad = document.getElementById("system-load");
const systemLoadMeta = document.getElementById("system-load-meta");
const systemLoadGraph = document.getElementById("system-load-graph");
const systemMemory = document.getElementById("system-memory");
const systemMemoryMeta = document.getElementById("system-memory-meta");
const systemDisk = document.getElementById("system-disk");
const systemDiskMeta = document.getElementById("system-disk-meta");
const headerLoad = document.getElementById("header-load");
const headerLoadGraph = document.getElementById("header-load-graph");
const headerMemory = document.getElementById("header-memory");
const headerDisk = document.getElementById("header-disk");
const pinSectionSelect = document.getElementById("pin-section-select");
const pinHost = document.getElementById("pin-host");

let engageToggleAction = null;

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
  lteSignal: document.getElementById("lte-signal-card"),
  systemLoad: document.getElementById("system-load-card"),
  systemMemory: document.getElementById("system-memory-card"),
  systemDisk: document.getElementById("system-disk-card"),
};

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

function formatSeconds(value) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) {
    return "n/a";
  }
  const seconds = Math.max(0, Math.floor(Number(value)));
  return `${seconds} seconds`;
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

function resizeLoadGraph(graph) {
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
  resizeLoadGraph(graph);

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

  graph.context.strokeStyle = "rgba(74, 222, 128, 0.85)";
  graph.context.lineWidth = 2;
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

const pinSections = new Map();
let currentPinnedSection = null;

function setupPinSections() {
  if (!pinHost) {
    return;
  }
  document.querySelectorAll("[data-pin-section]").forEach((section) => {
    const key = section.getAttribute("data-pin-section");
    if (!key) {
      return;
    }
    const placeholder = document.createElement("span");
    placeholder.className = "pin-placeholder";
    placeholder.setAttribute("aria-hidden", "true");
    section.after(placeholder);
    pinSections.set(key, {
      element: section,
      placeholder,
    });
  });
}

function restoreSection(key) {
  const entry = pinSections.get(key);
  if (!entry) {
    return;
  }
  const { element, placeholder } = entry;
  if (placeholder.parentElement) {
    placeholder.parentElement.insertBefore(element, placeholder);
  }
}

function pinSection(key) {
  if (!pinHost || !pinSections.has(key)) {
    return;
  }
  if (currentPinnedSection && currentPinnedSection !== key) {
    restoreSection(currentPinnedSection);
  }
  const entry = pinSections.get(key);
  pinHost.appendChild(entry.element);
  currentPinnedSection = key;
}

function applyPinnedSection(value) {
  const key = value || "tailscale";
  if (!pinSections.has(key)) {
    return;
  }
  pinSection(key);
  if (pinSectionSelect) {
    pinSectionSelect.value = key;
  }
  try {
    localStorage.setItem("pinnedSection", key);
  } catch (err) {
    console.warn("Unable to persist pinned section", err);
  }
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
};

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
    startupStage.textContent = telemetryState.startupStage || "Awaiting telemetry";
    if (telemetryState.startupTimestamp) {
      const startupAge = formatTimestamp(telemetryState.startupTimestamp);
      startupMeta.textContent = startupAge.text;
      setAgeSeverity(startupMeta, startupAge.ageSeconds);
    } else {
      startupMeta.textContent = "No startup milestones yet.";
      setAgeSeverity(startupMeta, null);
    }
    setCardState(cards.startup, telemetryState.startupStage ? "ok" : null);
  }

  if (autopilotAlert && autopilotMeta) {
    autopilotAlert.textContent =
      telemetryState.autopilotMessage || "Awaiting telemetry";
    if (telemetryState.autopilotTimestamp) {
      const autopilotAge = formatTimestamp(telemetryState.autopilotTimestamp);
      autopilotMeta.textContent = autopilotAge.text;
      setAgeSeverity(autopilotMeta, autopilotAge.ageSeconds);
    } else {
      autopilotMeta.textContent = "No autopilot alerts yet.";
      setAgeSeverity(autopilotMeta, null);
    }
    if (telemetryState.autopilotLevel === "CRITICAL") {
      setCardState(cards.autopilot, "danger");
    } else if (telemetryState.autopilotLevel === "WARNING") {
      setCardState(cards.autopilot, "warn");
    } else if (telemetryState.autopilotMessage) {
      setCardState(cards.autopilot, null);
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

function setLoginPrompt(meta, onics) {
  if (
    !loginModal ||
    !loginMessage ||
    !loginCommand ||
    !loginSetupCommand ||
    !loginKeygenCommand ||
    !loginOpenBtn
  ) {
    return;
  }
  if (onics?.login_required) {
    const sshUser = meta.ssh_user || "pi";
    const sshCommand = `ssh -p ${meta.ssh_port} ${sshUser}@${meta.hostname}`;
    const sshUri = `ssh://${sshUser}@${meta.hostname}:${meta.ssh_port}`;
    const sshSetupCommand = `ssh-copy-id -p ${meta.ssh_port} ${sshUser}@${meta.hostname}`;
    const sshKeygenCommand = 'ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N ""';
    loginMessage.textContent =
      onics.login_message ||
      "SSH authentication is missing. Complete an interactive login to continue.";
    loginCommand.textContent = sshCommand;
    loginSetupCommand.textContent = sshSetupCommand;
    loginKeygenCommand.textContent = sshKeygenCommand;
    loginOpenBtn.onclick = () => {
      window.location.href = sshUri;
    };
    loginModal.classList.add("is-visible");
    loginModal.setAttribute("aria-hidden", "false");
  } else {
    loginModal.classList.remove("is-visible");
    loginModal.setAttribute("aria-hidden", "true");
  }
}

function updateSnapshot(snapshot) {
  if (!snapshot) {
    return;
  }

  if (loadingScreen && !loadingScreen.classList.contains("hidden")) {
    loadingScreen.classList.add("hidden");
    setTimeout(() => loadingScreen.remove(), 600);
  }

  const { meta, health, onics, autopilot } = snapshot;
  setLoginPrompt(meta, onics);

  const linkDegraded =
    health?.los ||
    health?.stale ||
    !health?.tailscale_ok ||
    !health?.dns_ok ||
    !health?.tcp_ok;
  connectionPill.textContent = health.los
    ? "LINK: LOS"
    : linkDegraded
      ? "LINK: DEGRADED"
      : "LINK: OK";
  connectionPill.style.borderColor = health.los
    ? "rgba(249,115,22,0.6)"
    : linkDegraded
      ? "rgba(245,158,11,0.6)"
      : "rgba(74,222,128,0.5)";
  connectionPill.style.color = health.los
    ? "#f97316"
    : linkDegraded
      ? "#f59e0b"
      : "#4ade80";

  if (onicsState) {
    onicsState.textContent = onics.state;
  }
  if (onicsRuntime) {
    onicsRuntime.textContent = `SSH ${onics.ssh_connected ? "connected" : "offline"} · last output ${formatAge(
      onics.last_output_age_s
    )}`;
    setAgeSeverity(onicsRuntime, onics.last_output_age_s);
  }
  if (startupFails) {
    const restartFails = Number.isFinite(onics.restart_failures)
      ? onics.restart_failures
      : 0;
    startupFails.textContent = `${restartFails}`;
  }

  if (tailscaleStatus) {
    tailscaleStatus.textContent = health.tailscale_ok ? "RUNNING" : "OFFLINE";
  }
  if (tailscaleMeta) {
    tailscaleMeta.textContent = health.tailscale_error || health.tailscale_backend_state;
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
    if (lteSignalValue) {
      if (Number.isFinite(system.lte_signal_percent)) {
        lteSignalValue.textContent = `${Math.round(system.lte_signal_percent)}%`;
      } else {
        lteSignalValue.textContent = "n/a";
      }
    }
    if (lteSignalStatus) {
      lteSignalStatus.textContent = Number.isFinite(system.lte_signal_percent)
        ? "Signal strength"
        : "Unavailable";
    }
    if (lteSignalMeta) {
      lteSignalMeta.textContent = Number.isFinite(system.lte_signal_percent)
        ? Number.isFinite(system.lte_rssi_dbm)
          ? `RSSI ${system.lte_rssi_dbm} dBm`
          : "RSSI unavailable."
        : system.lte_signal_error || "LTE signal unavailable.";
    }
    if (cards.lteSignal) {
      if (Number.isFinite(system.lte_signal_percent)) {
        const percent = Number(system.lte_signal_percent);
        const state = percent >= 70 ? "ok" : percent >= 40 ? "warn" : "danger";
        setCardState(cards.lteSignal, state);
      } else {
        setCardState(cards.lteSignal, "warn");
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

  const restartReady = health.tailscale_ok && health.dns_ok && health.tcp_ok;
  serviceRestartButtons.forEach((button) => {
    button.disabled = !restartReady;
  });
}

function appendLog(line) {
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
    telemetryState.trackingConfidence = null;
    telemetryState.trackingTimestamp = null;
    telemetryState.startupStage = null;
    telemetryState.startupTimestamp = null;
    telemetryState.autopilotLevel = null;
    telemetryState.autopilotMessage = null;
    telemetryState.autopilotTimestamp = null;
    updateTelemetryCards();
    updateSnapshot(data.snapshot);
  } catch (err) {
    appendLog(`CLEAR FAILED: ${err.message}`);
  } finally {
    clearBtn.disabled = false;
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

loginDismissBtn?.addEventListener("click", () => {
  loginModal?.classList.remove("is-visible");
  loginModal?.setAttribute("aria-hidden", "true");
});

const eventSource = new EventSource("/stream");

eventSource.addEventListener("status", (event) => {
  updateSnapshot(JSON.parse(event.data));
});

eventSource.addEventListener("state", (event) => {
  updateSnapshot(JSON.parse(event.data));
});

eventSource.addEventListener("log", (event) => {
  const payload = JSON.parse(event.data);
  appendLog(payload.line);
});

eventSource.onerror = () => {
  connectionPill.textContent = "LINK: OFFLINE";
  connectionPill.style.borderColor = "rgba(249,115,22,0.6)";
  connectionPill.style.color = "#f97316";
};

setupPinSections();

if (pinSectionSelect) {
  const saved = (() => {
    try {
      return localStorage.getItem("pinnedSection");
    } catch (err) {
      return null;
    }
  })();
  const initial = saved || pinSectionSelect.value;
  applyPinnedSection(initial);
  pinSectionSelect.addEventListener("change", (event) => {
    applyPinnedSection(event.target.value);
  });
} else {
  applyPinnedSection("tailscale");
}

if (loadGraphs.length) {
  const resizeObserver = new ResizeObserver(() => {
    drawAllLoadGraphs();
  });
  loadGraphs.forEach((graph) => {
    resizeObserver.observe(graph.canvas);
  });
  window.addEventListener("resize", drawAllLoadGraphs);
  setInterval(drawAllLoadGraphs, 1000);
}

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
