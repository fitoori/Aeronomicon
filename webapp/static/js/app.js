const engageBtn = document.getElementById("engage-btn");
const disengageBtn = document.getElementById("disengage-btn");
const clearBtn = document.getElementById("clear-btn");
const logView = document.getElementById("log-view");
const loadingScreen = document.getElementById("loading-screen");
const connectionPill = document.getElementById("connection-pill");
const appMeta = document.getElementById("app-meta");
const onicsState = document.querySelector("#onics-state .status-strip__value");
const onicsRuntime = document.getElementById("onics-runtime");
const loginModal = document.getElementById("login-modal");
const loginMessage = document.getElementById("login-message");
const loginCommand = document.getElementById("login-command");
const loginSetupCommand = document.getElementById("login-setup-command");
const loginKeygenCommand = document.getElementById("login-keygen-command");
const loginOpenBtn = document.getElementById("login-open-btn");
const loginDismissBtn = document.getElementById("login-dismiss-btn");

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
const autopilotAlert = document.getElementById("autopilot-alert");
const autopilotMeta = document.getElementById("autopilot-meta");

const arducopterStatus = document.getElementById("arducopter-status");
const arducopterMeta = document.getElementById("arducopter-meta");
const mavproxyStatus = document.getElementById("mavproxy-status");
const mavproxyMeta = document.getElementById("mavproxy-meta");
const uplinkStatus = document.getElementById("uplink-status");
const uplinkMeta = document.getElementById("uplink-meta");

const cards = {
  tailscale: document.getElementById("tailscale-card"),
  dns: document.getElementById("dns-card"),
  tcp: document.getElementById("tcp-card"),
  tracking: document.getElementById("tracking-card"),
  startup: document.getElementById("startup-card"),
  autopilot: document.getElementById("autopilot-card"),
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

function formatAge(value) {
  if (value === null || value === undefined) {
    return "n/a";
  }
  return `${value.toFixed(1)}s`;
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
  if (!ts) {
    return "Awaiting telemetry.";
  }
  return `Last update ${ts}`;
}

function updateTelemetryCards() {
  if (trackingConfidence && trackingMeta) {
    trackingConfidence.textContent =
      telemetryState.trackingConfidence || "Awaiting telemetry";
    trackingMeta.textContent = telemetryState.trackingTimestamp
      ? formatTimestamp(telemetryState.trackingTimestamp)
      : "No tracking updates yet.";
    setCardState(cards.tracking, telemetryState.trackingConfidence ? "ok" : null);
  }

  if (startupStage && startupMeta) {
    startupStage.textContent = telemetryState.startupStage || "Awaiting telemetry";
    startupMeta.textContent = telemetryState.startupTimestamp
      ? formatTimestamp(telemetryState.startupTimestamp)
      : "No startup milestones yet.";
    setCardState(cards.startup, telemetryState.startupStage ? "ok" : null);
  }

  if (autopilotAlert && autopilotMeta) {
    autopilotAlert.textContent =
      telemetryState.autopilotMessage || "Awaiting telemetry";
    autopilotMeta.textContent = telemetryState.autopilotTimestamp
      ? formatTimestamp(telemetryState.autopilotTimestamp)
      : "No autopilot alerts yet.";
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

  const trackingMatch = cleaned.match(/Tracking confidence:\s*(.+)$/i);
  if (trackingMatch) {
    telemetryState.trackingConfidence = trackingMatch[1].trim();
    telemetryState.trackingTimestamp = timestamp;
  }

  for (const stage of startupStages) {
    if (stage.regex.test(cleaned)) {
      telemetryState.startupStage = stage.label;
      telemetryState.startupTimestamp = timestamp;
      break;
    }
  }

  const autopilotMatch = cleaned.match(/^([A-Z]+):autopilot:(.+)$/);
  if (autopilotMatch) {
    telemetryState.autopilotLevel = autopilotMatch[1].trim();
    telemetryState.autopilotMessage = autopilotMatch[2].trim();
    telemetryState.autopilotTimestamp = timestamp;
  }

  updateTelemetryCards();
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

  const { meta, health, onics } = snapshot;
  const metaLine = `${meta.hostname} · ${meta.ssh_user}@${meta.hostname}:${meta.ssh_port}`;
  appMeta.textContent = metaLine;
  setLoginPrompt(meta, onics);

  const linkDegraded =
    health.los ||
    health.stale ||
    !health.tailscale_ok ||
    !health.dns_ok ||
    !health.tcp_ok;
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

  onicsState.textContent = onics.state;
  onicsRuntime.textContent = `SSH ${onics.ssh_connected ? "connected" : "offline"} · last output ${formatAge(
    onics.last_output_age_s
  )}`;

  tailscaleStatus.textContent = health.tailscale_ok ? "RUNNING" : "OFFLINE";
  tailscaleMeta.textContent = health.tailscale_error || health.tailscale_backend_state;
  setCardState(cards.tailscale, health.tailscale_ok ? "ok" : "danger");

  dnsStatus.textContent = health.dns_ok ? "RESOLVED" : "UNRESOLVED";
  dnsMeta.textContent = health.dns_ok
    ? `IPs: ${health.dns_ips.join(", ")}`
    : health.dns_error || "Awaiting DNS";
  setCardState(cards.dns, health.dns_ok ? "ok" : "warn");

  tcpStatus.textContent = health.tcp_ok ? "CONNECTED" : "BLOCKED";
  tcpMeta.textContent = health.tcp_ok
    ? `Probe ${health.tcp_ip}:${meta.ssh_port} ok`
    : health.tcp_error || "Awaiting TCP";
  setCardState(cards.tcp, health.tcp_ok ? "ok" : "danger");

  if (snapshot.autopilot && snapshot.autopilot.services) {
    const services = snapshot.autopilot.services;
    const arducopter = services.arducopter || {};
    const mavproxy = services.mavproxy || {};
    const uplink = services.uplink || {};

    arducopterStatus.textContent = formatServiceStatus(arducopter.status);
    arducopterMeta.textContent = arducopter.detail || "Awaiting status.";
    setCardState(cards.arducopter, serviceStateToCard(arducopter.status));

    mavproxyStatus.textContent = formatServiceStatus(mavproxy.status);
    mavproxyMeta.textContent = mavproxy.detail || "Awaiting status.";
    setCardState(cards.mavproxy, serviceStateToCard(mavproxy.status));

    uplinkStatus.textContent = formatServiceStatus(uplink.status);
    uplinkMeta.textContent = uplink.detail || "Awaiting status.";
    setCardState(cards.uplink, serviceStateToCard(uplink.status));
  }

  const canEngage = onics.state === "IDLE" || onics.state === "ERROR" || onics.state === "LOS";
  const canDisengage = onics.state === "RUNNING" || onics.state === "STARTING";
  engageBtn.disabled = !canEngage;
  disengageBtn.disabled = !canDisengage;
}

function appendLog(line) {
  const div = document.createElement("div");
  div.className = "log-line";
  div.textContent = line;
  logView.prepend(div);

  const maxLines = 400;
  while (logView.children.length > maxLines) {
    logView.removeChild(logView.lastChild);
  }

  parseTelemetry(line);
}

async function sendCommand(path) {
  const res = await fetch(path, { method: "POST" });
  if (!res.ok) {
    const data = await res.json().catch(() => ({}));
    throw new Error(data.msg || "Command failed");
  }
  return res.json();
}

engageBtn?.addEventListener("click", async () => {
  engageBtn.disabled = true;
  try {
    const data = await sendCommand("/api/engage");
    updateSnapshot(data.snapshot);
  } catch (err) {
    appendLog(`ENGAGE FAILED: ${err.message}`);
  }
});

disengageBtn?.addEventListener("click", async () => {
  disengageBtn.disabled = true;
  try {
    const data = await sendCommand("/api/disengage");
    updateSnapshot(data.snapshot);
  } catch (err) {
    appendLog(`DISENGAGE FAILED: ${err.message}`);
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
