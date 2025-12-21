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
const loginOpenBtn = document.getElementById("login-open-btn");
const loginDismissBtn = document.getElementById("login-dismiss-btn");

const tailscaleStatus = document.getElementById("tailscale-status");
const tailscaleMeta = document.getElementById("tailscale-meta");
const dnsStatus = document.getElementById("dns-status");
const dnsMeta = document.getElementById("dns-meta");
const tcpStatus = document.getElementById("tcp-status");
const tcpMeta = document.getElementById("tcp-meta");
const staleStatus = document.getElementById("stale-status");
const staleMeta = document.getElementById("stale-meta");

const cards = {
  tailscale: document.getElementById("tailscale-card"),
  dns: document.getElementById("dns-card"),
  tcp: document.getElementById("tcp-card"),
  stale: document.getElementById("stale-card"),
};

function setCardState(card, state) {
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

function setLoginPrompt(meta, onics) {
  if (!loginModal || !loginMessage || !loginCommand || !loginSetupCommand || !loginOpenBtn) {
    return;
  }
  if (onics?.login_required) {
    const sshUser = "pi";
    const sshCommand = `ssh -p ${meta.ssh_port} ${sshUser}@${meta.hostname}`;
    const sshUri = `ssh://${sshUser}@${meta.hostname}:${meta.ssh_port}`;
    const sshSetupCommand = `ssh-copy-id -p ${meta.ssh_port} ${sshUser}@${meta.hostname}`;
    loginMessage.textContent =
      onics.login_message ||
      "SSH authentication is missing. Complete an interactive login to continue.";
    loginCommand.textContent = sshCommand;
    loginSetupCommand.textContent = sshSetupCommand;
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

  connectionPill.textContent = health.los
    ? "LINK: LOS"
    : health.stale
      ? "LINK: DEGRADED"
      : "LINK: OK";
  connectionPill.style.borderColor = health.los
    ? "rgba(249,115,22,0.6)"
    : health.stale
      ? "rgba(245,158,11,0.6)"
      : "rgba(74,222,128,0.5)";
  connectionPill.style.color = health.los
    ? "#f97316"
    : health.stale
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

  if (health.los) {
    staleStatus.textContent = "LOS";
    staleMeta.textContent = "No ok heartbeat received.";
    setCardState(cards.stale, "danger");
  } else if (health.stale) {
    staleStatus.textContent = "STALE";
    staleMeta.textContent = `Last ok ${formatAge(health.last_ok_age_s)} ago.`;
    setCardState(cards.stale, "warn");
  } else {
    staleStatus.textContent = "OK";
    staleMeta.textContent = `Last ok ${formatAge(health.last_ok_age_s)} ago.`;
    setCardState(cards.stale, "ok");
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
  .then(updateSnapshot)
  .catch(() => {
    appendLog("WARNING: Unable to fetch initial snapshot.");
  });
