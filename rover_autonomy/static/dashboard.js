"use strict";
const byId = id => document.getElementById(id);
let epoch = null, frozen = false, imageUrl = null, heartbeatBusy = false;
let heartbeatFailures = 0, lastHeartbeatOk = null, heartbeatConfigInitialized = false;
const verboseDisplayFields = new Set(["gap_ranges", "sampled_points", "obstacle_points"]);

async function json(url, options) {
  const response = await fetch(url, options);
  const value = await response.json();
  if (!response.ok) throw new Error(value.error || `HTTP ${response.status}`);
  return value;
}
function displayJson(value) {
  return JSON.stringify(value, (key, item) =>
    verboseDisplayFields.has(key) ? undefined : item, 2);
}
function post(url, value = {}) {
  return json(url, {method: "POST", headers: {"Content-Type": "application/json", "X-Rover-Autonomy": "1"},
    body: JSON.stringify(value)});
}
async function heartbeat() {
  if (heartbeatBusy) return;
  heartbeatBusy = true;
  const started = performance.now();
  const abort = new AbortController();
  const timeout = setTimeout(() => abort.abort(), 3000);
  try {
    await json("/api/heartbeat", {method: "POST", headers: {"Content-Type": "application/json", "X-Rover-Autonomy": "1"},
      body: JSON.stringify({}), signal: abort.signal});
    lastHeartbeatOk = {at: Date.now(), latencyMs: performance.now() - started};
    heartbeatFailures = 0;
  } catch (error) {
    heartbeatFailures++;
    byId("request-status").textContent =
      `Heartbeat unavailable (${heartbeatFailures} consecutive failure${heartbeatFailures === 1 ? "" : "s"}): ${error.message}`;
  } finally {
    clearTimeout(timeout);
    heartbeatBusy = false;
  }
}
setInterval(heartbeat, 500);
heartbeat();

async function status() {
  try {
    const value = await json("/api/status");
    epoch = value.epoch;
    byId("mode").textContent = value.motion_enabled ? "LIVE MOTOR OUTPUT ENABLED" : "MOCK MOTORS - no physical motion";
    const heartbeatAge = value.heartbeat_age_s;
    if (!heartbeatConfigInitialized) {
      byId("heartbeat-disabled").checked = !value.heartbeat_watchdog_enabled;
      byId("heartbeat-timeout").disabled = !value.heartbeat_watchdog_enabled;
      if (value.heartbeat_timeout_s !== null) byId("heartbeat-timeout").value = value.heartbeat_timeout_s;
      heartbeatConfigInitialized = true;
    }
    byId("heartbeat-warning").hidden = value.heartbeat_watchdog_enabled;
    const heartbeatText = lastHeartbeatOk
      ? `Browser heartbeat ${lastHeartbeatOk.latencyMs.toFixed(0)} ms; last success ${((Date.now() - lastHeartbeatOk.at) / 1000).toFixed(1)} s ago; failures ${heartbeatFailures}`
      : `Browser heartbeat has not succeeded; failures ${heartbeatFailures}`;
    const remaining = Number.isFinite(value.remaining_m) ? `; ${value.remaining_m.toFixed(2)} m remaining` : "";
    byId("controller-state").textContent =
      `${value.state}: ${value.reason}${remaining} | ${heartbeatText}; Pi last received ${heartbeatAge === null ? "never" : `${heartbeatAge.toFixed(1)} s ago`}`;
    byId("controller-state").className =
      ["FAULT", "BLOCKED"].includes(value.state) || heartbeatFailures > 0 ? "stale" : "";
    byId("status").textContent = displayJson(value);
  } catch (error) {
    epoch = null;
    byId("mode").textContent = `Controller unavailable: ${error.message}`;
  } finally { setTimeout(status, 500); }
}

byId("heartbeat-disabled").addEventListener("change", event => {
  byId("heartbeat-timeout").disabled = event.target.checked;
});
byId("heartbeat-config").addEventListener("submit", async event => {
  event.preventDefault();
  const disabled = byId("heartbeat-disabled").checked;
  if (disabled && !confirm("Disable the stop triggered by GCS/SSH heartbeat loss? The rover may keep moving after this dashboard disconnects.")) {
    return;
  }
  try {
    const value = await post("/api/heartbeat/configure", {
      enabled: !disabled,
      timeout_s: disabled ? null : Number(byId("heartbeat-timeout").value),
    });
    heartbeatConfigInitialized = false;
    const text = value.enabled ? `${value.timeout_s} s` : "DISABLED";
    byId("request-status").textContent = `Heartbeat-loss stop setting applied: ${text}`;
  } catch (error) {
    byId("request-status").textContent = `Heartbeat setting rejected: ${error.message}`;
  }
});

function drawMap(data) {
  const canvas = byId("map"), ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const project = point => [canvas.width / 2 - point[1] * 150, canvas.height - 30 - point[0] * 150];
  for (const point of data.obstacle_points || []) {
    const [x, y] = project(point);
    ctx.fillStyle = "#f47e75"; ctx.fillRect(x - 2, y - 2, 4, 4);
  }
  for (const candidate of data.candidates || []) {
    const selected = candidate.heading === data.selected_heading_rad;
    ctx.strokeStyle = selected ? "#67eaa6" : candidate.clear ? "#7e97c4" : "#d87069";
    ctx.lineWidth = selected ? 4 : 1;
    ctx.beginPath();
    for (const [i, point] of (candidate.path || []).entries()) {
      const [x, y] = project(point);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  ctx.fillStyle = "#e6eef8"; ctx.fillText("Rover: forward up, left to the left", 12, 18);
  ctx.fillRect(canvas.width / 2 - 8, canvas.height - 38, 16, 16);
}

async function matchedView() {
  for (let attempt = 0; attempt < 2; attempt++) {
    const data = await json("/api/view");
    const response = await fetch(`/api/frame.jpg?seq=${data.seq}`);
    if (response.status === 409 && attempt === 0) continue;
    if (!response.ok) throw new Error(`Matching image unavailable: HTTP ${response.status}`);
    if (response.headers.get("X-Rover-Sequence") !== String(data.seq)) {
      throw new Error("Image/decision sequence mismatch");
    }
    const url = URL.createObjectURL(await response.blob()), image = new Image();
    image.src = url;
    try { await image.decode(); }
    catch (error) { URL.revokeObjectURL(url); throw error; }
    return {data, url};
  }
  throw new Error("Matching image expired after one retry");
}

async function refresh() {
  try {
    if (frozen) return;
    const {data, url} = await matchedView();
    if (frozen) { URL.revokeObjectURL(url); return; }
    byId("camera").src = url;
    byId("camera").hidden = false;
    if (imageUrl) URL.revokeObjectURL(imageUrl);
    imageUrl = url;
    byId("decision").textContent = displayJson(data);
    drawMap(data);
    byId("image-status").textContent =
      `Matched frame ${data.seq} / OAK-D ${data.sensor_sequence ?? "synthetic"}; age at view response ${data.view_age_s.toFixed(2)} s`;
    byId("image-status").className = data.view_age_s > 1 ? "stale" : "";
  } catch (error) {
    byId("image-status").textContent = `STALE debug view retained: ${error.message}`;
    byId("image-status").className = "stale";
  } finally { setTimeout(refresh, 300); }
}

byId("goal").addEventListener("submit", async event => {
  event.preventDefault();
  if (epoch === null) { byId("request-status").textContent = "Refresh controller status before starting."; return; }
  try {
    const value = await post("/api/goal", {x: Number(byId("x").value), y: Number(byId("y").value),
      request_id: crypto.randomUUID(), epoch});
    epoch = value.epoch;
    byId("request-status").textContent = `Accepted goal ${value.goal_id}`;
  } catch (error) { byId("request-status").textContent = `Goal rejected: ${error.message}`; }
});
for (const action of ["stop", "cancel"]) {
  byId(action).addEventListener("click", async () => {
    try { await post(`/api/${action}`); epoch = null; byId("request-status").textContent = `${action} requested`; }
    catch (error) { byId("request-status").textContent = `STOP REQUEST FAILED: ${error.message}. Use physical stop.`; }
  });
}
byId("freeze").addEventListener("click", () => {
  frozen = !frozen;
  byId("freeze").textContent = frozen ? "Resume live debug view" : "Freeze debug view";
  if (frozen) {
    byId("image-status").textContent = "FROZEN debug view; controller and heartbeat remain live.";
    byId("image-status").className = "stale";
  }
});
status();
refresh();
