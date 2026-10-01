"use strict";

const byId = id => document.getElementById(id);
const number = (value, digits = 2) => typeof value === "number" ? value.toFixed(digits) : "-";
let frozen = false;
let displayed = null;
let imageUrl = null;
let imageSequence = null;
let mapProjection = null;
let pendingRevision = null;

async function getJson(url, options) {
  const response = await fetch(url, options);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

async function writeGoal(path, body) {
  return getJson(path, {
    method: "POST",
    headers: { "Content-Type": "application/json", "X-Rover-Preview": "1" },
    body: JSON.stringify(body)
  });
}

byId("goal-form").addEventListener("submit", async event => {
  event.preventDefault();
  try {
    const result = await writeGoal("/api/goal", {
      x: Number(byId("goal-x").value), y: Number(byId("goal-y").value)
    });
    pendingRevision = result.goal_revision;
    frozen = false;
    byId("goal-result").textContent = `Goal accepted for preview (${result.goal.join(", ")} m). Waiting for a new camera decision.`;
  } catch (error) {
    byId("goal-result").textContent = `Goal rejected: ${error.message}`;
  }
});

byId("cancel").addEventListener("click", async () => {
  try {
    const result = await writeGoal("/api/goal/cancel", {});
    pendingRevision = result.goal_revision;
    frozen = false;
    byId("goal-result").textContent = "Goal cleared. Waiting for the next preview frame.";
  } catch (error) {
    byId("goal-result").textContent = `Clear failed: ${error.message}`;
  }
});
byId("freeze").addEventListener("click", () => { frozen = true; });
byId("live").addEventListener("click", () => { frozen = false; });

function drawMap(data) {
  const canvas = byId("map"), ctx = canvas.getContext("2d");
  const extent = Math.max(2.5, data.goal ? data.goal[0] * 1.2 : 0,
    data.goal ? Math.abs(data.goal[1]) * 1.8 : 0);
  const scale = canvas.height * .82 / extent;
  const project = (x, y) => [canvas.width / 2 - y * scale, canvas.height * .9 - x * scale];
  mapProjection = project;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.font = "14px system-ui";
  ctx.lineWidth = 1;
  for (let x = 0; x <= extent; x += .5) {
    const start = project(x, -canvas.width / scale / 2), end = project(x, canvas.width / scale / 2);
    ctx.strokeStyle = "#263746";
    ctx.beginPath(); ctx.moveTo(...start); ctx.lineTo(...end); ctx.stroke();
    ctx.fillStyle = "#aebdcc"; ctx.fillText(`${x.toFixed(1)} m`, 6, start[1] - 4);
  }
  const point = (position, color, radius) => {
    ctx.fillStyle = color; ctx.beginPath(); ctx.arc(...project(position[0], position[1]), radius, 0, 2 * Math.PI); ctx.fill();
  };
  for (const p of data.sampled_points || []) point(p, "#53616d", 1.4);
  for (const p of data.obstacle_points || []) point(p, "#ff7474", 2.5);
  const path = (samples, color, width) => {
    if (!samples || !samples.length) return;
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
    ctx.moveTo(...project(...samples[0]));
    for (const p of samples.slice(1)) ctx.lineTo(...project(...p));
    ctx.stroke();
  };
  for (const candidate of data.candidates || []) {
    path(candidate.path, candidate.clear ? "#64ce8e" : "#a94250", 2);
    if (candidate.blocked_point) point(candidate.blocked_point, "#ffb3a7", 5);
  }
  const chosen = (data.candidates || []).find(c => c.clear && c.heading === data.selected_heading_rad);
  if (chosen) path(chosen.path, "#50d9ef", 5);
  if (data.search_step) {
    path(data.search_step.path, "#ffd67c", 5);
    point(data.search_step.point, "#ffd67c", 7);
  }
  const origin = project(0, 0);
  ctx.strokeStyle = "#aebdcc"; ctx.lineWidth = 2; ctx.setLineDash([6, 5]);
  ctx.beginPath(); ctx.arc(...origin, (data.collision_radius_m || .205) * scale, 0, Math.PI * 2); ctx.stroke();
  ctx.setLineDash([]);
  point([0, 0], "#dce6ef", 5);
  ctx.fillStyle = "#dce6ef"; ctx.fillText("ROVER  +X forward / +Y left", origin[0] - 100, origin[1] + 35);
  if (data.goal) {
    const target = project(...data.goal);
    ctx.strokeStyle = "#fff"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(target[0] - 8, target[1]); ctx.lineTo(target[0] + 8, target[1]);
    ctx.moveTo(target[0], target[1] - 8); ctx.lineTo(target[0], target[1] + 8); ctx.stroke();
    ctx.fillStyle = "#fff"; ctx.fillText("GOAL", target[0] + 12, target[1]);
  }
}

byId("map").addEventListener("mousemove", event => {
  if (!displayed || !mapProjection) return;
  const rect = event.currentTarget.getBoundingClientRect();
  const u = (event.clientX - rect.left) * event.currentTarget.width / rect.width;
  const v = (event.clientY - rect.top) * event.currentTarget.height / rect.height;
  let nearest = null, minimum = 15;
  for (const p of displayed.obstacle_points || []) {
    const pixel = mapProjection(p[0], p[1]), distance = Math.hypot(u - pixel[0], v - pixel[1]);
    if (distance < minimum) { nearest = p; minimum = distance; }
  }
  byId("point-info").textContent = nearest
    ? `Measured point: X ${number(nearest[0])}, Y ${number(nearest[1])}, Z ${number(nearest[2])} m. Horizontal range from base origin: ${number(Math.hypot(nearest[0], nearest[1]))} m.`
    : "Hover a red point to inspect its range and rover-relative position.";
});

function showDecision(data) {
  displayed = data;
  byId("state").textContent = data.state;
  byId("state").className = data.state === "FAULT" || data.state === "PREVIEW_BLOCKED" ? "blocked" : "clear";
  byId("mode").textContent = frozen ? "Frozen failure" : "Latest preview";
  byId("reason").textContent = data.reason;
  byId("source").textContent = data.source;
  byId("frame").textContent = `${data.seq} / goal revision ${data.goal_revision}`;
  byId("timestamp").textContent = data.captured_at ? new Date(data.captured_at * 1000).toLocaleString() : "-";
  byId("valid").textContent = typeof data.valid_fraction === "number" ? `${number(data.valid_fraction * 100, 1)}%` : "-";
  byId("unknown").textContent = typeof data.unmeasured_fraction === "number" ? `${number(data.unmeasured_fraction * 100, 1)}%` : "-";
  byId("counts").textContent = `${data.sampled_depth_points ?? "-"} / ${data.considered_obstacle_points ?? "-"}`;
  byId("range").textContent = data.nearest_obstacle_range_m == null ? "No considered return" : `${number(data.nearest_obstacle_range_m)} m from base origin`;
  byId("processing").textContent = `${number(data.processing_ms, 1)} ms`;
  byId("mount-warning").hidden = data.camera_mount_verified === true;
  byId("download").hidden = !frozen || !data.has_camera_frame;
  byId("download").href = `/api/frozen/download?seq=${data.seq}`;
  byId("raw").textContent = JSON.stringify(data, null, 2);
  const rows = (data.candidates || []).map(candidate => {
    const row = document.createElement("tr");
    const selected = candidate.clear && candidate.heading === data.selected_heading_rad;
    row.className = selected ? "chosen" : candidate.clear ? "clear" : "blocked";
    const values = [
      `${number(candidate.heading * 180 / Math.PI, 1)} deg`,
      `${number(candidate.linear)} m/s / ${number(candidate.angular)} rad/s`,
      selected ? "SELECTED" : candidate.clear ? "CLEAR" : "BLOCKED",
      candidate.blocked_point ? candidate.blocked_point.map(v => number(v, 3)).join(", ") : "-"
    ];
    for (const value of values) {
      const cell = document.createElement("td"); cell.textContent = value; row.appendChild(cell);
    }
    return row;
  });
  byId("candidates").replaceChildren(...rows);
  byId("search").textContent = data.search_step
    ? `Hypothetical search prefix: ${number(data.search_step.distance)} m to (${data.search_step.point.map(v => number(v)).join(", ")}). This is not a driving command.`
    : "No search prefix";
  byId("depth-policy").textContent = data.missing_depth_blocks_paths
    ? "Conservative mode: unmeasured rays can block paths. Missing depth is not proof of free space."
    : "Measured-return mode: missing depth does not veto paths, but is never proof of free space.";
  if (pendingRevision !== null && data.goal_revision >= pendingRevision) {
    byId("goal-result").textContent = data.goal
      ? `Displaying the actual preview decision for goal (${data.goal.join(", ")} m).`
      : "Goal cleared; displaying obstacle inputs only.";
    pendingRevision = null;
  }
  drawMap(data);
}

function showHealth(health) {
  byId("fps").textContent = `${number(health.input_fps, 1)} frames/s`;
  byId("age").textContent = `${number(health.input_dequeue_age_s, 2)} s`;
  byId("cpu").textContent = `${number(health.process_cpu_percent, 1)}%`;
  byId("memory").textContent = `${number(health.rss_mb, 1)} MB`;
  byId("temperature").textContent = health.cpu_temperature_c == null ? "Unavailable" : `${number(health.cpu_temperature_c, 1)} C`;
  byId("workers").textContent = `${health.camera_thread_alive ? "running" : "STOPPED"} / ${health.display_thread_alive ? "running" : "STOPPED"}`;
  byId("health-error").textContent = health.visualization_error
    ? `Visualization failed: ${health.visualization_error}`
    : !health.camera_thread_alive ? "Camera worker stopped. Inspect the service log and fault reason." : "";
}

async function refresh() {
  const requestedFrozen = frozen;
  try {
    const [data, health] = await Promise.all([
      getJson(requestedFrozen ? "/api/frozen" : "/api/view"), getJson("/api/health")
    ]);
    if (requestedFrozen !== frozen) return;
    showHealth(health);
    if (!displayed || data.seq !== displayed.seq || frozen !== (byId("mode").textContent === "Frozen failure")
        || (data.image_available && imageSequence !== data.seq)) {
      if (data.image_available) {
        const response = await fetch(`${requestedFrozen ? "/api/frozen/frame.jpg" : "/api/frame.jpg"}?seq=${data.seq}`);
        if (response.status === 409) return;
        if (!response.ok) throw new Error(`Image request failed: HTTP ${response.status}`);
        const blob = await response.blob(), url = URL.createObjectURL(blob);
        const image = new Image();
        image.src = url;
        try { await image.decode(); }
        catch (error) { URL.revokeObjectURL(url); throw error; }
        if (requestedFrozen !== frozen) { URL.revokeObjectURL(url); return; }
        byId("camera").src = url;
        byId("camera").hidden = false;
        if (imageUrl) URL.revokeObjectURL(imageUrl);
        imageUrl = url;
        imageSequence = data.seq;
        byId("image-status").textContent = `Image and decision share frame ${data.seq}.`;
        showDecision(data);
      } else {
        byId("camera").hidden = true;
        byId("image-status").textContent = "No image available for this decision.";
        showDecision(data);
      }
    }
    byId("connection").textContent = frozen ? "Connected - inspecting a frozen failure" : "Connected - no-motion preview";
    byId("connection").className = "";
  } catch (error) {
    byId("connection").textContent = `Preview unavailable: ${error.message}. Last displayed decision is retained.`;
    byId("connection").className = "disconnected";
    if (frozen && error.message === "no frozen failure available") frozen = false;
  } finally {
    setTimeout(refresh, 250);
  }
}

getJson("/api/config").then(config => {
  const {recorded_decision, ...settings} = config;
  byId("config").textContent = JSON.stringify(settings, null, 2);
  byId("recorded").hidden = recorded_decision == null;
  byId("recorded-decision").textContent = JSON.stringify(recorded_decision, null, 2);
}).catch(error => { byId("config").textContent = `Settings unavailable: ${error.message}`; });
refresh();
