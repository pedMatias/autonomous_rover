import json
from pathlib import Path

import pytest
import quickjs


SCRIPT = (Path(__file__).resolve().parents[1] / "rover_autonomy/static/dashboard.js").read_text()


@pytest.mark.parametrize("failure_count,expected_requests,stale", [(0, 1, False), (1, 2, False), (2, 2, True)])
def test_dashboard_retries_once_and_preserves_sequence_matching(failure_count, expected_requests, stale):
    context = quickjs.Context()
    context.eval("""
      const elements = {};
      let drawCalls = 0;
      const timers = [];
      let frameRequests = 0, viewRequests = 0;
      const document = {getElementById(id) {
        return elements[id] ||= {textContent: "", className: "", hidden: true, width: 600, height: 450,
          addEventListener() {}, getContext() { return {clearRect() {}, fillText() {},
            beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}, fillRect() {drawCalls++;}}; }};
      }};
      const URL = {createObjectURL: blob => "blob:" + blob.seq, revokeObjectURL() {}};
      class Image { decode() { return Promise.resolve(); } }
      function setInterval(fn, delay) { timers.push([fn, delay]); }
      function setTimeout(fn, delay) { if (delay === 1000) fn(); }
      function clearTimeout() {}
      const performance = {now: () => 0};
      class AbortController {
        constructor() { this.signal = {aborted: false}; }
        abort() { this.signal.aborted = true; }
      }
      const crypto = {randomUUID: () => "request"};
    """)
    context.eval(f"const failures = {failure_count};")
    context.eval("""
      async function fetch(url, options) {
        if (url === "/api/view") {
          const seq = ++viewRequests;
          return {ok: true, json: async () => ({seq, view_age_s: .1, candidates: [],
            sampled_points: [[1, 2, 3]], obstacle_points: [[1, 2, 3]]})};
        }
        if (url.startsWith("/api/frame.jpg")) {
          frameRequests++;
          const seq = Number(url.split("=")[1]);
          const expired = frameRequests <= failures;
          return {ok: !expired, status: expired ? 409 : 200,
            headers: {get: () => String(seq)}, blob: async () => ({seq})};
        }
        return {ok: true, json: async () => ({epoch: 0, motion_enabled: false,
          state: "BLOCKED", reason: "all local candidate paths blocked", remaining_m: 18.4,
          heartbeat_age_s: 2.1, heartbeat_watchdog_enabled: true, heartbeat_timeout_s: 2,
          logging: {gap_ranges: [{kind: "frame", first: 1, last: 2}]}})};
      }
    """)
    context.eval(SCRIPT)
    for _ in range(100):
        if not context.execute_pending_job():
            break
    else:
        pytest.fail("dashboard did not settle")
    assert context.eval("frameRequests") == expected_requests
    assert context.eval('elements["image-status"].className === "stale"') == stale
    if not stale:
        displayed = json.loads(context.eval('elements["decision"].textContent'))
        assert context.eval('elements["camera"].src') == f"blob:{displayed['seq']}"
    else:
        assert "STALE" in context.eval('elements["image-status"].textContent')
    assert context.eval("timers[0][1]") == 500
    assert "BLOCKED: all local candidate paths blocked" in context.eval('elements["controller-state"].textContent')
    assert "18.40 m remaining" in context.eval('elements["controller-state"].textContent')
    if not stale:
        assert all(field not in context.eval('elements["decision"].textContent')
                   for field in ("sampled_points", "obstacle_points", "gap_ranges"))
        assert context.eval("drawCalls") >= 2
    assert "gap_ranges" not in context.eval('elements["status"].textContent')


def test_heartbeat_request_has_a_bound_and_visible_failure_reporting():
    assert "new AbortController()" in SCRIPT
    assert "abort.abort()" in SCRIPT
    assert "3000" in SCRIPT
    assert "Heartbeat unavailable" in SCRIPT
