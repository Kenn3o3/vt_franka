from __future__ import annotations

import struct
from typing import Callable

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse, Response

from .control import OperatorActionError, SupportsOperatorUi
from .logs import OperatorLogBuffer


def create_operator_app(
    controller: SupportsOperatorUi,
    log_buffer: OperatorLogBuffer,
    *,
    title: str = "VT Franka Operator",
) -> FastAPI:
    app = FastAPI(title=title, version="0.1.0", docs_url=None, redoc_url=None, openapi_url=None)

    @app.get("/operator", response_class=HTMLResponse)
    def operator_page() -> str:
        return _OPERATOR_PAGE

    @app.get("/operator/api/status")
    def operator_status() -> dict:
        return controller.get_operator_status()

    @app.get("/operator/api/logs")
    def operator_logs(limit: int = 200) -> dict:
        return {"entries": log_buffer.get_entries(limit=max(1, min(int(limit), log_buffer.max_records)))}

    @app.get("/operator/api/snapshot/{name}")
    def operator_snapshot(name: str) -> Response:
        snapshot = controller.get_operator_snapshot(name)
        if snapshot is None:
            return Response(status_code=404)
        media_type, payload = _encode_snapshot(snapshot.image, snapshot.image_format)
        return Response(content=payload, media_type=media_type)

    _register_action(app, "/operator/api/actions/reset", controller.operator_reset_ready_pose)
    _register_action(app, "/operator/api/actions/start", controller.operator_start_episode)
    _register_action(app, "/operator/api/actions/stop", controller.operator_stop_episode)
    _register_action(app, "/operator/api/actions/discard", controller.operator_discard_latest_episode)
    _register_action(app, "/operator/api/actions/quit", controller.operator_quit)

    return app


def _register_action(app: FastAPI, path: str, action: Callable[[], None]) -> None:
    @app.post(path)
    def _action() -> JSONResponse:
        try:
            action()
        except OperatorActionError as exc:
            return JSONResponse(status_code=409, content={"detail": str(exc)})
        return JSONResponse(content={"status": "ok"})


def _encode_snapshot(image, image_format: str) -> tuple[str, bytes]:
    extension = f".{image_format.lower().lstrip('.')}"
    media_type = "image/jpeg" if extension in {".jpg", ".jpeg"} else "image/png"

    try:
        import cv2

        success, encoded = cv2.imencode(extension, image)
        if not success:
            raise RuntimeError("Failed to encode operator snapshot")
        return media_type, encoded.tobytes()
    except ImportError:
        pass

    return "image/bmp", _encode_bmp(image)


def _encode_bmp(image) -> bytes:
    height = int(image.shape[0])
    width = int(image.shape[1])
    if getattr(image, "ndim", 0) != 3 or image.shape[2] != 3:
        raise RuntimeError("Operator snapshots require HxWx3 images")

    row_stride = width * 3
    padding = (4 - (row_stride % 4)) % 4
    pixel_rows = []
    for row_index in range(height - 1, -1, -1):
        row = image[row_index].tobytes()
        pixel_rows.append(row + (b"\x00" * padding))
    pixel_data = b"".join(pixel_rows)

    dib_header_size = 40
    pixel_offset = 14 + dib_header_size
    file_size = pixel_offset + len(pixel_data)

    file_header = b"BM" + struct.pack("<IHHI", file_size, 0, 0, pixel_offset)
    dib_header = struct.pack(
        "<IIIHHIIIIII",
        dib_header_size,
        width,
        height,
        1,
        24,
        0,
        len(pixel_data),
        2835,
        2835,
        0,
        0,
    )
    return file_header + dib_header + pixel_data


_OPERATOR_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>VT Franka Operator</title>
  <style>
    :root {
      --bg: #f2eee5;
      --panel: rgba(255, 252, 246, 0.9);
      --ink: #1f2430;
      --muted: #5f6773;
      --line: rgba(31, 36, 48, 0.12);
      --ready: #1d6b44;
      --wait: #9a3d1e;
      --accent: #c74c2b;
      --shadow: 0 18px 40px rgba(58, 38, 20, 0.12);
      --font-ui: "IBM Plex Sans", "Avenir Next", "Segoe UI", sans-serif;
      --font-mono: "IBM Plex Mono", "SFMono-Regular", monospace;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: var(--font-ui);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(199, 76, 43, 0.18), transparent 30%),
        radial-gradient(circle at bottom right, rgba(29, 107, 68, 0.16), transparent 28%),
        linear-gradient(180deg, #f7f2ea, #ebe4d7);
      min-height: 100vh;
    }
    .shell {
      max-width: 1480px;
      margin: 0 auto;
      padding: 24px;
      display: grid;
      gap: 18px;
    }
    .hero, .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 22px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(12px);
    }
    .hero {
      padding: 22px 24px;
      display: grid;
      gap: 16px;
    }
    .hero-top {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 16px;
      flex-wrap: wrap;
    }
    .title {
      display: grid;
      gap: 4px;
    }
    .eyebrow {
      text-transform: uppercase;
      letter-spacing: 0.14em;
      color: var(--muted);
      font-size: 12px;
    }
    h1 {
      margin: 0;
      font-size: clamp(28px, 4vw, 44px);
      line-height: 1;
      font-weight: 700;
    }
    .badges {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }
    .badge {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      border-radius: 999px;
      padding: 10px 14px;
      font-size: 14px;
      font-weight: 600;
      border: 1px solid var(--line);
      background: rgba(255, 255, 255, 0.7);
    }
    .badge.ready { color: var(--ready); }
    .badge.wait { color: var(--wait); }
    .metrics {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 12px;
    }
    .metric {
      padding: 14px 16px;
      border-radius: 16px;
      background: rgba(255, 255, 255, 0.74);
      border: 1px solid var(--line);
    }
    .metric-label {
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.12em;
      color: var(--muted);
      margin-bottom: 8px;
    }
    .metric-value {
      font-size: 18px;
      font-weight: 600;
      word-break: break-word;
    }
    .layout {
      display: grid;
      grid-template-columns: minmax(280px, 360px) minmax(0, 1fr) minmax(320px, 420px);
      gap: 18px;
    }
    .panel {
      padding: 18px;
      display: grid;
      gap: 14px;
      min-height: 180px;
    }
    .panel h2 {
      margin: 0;
      font-size: 18px;
      font-weight: 700;
    }
    .actions {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    button {
      border: 0;
      border-radius: 14px;
      padding: 14px 16px;
      font: inherit;
      font-weight: 700;
      background: linear-gradient(135deg, #1f2430, #33415a);
      color: white;
      cursor: pointer;
      transition: transform 120ms ease, opacity 120ms ease;
    }
    button.secondary { background: linear-gradient(135deg, #9b4228, #c74c2b); }
    button.warning { background: linear-gradient(135deg, #6f2b16, #9a3d1e); }
    button:disabled {
      opacity: 0.35;
      cursor: not-allowed;
      transform: none;
    }
    button:not(:disabled):hover { transform: translateY(-1px); }
    .list {
      display: grid;
      gap: 10px;
    }
    .item {
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px 14px;
      background: rgba(255, 255, 255, 0.68);
    }
    .item strong {
      display: block;
      margin-bottom: 6px;
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
    }
    .status-chip {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 6px 10px;
      border-radius: 999px;
      font-size: 12px;
      font-weight: 700;
      background: rgba(31, 36, 48, 0.08);
    }
    .status-chip.ok { color: var(--ready); }
    .status-chip.bad { color: var(--wait); }
    .snapshot {
      min-height: 260px;
      border-radius: 18px;
      overflow: hidden;
      border: 1px solid var(--line);
      background:
        linear-gradient(135deg, rgba(31, 36, 48, 0.9), rgba(79, 55, 36, 0.8));
      display: grid;
      place-items: center;
      position: relative;
    }
    .snapshot img {
      width: 100%;
      height: 100%;
      object-fit: cover;
      display: none;
    }
    .snapshot img.visible { display: block; }
    .snapshot-empty {
      padding: 18px;
      text-align: center;
      color: rgba(255, 255, 255, 0.82);
      max-width: 280px;
      line-height: 1.45;
    }
    .snapshot-caption {
      position: absolute;
      left: 12px;
      bottom: 12px;
      background: rgba(15, 18, 22, 0.72);
      color: white;
      padding: 8px 10px;
      border-radius: 999px;
      font-size: 12px;
      display: none;
    }
    .snapshot-caption.visible { display: inline-flex; }
    .logs {
      min-height: 520px;
      max-height: 70vh;
      overflow: auto;
      border-radius: 16px;
      padding: 12px;
      background: #171c25;
      color: #dfe5ee;
      font-family: var(--font-mono);
      font-size: 12px;
      line-height: 1.45;
    }
    .log-entry {
      padding: 8px 0;
      border-bottom: 1px solid rgba(223, 229, 238, 0.08);
      white-space: pre-wrap;
    }
    .log-meta {
      color: #8ea2bb;
      margin-bottom: 4px;
    }
    .empty {
      color: var(--muted);
    }
    @media (max-width: 1180px) {
      .layout {
        grid-template-columns: 1fr;
      }
      .logs { min-height: 320px; max-height: 45vh; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <section class="hero">
      <div class="hero-top">
        <div class="title">
          <div class="eyebrow">Browser Operator Console</div>
          <h1>VT Franka</h1>
        </div>
        <div class="badges">
          <div id="modeBadge" class="badge">Mode</div>
          <div id="readyBadge" class="badge">Status</div>
        </div>
      </div>
      <div class="metrics">
        <div class="metric"><div class="metric-label">Run</div><div id="runValue" class="metric-value">-</div></div>
        <div class="metric"><div class="metric-label">Active Episode</div><div id="activeEpisodeValue" class="metric-value">-</div></div>
        <div class="metric"><div class="metric-label">Next Episode</div><div id="nextEpisodeValue" class="metric-value">-</div></div>
        <div class="metric"><div class="metric-label">Controller Age</div><div id="controllerAgeValue" class="metric-value">-</div></div>
      </div>
    </section>

    <section class="layout">
      <section class="panel">
        <h2>Actions</h2>
        <div class="actions">
          <button id="resetButton" onclick="invokeAction('reset')">Reset Pose</button>
          <button id="startButton" class="secondary" onclick="invokeAction('start')">Start Episode</button>
          <button id="stopButton" onclick="invokeAction('stop')">Stop / Save</button>
          <button id="discardButton" class="warning" onclick="invokeAction('discard', true)">Discard Latest</button>
          <button id="quitButton" class="warning" onclick="invokeAction('quit', true)">Quit</button>
        </div>
        <div>
          <h2>Blocking Reasons</h2>
          <div id="reasonsList" class="list">
            <div class="empty">No blocking reasons.</div>
          </div>
        </div>
        <div>
          <h2>Worker Health</h2>
          <div id="workersList" class="list"></div>
        </div>
      </section>

      <section class="panel">
        <h2>Pre-Episode Snapshot</h2>
        <div class="snapshot">
          <img id="snapshotImage" alt="RGB camera snapshot">
          <div id="snapshotEmpty" class="snapshot-empty">A frozen RGB snapshot will appear here when the next episode is allowed to start.</div>
          <div id="snapshotCaption" class="snapshot-caption"></div>
        </div>
        <div>
          <h2>Session State</h2>
          <div id="statusList" class="list"></div>
        </div>
      </section>

      <section class="panel">
        <h2>Recent Logs</h2>
        <div id="logsPanel" class="logs"></div>
      </section>
    </section>
  </main>

  <script>
    let lastSnapshotToken = null;

    async function fetchJson(path, options = undefined) {
      const response = await fetch(path, options);
      const data = response.status === 204 ? {} : await response.json().catch(() => ({}));
      if (!response.ok) {
        throw new Error(data.detail || `Request failed: ${response.status}`);
      }
      return data;
    }

    async function refreshStatus() {
      const status = await fetchJson('/operator/api/status');
      renderStatus(status);
      await refreshSnapshot(status);
    }

    async function refreshLogs() {
      const data = await fetchJson('/operator/api/logs?limit=200');
      const panel = document.getElementById('logsPanel');
      if (!data.entries.length) {
        panel.innerHTML = '<div class="empty">No recent logs.</div>';
        return;
      }
      panel.innerHTML = data.entries.map((entry) => {
        const stamp = new Date(entry.created * 1000).toLocaleTimeString();
        return `<div class="log-entry"><div class="log-meta">[${stamp}] ${entry.level} ${entry.logger}</div>${escapeHtml(entry.message)}</div>`;
      }).join('');
      panel.scrollTop = panel.scrollHeight;
    }

    function selectSnapshot(status) {
      const snapshots = status.snapshots || {};
      const preferredRoles = ['third_person', 'wrist'];
      for (const role of preferredRoles) {
        if (snapshots[role] && snapshots[role].available) {
          return { role, snapshot: snapshots[role] };
        }
      }
      for (const [role, snapshot] of Object.entries(snapshots)) {
        if (snapshot && snapshot.available) {
          return { role, snapshot };
        }
      }
      return null;
    }

    async function refreshSnapshot(status) {
      const selected = selectSnapshot(status);
      const image = document.getElementById('snapshotImage');
      const empty = document.getElementById('snapshotEmpty');
      const caption = document.getElementById('snapshotCaption');
      if (!selected) {
        lastSnapshotToken = null;
        image.classList.remove('visible');
        image.removeAttribute('src');
        empty.style.display = 'block';
        caption.classList.remove('visible');
        caption.textContent = '';
        return;
      }
      const { role, snapshot } = selected;
      if (snapshot.token !== lastSnapshotToken) {
        image.src = `/operator/api/snapshot/${encodeURIComponent(role)}?token=${encodeURIComponent(snapshot.token)}`;
        lastSnapshotToken = snapshot.token;
      }
      image.classList.add('visible');
      empty.style.display = 'none';
      caption.textContent = snapshot.label || `Frozen ${role} view`;
      caption.classList.add('visible');
    }

    function renderStatus(status) {
      document.getElementById('modeBadge').textContent = (status.mode || 'unknown').toUpperCase();
      const readyBadge = document.getElementById('readyBadge');
      readyBadge.textContent = status.ready ? 'READY' : 'WAIT';
      readyBadge.className = `badge ${status.ready ? 'ready' : 'wait'}`;
      document.getElementById('runValue').textContent = status.run_name || '-';
      document.getElementById('activeEpisodeValue').textContent = status.active_episode_name || 'off';
      document.getElementById('nextEpisodeValue').textContent = status.next_episode_name || '-';

      const controllerAge = status.controller_state && status.controller_state.age_sec;
      document.getElementById('controllerAgeValue').textContent = controllerAge == null ? 'n/a' : `${controllerAge.toFixed(3)} s`;

      renderReasons(status.reasons || []);
      renderWorkers(status.workers || {});
      renderSessionDetails(status);
      renderActions(status.allowed_actions || {});
    }

    function renderReasons(reasons) {
      const list = document.getElementById('reasonsList');
      if (!reasons.length) {
        list.innerHTML = '<div class="empty">No blocking reasons.</div>';
        return;
      }
      list.innerHTML = reasons.map((reason) => `<div class="item">${escapeHtml(reason)}</div>`).join('');
    }

    function renderWorkers(workers) {
      const list = document.getElementById('workersList');
      const items = Object.entries(workers);
      if (!items.length) {
        list.innerHTML = '<div class="empty">No background workers.</div>';
        return;
      }
      list.innerHTML = items.map(([name, worker]) => {
        const ok = worker.alive && !worker.error;
        const error = worker.error ? `<div>${escapeHtml(worker.error)}</div>` : '';
        return `<div class="item"><strong>${escapeHtml(name)}</strong><span class="status-chip ${ok ? 'ok' : 'bad'}">${ok ? 'healthy' : 'attention'}</span>${error}</div>`;
      }).join('');
    }

    function renderSessionDetails(status) {
      const details = [];
      details.push(renderDetail('Quest', status.quest_connected ? 'connected' : 'missing'));
      details.push(renderDetail('Teleop', status.teleop_enabled ? 'enabled' : 'blocked'));
      details.push(renderDetail('Recording', status.active_episode_name || 'off'));
      if (status.latest_saved_episode_name) {
        details.push(renderDetail('Latest Saved', status.latest_saved_episode_name));
      }
      if (status.preview_note) {
        details.push(renderDetail('Snapshot', status.preview_note));
      }
      const controller = status.controller_state || {};
      details.push(renderDetail('Controller Samples', `${controller.sample_count || 0}`));
      details.push(renderDetail('Controller Failures', `${controller.failure_count || 0}`));
      document.getElementById('statusList').innerHTML = details.join('');
    }

    function renderDetail(label, value) {
      return `<div class="item"><strong>${escapeHtml(label)}</strong>${escapeHtml(value ?? '-')}</div>`;
    }

    function renderActions(allowed) {
      document.getElementById('resetButton').disabled = !allowed.reset;
      document.getElementById('startButton').disabled = !allowed.start;
      document.getElementById('stopButton').disabled = !allowed.stop;
      document.getElementById('discardButton').disabled = !allowed.discard;
      document.getElementById('quitButton').disabled = !allowed.quit;
    }

    async function invokeAction(action, confirmFirst = false) {
      if (confirmFirst) {
        const confirmed = window.confirm(`Run ${action}?`);
        if (!confirmed) {
          return;
        }
      }
      try {
        await fetchJson(`/operator/api/actions/${action}`, { method: 'POST' });
        await refreshStatus();
        await refreshLogs();
      } catch (error) {
        window.alert(error.message);
      }
    }

    function escapeHtml(value) {
      return String(value)
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;');
    }

    async function refreshAll() {
      try {
        await Promise.all([refreshStatus(), refreshLogs()]);
      } catch (error) {
        console.error(error);
      }
    }

    refreshAll();
    setInterval(refreshAll, 1000);
  </script>
</body>
</html>
"""
