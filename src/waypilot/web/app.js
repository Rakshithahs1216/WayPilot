// WayPilot web UI.
//
// Data flow:
//   ROS /waypilot/status --SSE /api/events--> render()
//   user actions --fetch /api/...--> mission_manager services
// The UI holds no mission state of its own; it always renders the latest
// status pushed by the server.
'use strict';

(() => {
  const STALE_AFTER_MS = 2500;
  const TRAIL_MAX = 2000;
  const ROBOT_RADIUS_M = 0.105;
  const SNAP_M = 0.05;

  const $ = (id) => document.getElementById(id);
  const canvas = $('map');
  const ctx = canvas.getContext('2d');

  let status = null;
  let lastStatusAt = 0;
  let trail = [];
  const view = { cx: 0, cy: 0, scale: 80, autoFit: true };

  // ------------------------------------------------------------------ API
  async function api(method, path, body) {
    try {
      // Writes must be JSON: the server rejects other content types (CSRF guard).
      const res = await fetch(path, {
        method,
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body || {}),
      });
      const data = await res.json().catch(() => ({ success: false, message: res.statusText }));
      if (!res.ok || !data.success) toast(data.message || `Request failed (${res.status})`, true);
      return data;
    } catch (err) {
      toast('Cannot reach the web bridge', true);
      return { success: false };
    }
  }

  const addWaypoint = (x, y) => api('POST', '/api/waypoints', { x, y });
  const removeWaypoint = (id) => api('DELETE', `/api/waypoints/${id}`);
  const sendCommand = (cmd) => api('POST', `/api/mission/${cmd}`);

  // --------------------------------------------------------- live status
  function connect() {
    const source = new EventSource('/api/events');
    source.onmessage = (ev) => {
      status = JSON.parse(ev.data);
      lastStatusAt = Date.now();
      recordTrail();
      renderPanels();
      requestDraw();
    };
    source.onerror = () => setConn('offline', 'Disconnected — retrying…');
    // EventSource reconnects automatically.
  }

  function recordTrail() {
    const r = status.robot;
    if (!r.valid) return;
    const last = trail[trail.length - 1];
    if (!last || Math.hypot(last.x - r.x, last.y - r.y) > 0.01) {
      trail.push({ x: r.x, y: r.y });
      if (trail.length > TRAIL_MAX) trail.shift();
    }
  }

  setInterval(() => {
    if (!lastStatusAt) return;
    if (Date.now() - lastStatusAt > STALE_AFTER_MS) setConn('stale', 'No data from robot');
    else setConn('online', 'Live');
  }, 500);

  function setConn(kind, text) {
    const el = $('conn');
    el.className = `pill pill-${kind}`;
    el.textContent = text;
  }

  // -------------------------------------------------------------- panels
  const fmt = (v, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : '—');

  function renderPanels() {
    const s = status;
    const running = s.state === 'RUNNING' || s.state === 'PAUSED';

    const badge = $('state');
    badge.textContent = s.state;
    badge.className = `badge badge-${s.state}`;
    $('message').textContent = s.message;

    const r = s.robot;
    $('pos').textContent = r.valid ? `(${fmt(r.x)}, ${fmt(r.y)}) m` : 'no odometry';
    $('yaw').textContent = r.valid ? `${fmt((r.yaw * 180) / Math.PI, 1)}°` : '—';
    $('speed').textContent = r.valid ? `${fmt(r.linear_speed)} m/s · ${fmt(r.angular_speed)} rad/s` : '—';

    const active = s.active_index >= 0 ? s.waypoints[s.active_index] : null;
    $('active').textContent = active
      ? `${s.active_index + 1} of ${s.waypoints.length} · (${fmt(active.x)}, ${fmt(active.y)})`
      : '—';
    $('dist').textContent = s.distance_to_target >= 0 ? `${fmt(s.distance_to_target)} m` : '—';

    const total = s.waypoints.length;
    const done = s.waypoints.filter((w) => w.status === 'reached' || w.status === 'skipped').length;
    $('progress-bar').style.width = `${(s.progress * 100).toFixed(1)}%`;
    $('progress-text').textContent = `${done} / ${total}`;

    // Mission buttons enabled per state.
    const can = {
      start: !running && total > 0,
      pause: s.state === 'RUNNING',
      resume: s.state === 'PAUSED',
      skip: running,
      stop: running,
      clear: !running && total > 0,
    };
    document.querySelectorAll('[data-cmd]').forEach((b) => { b.disabled = !can[b.dataset.cmd]; });
    const startBtn = document.querySelector('[data-cmd="start"]');
    startBtn.textContent = s.state === 'COMPLETED' || s.state === 'ABORTED' ? 'Restart' : 'Start';
    if (s.state === 'COMPLETED' && s.waypoints.some((w) => w.status === 'pending')) startBtn.textContent = 'Continue';

    renderList(s, running);
  }

  function renderList(s, running) {
    const list = $('wp-list');
    $('wp-count').textContent = s.waypoints.length;
    $('wp-empty').hidden = s.waypoints.length > 0;

    const frag = document.createDocumentFragment();
    s.waypoints.forEach((w, i) => {
      const li = document.createElement('li');
      if (w.status === 'active') li.className = 'is-active';
      li.innerHTML = `
        <span class="wp-seq st-${w.status}">${i + 1}</span>
        <span class="mono">(${fmt(w.x)}, ${fmt(w.y)})</span>
        <span class="chip st-${w.status}">${w.status}</span>
        <button class="wp-remove" title="Remove waypoint ${i + 1}" aria-label="Remove waypoint ${i + 1}">×</button>`;
      const btn = li.querySelector('button');
      btn.disabled = running && w.status === 'active';
      btn.addEventListener('click', () => removeWaypoint(w.id));
      frag.appendChild(li);
    });
    list.replaceChildren(frag);
  }

  // ---------------------------------------------------------------- map
  let drawPending = false;
  function requestDraw() {
    if (drawPending) return;
    drawPending = true;
    requestAnimationFrame(() => { drawPending = false; draw(); });
  }

  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

  function size() {
    const dpr = window.devicePixelRatio || 1;
    const { width, height } = canvas.getBoundingClientRect();
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { w: width, h: height };
  }

  const toScreen = (x, y, w, h) => [w / 2 + (x - view.cx) * view.scale, h / 2 - (y - view.cy) * view.scale];
  const toWorld = (sx, sy, w, h) => [view.cx + (sx - w / 2) / view.scale, view.cy - (sy - h / 2) / view.scale];

  function fit(w, h) {
    const pts = [{ x: 0, y: 0 }];
    if (status) {
      pts.push(...status.waypoints);
      if (status.robot.valid) pts.push(status.robot);
    }
    pts.push(...trail);
    let minX = Infinity; let maxX = -Infinity; let minY = Infinity; let maxY = -Infinity;
    pts.forEach((p) => {
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
    });
    const pad = 1.0;
    const spanX = Math.max(maxX - minX + 2 * pad, 4);
    const spanY = Math.max(maxY - minY + 2 * pad, 4);
    view.cx = (minX + maxX) / 2;
    view.cy = (minY + maxY) / 2;
    view.scale = Math.min(w / spanX, h / spanY);
  }

  function gridStep() {
    const target = 70 / view.scale; // ~70 px between lines
    const steps = [0.1, 0.25, 0.5, 1, 2, 5, 10, 20];
    return steps.find((s) => s >= target) || 50;
  }

  function draw() {
    const { w, h } = size();
    if (view.autoFit) fit(w, h);
    ctx.clearRect(0, 0, w, h);
    drawGrid(w, h);
    if (!status) return;

    drawTrail(w, h);
    drawRoute(w, h);
    drawWaypoints(w, h);
    if (status.robot.valid) drawRobot(w, h);
  }

  function drawGrid(w, h) {
    const step = gridStep();
    const [x0, y1] = toWorld(0, 0, w, h);
    const [x1, y0] = toWorld(w, h, w, h);
    ctx.lineWidth = 1;
    ctx.font = '10px ui-monospace, monospace';
    ctx.fillStyle = css('--muted');

    for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
      const [sx] = toScreen(x, 0, w, h);
      ctx.strokeStyle = Math.abs(x) < 1e-9 ? css('--axis') : css('--grid');
      line(sx, 0, sx, h);
      ctx.fillText(`${+x.toFixed(2)}`, sx + 3, h - 4);
    }
    for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
      const [, sy] = toScreen(0, y, w, h);
      ctx.strokeStyle = Math.abs(y) < 1e-9 ? css('--axis') : css('--grid');
      line(0, sy, w, sy);
      ctx.fillText(`${+y.toFixed(2)}`, 3, sy - 3);
    }
  }

  function line(ax, ay, bx, by) {
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
  }

  function drawTrail(w, h) {
    if (trail.length < 2) return;
    ctx.strokeStyle = css('--trail');
    ctx.lineWidth = 3;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    trail.forEach((p, i) => {
      const [sx, sy] = toScreen(p.x, p.y, w, h);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }

  function drawRoute(w, h) {
    // Dashed line through the remaining route, starting at the robot.
    const remaining = status.waypoints.filter((p) => p.status === 'active' || p.status === 'pending');
    const pts = [];
    if (status.robot.valid && status.active_index >= 0) pts.push(status.robot);
    pts.push(...remaining);
    if (pts.length < 2) return;
    ctx.save();
    ctx.setLineDash([6, 6]);
    ctx.strokeStyle = css('--pending');
    ctx.globalAlpha = 0.6;
    ctx.lineWidth = 2;
    ctx.beginPath();
    pts.forEach((p, i) => {
      const [sx, sy] = toScreen(p.x, p.y, w, h);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
    ctx.restore();
  }

  function drawWaypoints(w, h) {
    ctx.font = 'bold 11px system-ui, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    status.waypoints.forEach((p, i) => {
      const [sx, sy] = toScreen(p.x, p.y, w, h);
      const r = p.status === 'active' ? 12 : 10;
      if (p.status === 'active') {
        ctx.beginPath();
        ctx.arc(sx, sy, r + 6, 0, Math.PI * 2);
        ctx.fillStyle = css('--active');
        ctx.globalAlpha = 0.25;
        ctx.fill();
        ctx.globalAlpha = 1;
      }
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, Math.PI * 2);
      ctx.fillStyle = css(`--${p.status}`) || css('--pending');
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.fillText(String(i + 1), sx, sy + 0.5);
    });
    ctx.textAlign = 'start';
    ctx.textBaseline = 'alphabetic';
  }

  function drawRobot(w, h) {
    const r = status.robot;
    const [sx, sy] = toScreen(r.x, r.y, w, h);
    const rad = Math.max(ROBOT_RADIUS_M * view.scale, 9);
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-r.yaw);
    ctx.beginPath();
    ctx.arc(0, 0, rad, 0, Math.PI * 2);
    ctx.fillStyle = css('--surface');
    ctx.fill();
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = css('--robot');
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(rad + 6, 0);
    ctx.lineTo(-rad * 0.4, -rad * 0.6);
    ctx.lineTo(-rad * 0.4, rad * 0.6);
    ctx.closePath();
    ctx.fillStyle = css('--robot');
    ctx.fill();
    ctx.restore();
  }

  // ------------------------------------------------------- map gestures
  let drag = null;
  canvas.addEventListener('pointerdown', (e) => {
    drag = { x: e.offsetX, y: e.offsetY, cx: view.cx, cy: view.cy, moved: false };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', (e) => {
    const { w, h } = size();
    const [wx, wy] = toWorld(e.offsetX, e.offsetY, w, h);
    $('cursor').textContent = `x ${fmt(wx)}, y ${fmt(wy)}`;
    if (!drag) return;
    const dx = e.offsetX - drag.x;
    const dy = e.offsetY - drag.y;
    if (!drag.moved && Math.hypot(dx, dy) < 5) return;
    drag.moved = true;
    view.autoFit = false;
    view.cx = drag.cx - dx / view.scale;
    view.cy = drag.cy + dy / view.scale;
    requestDraw();
  });
  canvas.addEventListener('pointerup', (e) => {
    if (drag && !drag.moved) {
      const { w, h } = size();
      const [wx, wy] = toWorld(e.offsetX, e.offsetY, w, h);
      const snap = (v) => Math.round(v / SNAP_M) * SNAP_M;
      addWaypoint(+snap(wx).toFixed(2), +snap(wy).toFixed(2));
    }
    drag = null;
  });
  canvas.addEventListener('pointerleave', () => { $('cursor').textContent = 'x —, y —'; });
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const { w, h } = size();
    const [wx, wy] = toWorld(e.offsetX, e.offsetY, w, h);
    view.autoFit = false;
    view.scale = Math.min(800, Math.max(5, view.scale * Math.exp(-e.deltaY * 0.0015)));
    // Keep the point under the cursor fixed while zooming.
    view.cx = wx - (e.offsetX - w / 2) / view.scale;
    view.cy = wy + (e.offsetY - h / 2) / view.scale;
    requestDraw();
  }, { passive: false });

  // --------------------------------------------------------- UI wiring
  $('btn-fit').addEventListener('click', () => { view.autoFit = true; requestDraw(); });
  $('btn-trail').addEventListener('click', () => { trail = []; requestDraw(); });

  document.querySelectorAll('[data-cmd]').forEach((btn) => {
    btn.addEventListener('click', async () => {
      const cmd = btn.dataset.cmd;
      if (cmd === 'clear' && !window.confirm('Remove all waypoints?')) return;
      const res = await sendCommand(cmd);
      if (res.success) toast(res.message);
    });
  });

  $('add-form').addEventListener('submit', async (e) => {
    e.preventDefault();
    const form = e.target;
    const x = parseFloat(form.x.value);
    const y = parseFloat(form.y.value);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    const res = await addWaypoint(x, y);
    if (res.success) { form.reset(); form.x.focus(); }
  });

  let toastTimer = null;
  function toast(text, isError = false) {
    const el = $('toast');
    el.textContent = text;
    el.className = isError ? 'toast error' : 'toast';
    el.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { el.hidden = true; }, 2800);
  }

  window.addEventListener('resize', requestDraw);
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', requestDraw);

  requestDraw();
  connect();
})();
