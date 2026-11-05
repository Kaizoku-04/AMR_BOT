// OpenAMR warehouse dashboard: polls the WMS (/api/v1/live once a second) and draws the lane graph as inline SVG.
// Plain JavaScript, no libraries, nothing from the internet (ADR-046).
'use strict';
const $ = (s) => document.querySelector(s);
const NS = 'http://www.w3.org/2000/svg';
const COLORS = { IDLE: 'var(--idle)', TO_PICK: 'var(--drive)', TO_DROP: 'var(--drive)', AT_STATION: 'var(--ok)',
  LOADED: 'var(--drive)', CHARGING: 'var(--charge)', TO_CHARGE: 'var(--charge)', YIELDING: 'var(--warn)',
  STUCK: 'var(--err)', ISOLATED: 'var(--err)' };
const ST_COLORS = { IDLE: 'var(--ok)', BUSY: 'var(--info)', FAULT: 'var(--err)', STARVED: 'var(--warn)' };
let layout = null, nodePos = {}, nodeName = {}, view = null;
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const pct = (v) => `${Math.round(100 * (v || 0))} %`;
const el = (tag, attrs, parent) => { const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v); if (parent) parent.appendChild(e); return e; };

async function get(path) { const r = await fetch(path); if (!r.ok) throw new Error(`${path}: ${r.status}`); return r.json(); }

function drawLayout() {
  const svg = $('#map');
  const xs = layout.nodes.map((n) => n.x), ys = layout.nodes.map((n) => n.y);
  const pad = 1.5, x0 = Math.min(...xs) - pad, x1 = Math.max(...xs) + pad, y0 = Math.min(...ys) - pad, y1 = Math.max(...ys) + pad;
  view = { x0, y1 };
  svg.setAttribute('viewBox', `0 0 ${x1 - x0} ${y1 - y0}`);
  layout.nodes.forEach((n) => { nodePos[n.id] = [n.x - x0, y1 - n.y]; nodeName[n.id] = n.name; });
  const g = el('g', { id: 'lanes' }, svg);
  layout.edges.forEach(([a, b]) => { const [ax, ay] = nodePos[a], [bx, by] = nodePos[b];
    el('line', { x1: ax, y1: ay, x2: bx, y2: by, stroke: 'var(--lane)', 'stroke-width': 0.06 }, g); });
  layout.nodes.forEach((n) => {
    const [x, y] = nodePos[n.id];
    if (n.kind === 'aisle_bay') {
      for (const side of ['W', 'E']) el('rect', { id: `bin-${n.name}:${side}`, x: x + (side === 'W' ? -0.75 : 0.35), y: y - 0.2,
        width: 0.4, height: 0.4, rx: 0.05, fill: 'var(--empty)' }, g);
    } else if (n.kind === 'charger' || n.kind === 'parking') {
      el('rect', { x: x - 0.25, y: y - 0.25, width: 0.5, height: 0.5, fill: 'none', stroke: n.kind === 'charger' ?
        'var(--charge)' : 'var(--idle)', 'stroke-width': 0.06 }, g);
    } else if (n.kind === 'bay') {
      el('rect', { id: `bay-${n.name}`, x: x - 0.45, y: y - 0.45, width: 0.9, height: 0.9, rx: 0.1, fill: 'var(--idle)',
        opacity: 0.85 }, g);
      el('text', { x: x, y: y - 0.65, 'font-size': 0.55, 'text-anchor': 'middle', fill: 'var(--muted)' }, g).textContent =
        n.name.replace('_bay', '');
    }
  });
  el('g', { id: 'robots' }, svg);
  $('#legend').innerHTML = Object.entries({ driving: 'var(--drive)', 'at station': 'var(--ok)', charging: 'var(--charge)',
    waiting: 'var(--warn)', 'stuck / silent': 'var(--err)', idle: 'var(--idle)' })
    .map(([k, c]) => `<span><i style="background:${c}"></i>${k}</span>`).join('');
}

function drawLive(d) {
  for (const [b, full] of Object.entries(d.bins)) { const r = document.getElementById(`bin-${b}`);
    if (r) r.setAttribute('fill', full ? 'var(--full)' : 'var(--empty)'); }
  d.stations.forEach((s) => { const r = document.getElementById(`bay-${s.bay}`);
    if (r) r.setAttribute('fill', s.stale ? 'var(--err)' : ST_COLORS[s.state] || 'var(--idle)'); });
  const g = $('#robots'); g.innerHTML = '';
  d.robots.forEach((r) => {
    const x = r.x - view.x0, y = view.y1 - r.y;
    const c = r.silent ? 'var(--err)' : COLORS[r.status] || 'var(--idle)';
    if (r.route.length) { let pts = [[x, y]]; r.route.forEach((n) => nodePos[n] && pts.push(nodePos[n]));
      el('polyline', { points: pts.map((p) => p.join(',')).join(' '), fill: 'none', stroke: c, 'stroke-width': 0.08,
        'stroke-dasharray': '0.2 0.15', opacity: 0.7 }, g); }
    const t = el('g', { transform: `translate(${x},${y}) rotate(${-r.theta * 180 / Math.PI})` }, g);
    el('rect', { x: -0.4, y: -0.32, width: 0.8, height: 0.64, rx: 0.12, fill: c, stroke: r.silent ? 'var(--err)' : 'none',
      'stroke-width': 0.1, 'stroke-dasharray': r.silent ? '0.15 0.1' : '' }, t);
    el('path', { d: 'M0.25,-0.18 L0.4,0 L0.25,0.18 Z', fill: '#fff' }, t);
    if (r.loaded) el('rect', { x: -0.18, y: -0.18, width: 0.36, height: 0.36, fill: '#d9a441' }, t);
    const label = el('text', { x: x, y: y + 0.95, 'font-size': 0.55, 'text-anchor': 'middle', fill: 'var(--ink)' }, g);
    label.textContent = `${r.id.replace('amr_', '')} ${Math.round(100 * r.battery)}%`;
  });
}

function drawPanels(d) {
  const k = d.kpis;
  const tiles = [['Boxes / h (fleet)', k.boxes_per_h.toFixed(1)], ['Boxes moved', `${k.by_kind.inbound} in · ${k.by_kind.outbound} out`],
    ['Latency mean / p95', `${Math.round(k.latency_mean)} / ${Math.round(k.latency_p95)} s`],
    ['Driving · waiting', `${pct(k.shares.driving)} · ${pct(k.shares.yielding)}`], ['Charging', pct(k.shares.charging)],
    ['Battery lowest / mean', `${pct(k.battery_min)} / ${pct(k.battery_mean)}`]];
  if (k.orders) tiles.push(['Order fill rate', pct(k.orders.fill_rate)], ['Late orders', k.orders.late_orders]);
  tiles.push(['Stranded boxes', k.stranded]);
  $('#kpis').innerHTML = tiles.map(([n, v]) => `<div class="tile"><div class="v">${esc(v)}</div><div class="k">${esc(n)}</div></div>`).join('');
  $('#alarms').innerHTML = d.alarms.length ? d.alarms.map((a) => `<li class="lvl-${a.level}">${esc(a.text)}</li>`).join('')
    : '<li class="muted">No alarms</li>';
  $('#robots tbody').innerHTML = d.robots.map((r) => `<tr><td>${esc(r.id)}</td><td style="color:${r.silent ? 'var(--err)' : ''}">${r.silent ? 'SILENT' : esc(r.status)}${r.loaded ? ' · box' : ''}</td>
    <td><span class="bar"><b style="width:${Math.round(100 * r.battery)}%;background:${r.battery < 0.3 ? 'var(--warn)' : 'var(--ok)'}"></b></span> ${pct(r.battery)}</td>
    <td>${esc(r.task)}${r.mission && !r.mission.startsWith('m') ? ` (${esc(r.mission)})` : ''}</td><td>${(k.per_robot_h[r.id] || 0).toFixed(1)}</td></tr>`).join('');
  $('#arms tbody').innerHTML = d.stations.map((s) => { const a = k.arms[s.bay] || {};
    return `<tr><td>${esc(s.id)}</td><td style="color:${s.stale ? 'var(--err)' : ST_COLORS[s.state]}">${s.stale ? 'SILENT' : esc(s.state)}</td>
    <td>${pct(a.busy)}</td><td>${pct(a.starved)}</td><td>${pct(a.fault)}</td><td>${esc(s.pallets.join(' / '))}</td></tr>`; }).join('');
  $('#tasks tbody').innerHTML = d.tasks.slice(0, 40).map((t) => `<tr><td>${esc(t.id)}</td><td>${esc(t.kind)}</td><td>${esc(t.origin)}</td>
    <td>${esc(t.bin || t.dest)}</td><td>${esc(t.order)}</td><td>${t.priority}</td><td>${esc(t.state)}</td><td>${esc(t.robot)}</td></tr>`).join('');
  $('#events').innerHTML = d.events.slice().reverse().map((e) => `<li>t=${Math.round(e.t)} s <b>${esc(e.who)}</b> ${esc(e.what)}${e.why ? ' — ' + esc(e.why) : ''}</li>`).join('');
  const s = Math.round(d.now); $('#clock').textContent = `WMS clock ${Math.floor(s / 3600)}:${String(Math.floor(s / 60) % 60).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}

async function tick() {
  try {
    if (!layout) { layout = await get('/api/v1/layout'); drawLayout();
      const cmds = ['RELEASE_ROBOT', 'ACK_FAULT', 'PAUSE', 'RESUME', 'OUT_OF_SERVICE', 'IN_SERVICE'];
      $('#cmd select').innerHTML = cmds.map((c) => `<option>${c}</option>`).join(''); }
    const d = await get('/api/v1/live'); drawLive(d); drawPanels(d);
  } catch (e) { $('#clock').textContent = `WMS not reachable (${e.message})`; }
}

$('#cmd').addEventListener('submit', async (ev) => {
  ev.preventDefault();
  const body = Object.fromEntries(new FormData(ev.target).entries());
  const r = await fetch('/api/v1/operator', { method: 'POST', body: JSON.stringify(body) });
  const j = await r.json(); $('#cmdres').textContent = r.ok ? `sent ${j.command} ${j.target || 'fleet'}` : j.error;
});
tick(); setInterval(tick, 1000);
