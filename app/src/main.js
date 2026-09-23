import './style.css';
import { TurbineScene } from './scene.js';
import { Dynamics, T5, powerCurve, aep, steady } from './physics.js';
import { TwinSystem, REC_PER_DAY, fmtDay } from './twin.js';
import { Chart, fftMag } from './charts.js';

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const eur = (v) => { const a = Math.abs(v), s = v < 0 ? '−' : ''; return a >= 1e6 ? `${s}€${(a / 1e6).toFixed(2)}m` : a >= 1e3 ? `${s}€${Math.round(a / 1e3)}k` : `${s}€${Math.round(a)}`; };
const fmt = (v, d = 0) => Number(v).toLocaleString('en-GB', { minimumFractionDigits: d, maximumFractionDigits: d });

// --------------------------------------------------------------- state
let twin = new TwinSystem({ seed: 11 });
twin._newEvents = [];
twin.advance(3 * REC_PER_DAY);                 // three days of history so charts start with data
const dyn = new Dynamics();
{ const s0 = steady(twin.last.U, { rho: twin.last.rho }); dyn.w = Math.max(T5.wMin, s0.w); dyn.beta = s0.beta * Math.PI / 180; dyn.integ = dyn.beta / ((1 / (1 + dyn.beta / T5.thetaK)) * T5.KI); dyn.Qgen = s0.Q || 0; dyn.x = s0.thrust / T5.kTower; }
let Uf = twin.last.U, dirF = twin.last.dir, speed = 6, playing = true, acc = 0;
const narrow = window.matchMedia('(max-width: 760px)').matches;

const scene = new TurbineScene($('#viewport'));
if (narrow) { setMode('twin'); } else setMode('split');
scene.goTo('overview');

// --------------------------------------------------------------- controls
function setMode(m) {
  scene.setMode(m); $('#viewport').dataset.mode = m;
  $$('#modeSeg button').forEach(b => b.classList.toggle('on', b.dataset.mode === m));
}
$$('#modeSeg button').forEach(b => b.onclick = () => setMode(b.dataset.mode));
$$('#camSeg button').forEach(b => b.onclick = () => scene.goTo(b.dataset.cam));
function setSpeed(s) { speed = s; $$('#speedSeg button').forEach(b => b.classList.toggle('on', +b.dataset.speed === s)); }
$$('#speedSeg button').forEach(b => b.onclick = () => { setSpeed(+b.dataset.speed); if (!playing) togglePlay(); });
setSpeed(6);
function togglePlay() { playing = !playing; $('#playBtn').textContent = playing ? '❚❚' : '►'; $('#playBtn').setAttribute('aria-label', playing ? 'Pause' : 'Play'); }
$('#playBtn').onclick = togglePlay;
$('#flowChk').onchange = (e) => scene.showFlow = e.target.checked;
$('#exagChk').onchange = (e) => scene.exag = e.target.checked ? 15 : 1;

const CAM_FOR = { bearing: 'nacelle', yaw: 'overview', icing: 'rotor', wake: 'farm' };
$$('.scen-card').forEach(card => {
  card.querySelector('button').onclick = () => {
    const name = card.dataset.scen;
    twin.inject(name, name === 'yaw' ? { offset: 8 } : {});
    card.classList.remove('done'); card.classList.add('running'); card.querySelector('button').textContent = 'Running…';
    setSpeed(144); if (!playing) togglePlay();
    if (scene.mode === 'physical') setMode(narrow ? 'twin' : 'split');
    scene.goTo(CAM_FOR[name]);
    renderLog(true);
  };
});
$('#resetBtn').onclick = () => {
  twin = new TwinSystem({ seed: 11 + Math.floor(Math.random() * 1000) }); twin._newEvents = []; twin.advance(3 * REC_PER_DAY);
  $$('.scen-card').forEach(c => { c.classList.remove('running', 'done'); c.querySelector('button').textContent = 'Inject'; });
  seenEvents = 0; renderLog(); setSpeed(6);
};

// --------------------------------------------------------------- readouts
const RO = [
  ['wind', 'Wind at hub'], ['power', 'Power'], ['rotor', 'Rotor'], ['thrust', 'Rotor thrust'],
  ['bearing', 'HSS bearing'], ['vib', 'Bearing vibration'], ['air', 'Air'], ['life', 'Tower fatigue life'],
];
$('#readouts').innerHTML = RO.map(([k, l]) => `<div class="ro" id="ro-${k}"><div class="l">${l}</div><div class="v"></div><div class="d"></div></div>`).join('');
function ro(k, v, d, cls = '') { const e = $(`#ro-${k}`); e.className = 'ro ' + cls; e.children[1].innerHTML = v; e.children[2].innerHTML = d; }

// --------------------------------------------------------------- event log
let seenEvents = 0;
function renderLog(force) {
  const ev = twin.events;
  const html = ev.slice(0, 40).map((e, i) => `<li class="${e.type} ${i < ev.length - seenEvents ? 'new' : ''}"><time>${fmtDay(e.day)}</time><b>${e.title}</b>${e.detail}</li>`).join('');
  $('#log').innerHTML = html || '<li class="muted">Normal operation. The twin compares every 10-minute record with its physics model.</li>';
  seenEvents = ev.length;
}

// --------------------------------------------------------------- charts
const cPower = new Chart($('#cPower'), { pad: { l: 40 } });
const cA = new Chart($('#cResidA'), { pad: { l: 40, b: 18, t: 8 } });
const cB = new Chart($('#cResidB'), { pad: { l: 40, b: 22, t: 8 } });
const cSpec = new Chart($('#cSpec'), { pad: { l: 44 } });
const cLife = new Chart($('#cLife'), { pad: { l: 40 } });
const pcCache = new Map();
function twinCurve(rho) {
  const k = Math.round(rho * 100) / 100;
  if (!pcCache.has(k)) pcCache.set(k, powerCurve({ rho: k }, 25.5, 0.25).map(([U, P]) => [U, Math.min(P, 5000) / 1000]));
  return pcCache.get(k);
}
function drawPower() {
  const h = twin.hist, now = twin.day;
  const recent = h.filter(r => r.day > now - 7 && !r.stopped);
  cPower.frame({ x0: 0, x1: 25, y0: 0, y1: 5.6, xTicks: [0, 5, 10, 15, 20, 25], yTicks: [0, 1, 2, 3, 4, 5], xLabel: 'wind speed, m/s', yLabel: 'MW' });
  const n = recent.length;
  cPower.dots(recent.map(r => [r.Unac, r.P / 1000]), css('--accent'), 1.7, (i) => 0.12 + 0.6 * (i / n) ** 2);
  const last = recent.filter(r => r.day > now - 0.5);
  cPower.dots(last.map(r => [r.Unac, r.P / 1000]), css('--warn'), 2.2, () => 0.9);
  cPower.line(twinCurve(twin.last.rho), '#ffffff', 2);
  cPower.text(15.5, 3.9, '▬ twin (physics)', '#ffffff');
  cPower.text(15.5, 3.3, '● last 12 h', css('--warn'));
  cPower.text(15.5, 2.7, '● last 7 days', css('--accent'));
  const day = h.filter(r => r.day > now - 1);
  const E = day.reduce((a, r) => a + r.P / 6000, 0), Ee = day.reduce((a, r) => a + r.Pexp / 6000, 0);
  $('#fPower').innerHTML = `Last 24 h: produced <b>${fmt(E, 1)} MWh</b>, twin expected <b>${fmt(Ee, 1)} MWh</b> (${Ee > 0 ? fmt(100 * E / Ee, 1) : '–'} %).`;
}
function drawResid() {
  const h = twin.hist, now = twin.day, x0 = Math.max(0, now - 10);
  const pts = h.filter(r => r.day > x0);
  const ticks = []; for (let d = Math.ceil(x0); d <= now; d += Math.max(1, Math.round((now - x0) / 5))) ticks.push(d);
  cA.frame({ x0, x1: Math.max(now, x0 + 1), y0: 0.9, y1: 1.04, xTicks: [], yTicks: [0.92, 0.96, 1.0], yFmt: (v) => v.toFixed(2), yLabel: 'P ÷ twin' });
  cA.hline(0.988, css('--bad'), 'alert band');
  cA.line(pts.map(r => [r.day, r.powerRatio]), css('--accent'), 2);
  cB.frame({ x0, x1: Math.max(now, x0 + 1), y0: -2, y1: 16, xTicks: ticks, yTicks: [0, 5, 10, 15], xFmt: (d) => `day ${d}`, yLabel: '°C above twin' });
  cB.hline(3, css('--bad'), 'alert 3 °C');
  cB.dots(pts.map(r => [r.day, r.Tb - r.TbExp]), css('--muted'), 1.2, () => 0.35);
  cB.line(pts.map(r => [r.day, r.ewmaT]), css('--warn'), 2);
  for (const e of twin.events) if (e.type === 'alert' && e.day > x0) { cA.vline(e.day, css('--bad')); cB.vline(e.day, css('--bad')); }
  const L = twin.last;
  $('#fResid').innerHTML = `Power ratio <b>${L.powerRatio ? L.powerRatio.toFixed(3) : '–'}</b> · bearing <b>${(L.ewmaT >= 0 ? '+' : '') + L.ewmaT.toFixed(1)} °C</b> vs twin`;
}
// live vibration: synthesise the high-speed-shaft bearing signal and take its spectrum
const FS = 512, NS = 1024, sig = new Float64Array(NS);
let specPhase = 0;
function drawSpec(out) {
  const L = twin.last, f1 = Math.max(0.01, out.genRpm / 60), bpfo = 5.12 * f1, load = out.P / 5e6;
  const Ab = Math.max(0, L.vib - L.vibExp) * 1.4 + 0.004 * (out.genRpm > 50);
  for (let i = 0; i < NS; i++) {
    const t = specPhase + i / FS, w = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / (NS - 1));
    let v = 0;
    if (out.genRpm > 50) {
      v += 0.03 * Math.sin(2 * Math.PI * f1 * t) + 0.012 * Math.sin(2 * Math.PI * 2 * f1 * t + 1);
      v += Ab * (Math.sin(2 * Math.PI * bpfo * t) + 0.45 * Math.sin(2 * Math.PI * 2 * bpfo * t + 0.3) + 0.35 * Math.sin(2 * Math.PI * (bpfo - f1) * t) + 0.35 * Math.sin(2 * Math.PI * (bpfo + f1) * t));
    }
    v += (0.01 + 0.012 * load) * (Math.random() * 2 - 1) * 1.7;
    sig[i] = v * w * 2;
  }
  specPhase += NS / FS;
  const mag = fftMag(sig);
  const pts = []; for (let k = 1; k < NS / 2; k++) { const f = k * FS / NS; if (f > 200) break; pts.push([f, mag[k]]); }
  const ymax = Math.max(0.06, Math.min(0.6, Math.max(...pts.map(p => p[1])) * 1.25));
  cSpec.frame({ x0: 0, x1: 200, y0: 0, y1: ymax, xTicks: [0, 50, 100, 150, 200], yTicks: [0, ymax / 2, ymax].map(v => +v.toFixed(2)), yFmt: (v) => v.toFixed(2), xLabel: 'Hz', yLabel: 'g' });
  cSpec.area(pts, 'rgba(58,208,240,.18)'); cSpec.line(pts, css('--accent'), 1.4);
  if (out.genRpm > 50) {
    cSpec.vline(f1, css('--muted'), '1× shaft');
    cSpec.vline(bpfo, Ab > 0.03 ? css('--bad') : css('--muted'), 'BPFO');
  }
  const ratio = L.vib / L.vibExp;
  $('#fSpec').innerHTML = out.genRpm > 50 ? `Shaft <b>${f1.toFixed(1)} Hz</b> · outer-race frequency <b>${bpfo.toFixed(0)} Hz</b> · envelope <b>${ratio.toFixed(1)} ×</b> twin baseline${ratio > 2.2 ? ' <span style="color:var(--bad)">(defect signature)</span>' : ''}` : 'Rotor stopped.';
}
function lifeRates() {
  const r7 = twin.life.rate7; const rate = r7.length ? r7.reduce((a, b) => a + b, 0) / r7.length * REC_PER_DAY * 365 : 0.05;
  const ws = twin.wakeSummary();
  return { rate, rateBase: ws ? ws.rateBase : rate, ws };
}
function drawLife() {
  const used = twin.life.used, age = twin.age + twin.day / 365;
  const { rate, rateBase, ws } = lifeRates();
  const lifeT = age + (1 - used) / Math.max(rate, 1e-6), lifeB = age + (1 - twin.life.usedBase) / Math.max(rateBase, 1e-6);
  const x1 = Math.min(45, Math.max(32, Math.ceil(Math.max(lifeT, lifeB) / 5) * 5 + 2));
  cLife.frame({ x0: 0, x1, y0: 0, y1: 1.05, xTicks: [0, 10, 20, 30, 40].filter(v => v <= x1), yTicks: [0, 0.25, 0.5, 0.75, 1], yFmt: (v) => `${v * 100}%`, xLabel: 'turbine age, years', yLabel: 'life used' });
  cLife.vline(20, css('--muted'), 'design life 20 y');
  cLife.hline(1, css('--bad'), '');
  cLife.line([[0, 0], [twin.age, twin.lifeUsed0], [age, used]], '#ffffff', 2.2);
  if (ws) cLife.line([[age, twin.life.usedBase], [lifeB, 1]], css('--bad'), 1.8, [5, 4]);
  cLife.line([[age, used], [lifeT, 1]], css('--good'), 2, [5, 4]);
  cLife.text(Math.min(lifeT, x1 - 1), 0.93, `${lifeT.toFixed(1)} y`, css('--good'), 'right');
  $('#fLife').innerHTML = `Used <b>${(used * 100).toFixed(1)} %</b> after ${age.toFixed(1)} years. Current rate <b>${(rate * 20).toFixed(2)} ×</b> design; projected life <b>${lifeT.toFixed(1)} years</b>` +
    (ws ? ` (without load-aware control: <b style="color:var(--bad)">${lifeB.toFixed(1)}</b>)` : '') + '.';
}

// --------------------------------------------------------------- ecosystem diagram
const NODES = [
  { id: 'asset', x: 20, t: 'Physical turbine', s: 'NREL 5 MW' },
  { id: 'edge', x: 200, t: 'Sensors & edge', s: 'SCADA · vibration · strain' },
  { id: 'data', x: 380, t: 'Data platform', s: 'history · weather · orders' },
  { id: 'model', x: 560, t: 'Physics twin', s: 'aero · control · loads' },
  { id: 'ai', x: 740, t: 'Analytics', s: 'residuals · remaining life' },
  { id: 'act', x: 920, t: 'Decisions', s: 'repair · control · heating' },
];
const PARTNERS = [['OEM service', 900], ['Crane & logistics', 760], ['Spare parts', 620], ['Grid & power market', 440], ['Insurer', 300], ['Fleet benchmark', 150]];
function buildEco() {
  const svg = $('#ecoSvg'); const W = 160, Y = 120, H = 96;
  let s = `<defs><marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#3ad0f0"/></marker>
  <marker id="arrG" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="#46d69b"/></marker></defs>`;
  PARTNERS.forEach(([n, x], i) => {
    s += `<line x1="${x + 60}" y1="44" x2="${i < 3 ? 1000 : 460}" y2="${Y}" stroke="#2d3b49" stroke-dasharray="3 4"/>`;
    s += `<rect x="${x}" y="22" width="120" height="24" rx="12" fill="#151e27" stroke="#2d3b49"/><text x="${x + 60}" y="38" text-anchor="middle" fill="#8c9aab" font-size="11.5">${n}</text>`;
  });
  NODES.forEach((n, i) => {
    s += `<g id="eco-${n.id}"><rect class="nb" x="${n.x}" y="${Y}" width="${W}" height="${H}" rx="12" fill="#10171e" stroke="#3ad0f0" stroke-opacity=".55"/>
      <text x="${n.x + 12}" y="${Y + 24}" fill="#e7edf3" font-size="14" font-weight="650">${n.t}</text>
      <text x="${n.x + 12}" y="${Y + 42}" fill="#8c9aab" font-size="10.5">${n.s}</text>
      <text class="live" x="${n.x + 12}" y="${Y + 70}" fill="#3ad0f0" font-size="15" font-weight="700"></text>
      <text class="live2" x="${n.x + 12}" y="${Y + 86}" fill="#8c9aab" font-size="10.5"></text></g>`;
    if (i < NODES.length - 1) {
      const x1 = n.x + W + 2, x2 = NODES[i + 1].x - 4;
      s += `<path id="p${i}" d="M${x1},${Y + H / 2} L${x2},${Y + H / 2}" stroke="#3ad0f0" stroke-width="2" marker-end="url(#arr)"/>`;
      s += `<circle r="3.5" fill="#7ff3ff"><animateMotion dur="${1.1 + i * 0.1}s" repeatCount="indefinite" path="M${x1},${Y + H / 2} L${x2},${Y + H / 2}"/></circle>`;
    }
  });
  const fb = `M${NODES[5].x + W / 2},${Y + H + 2} C${NODES[5].x + W / 2},${Y + H + 90} ${NODES[0].x + W / 2},${Y + H + 90} ${NODES[0].x + W / 2},${Y + H + 6}`;
  s += `<path d="${fb}" fill="none" stroke="#46d69b" stroke-width="2.2" marker-end="url(#arrG)"/>`;
  s += `<circle r="4.5" fill="#46d69b"><animateMotion dur="3.6s" repeatCount="indefinite" path="${fb}"/></circle>`;
  s += `<text x="550" y="${Y + H + 82}" text-anchor="middle" fill="#46d69b" font-size="12.5" font-weight="650">Feedback to the asset: repair windows, yaw offset, blade heating, load-aware control</text>`;
  svg.innerHTML = s;
}
buildEco();
let ecoFlash = {};
function updateEco(out) {
  const c = twin.counters, set = (id, a, b) => { const g = $(`#eco-${id}`); g.querySelector('.live').textContent = a; g.querySelector('.live2').textContent = b; };
  set('asset', `${(out.P / 1e6).toFixed(2)} MW`, `${out.rpm.toFixed(1)} rpm · ${out.U.toFixed(1)} m/s`);
  set('edge', fmt(c.sensorValues), 'sensor values sent');
  set('data', fmt(c.records), `10-min records · ${twin.day.toFixed(1)} days`);
  set('model', fmt(c.records * 3), 'physics evaluations');
  set('ai', `${c.alerts} alerts`, 'from residuals');
  const v = computeValue();
  set('act', `${c.actions} actions`, v.sim > 0 ? `${eur(v.sim)} value this run` : 'waiting for a finding');
  for (const id of ['ai', 'act']) {
    const r = $(`#eco-${id} .nb`); const on = ecoFlash[id] && performance.now() - ecoFlash[id] < 2500;
    r.setAttribute('stroke', on ? '#ffb84d' : '#3ad0f0'); r.setAttribute('stroke-width', on ? 3 : 1);
  }
}

// --------------------------------------------------------------- value model
const AEP = aep(); // MWh / year, design climate 7.5 m/s
const A = {};
function readAssumptions() {
  A.price = +$('#aPrice').value; A.fleet = +$('#aFleet').value; A.pf = +$('#aFail').value; A.yaw = +$('#aYaw').value;
  A.ice = +$('#aIce').value; A.wake = +$('#aWake').value; A.cost = +$('#aCost').value;
  $('#oPrice').textContent = `€${A.price}/MWh`; $('#oFleet').textContent = A.fleet; $('#oFail').textContent = A.pf.toFixed(2);
  $('#oYaw').textContent = `${Math.round(A.yaw * 100)} %`; $('#oIce').textContent = A.ice; $('#oWake').textContent = `${Math.round(A.wake * 100)} %`; $('#oCost').textContent = eur(A.cost);
}
$$('.assumptions input').forEach(i => i.oninput = () => { readAssumptions(); renderValue(); });
readAssumptions();
function computeValue() {
  const p = A.price, V = twin.value, rows = [];
  let sim = 0;
  // bearing
  const b = V.bearing;
  const bE = b ? { base: b.eLostBase, plan: b.eLostPlanned } : { base: AEP / 365 * 35, plan: AEP / 365 * 1.5 * 0.25 };
  const bEvent = 320000 - 45000 + (bE.base - bE.plan) * p;
  if (b) sim += bEvent;
  rows.push({ k: 'Predictive maintenance: gearbox bearing', did: b ? `Vibration warning ${b.detectedDaysBeforeFailure.toFixed(0)} days before failure; repair planned in a low-wind window (${fmt(bE.plan, 0)} MWh lost instead of ${fmt(bE.base, 0)} MWh and a €320k gearbox exchange).` : 'Detect early, repair up-tower for about €45k in 1.5 days instead of a €320k gearbox exchange and 35 days of downtime.',
    unit: `${eur(bEvent)} per avoided failure`, fleet: A.fleet * A.pf * bEvent, src: !!b });
  // yaw
  const y = V.yaw;
  const yLoss = y ? y.lossMWhYear : AEP * 0.0115, ySaved = y ? Math.max(0, y.monthsSaved) : 5.9;
  const yEvent = yLoss * ySaved / 12 * p;
  if (y) sim += yEvent;
  rows.push({ k: 'Energy recovery: yaw misalignment', did: y ? `8° offset found after ${(() => { const h = (twin.faults.yaw.detectedAt - twin.faults.yaw.start) * 24; return h < 48 ? h.toFixed(0) + ' h' : (h / 24).toFixed(1) + ' days'; })()} and estimated at ${y.est.toFixed(1)}°; ${fmt(yLoss, 0)} MWh/year loss stopped about ${ySaved.toFixed(1)} months earlier than the next annual service.` : `An 8° offset costs about ${fmt(AEP * 0.0115, 0)} MWh a year; the twin finds it in days rather than at the next annual service.`,
    unit: `${eur(yEvent)} per misaligned turbine`, fleet: A.fleet * A.yaw * yEvent, src: !!y });
  // icing
  const ic = V.icing;
  const perDay = ic ? (ic.eTwin - ic.eBase) / ic.days : AEP / 365 * 0.8 - 3;
  if (ic) sim += (ic.eTwin - ic.eBase) * p;
  rows.push({ k: 'Winter production: blade icing', did: ic ? `Blade heating instead of an ice stop: ${fmt(ic.eTwin, 0)} MWh produced during the cold spell against ${fmt(ic.eBase, 0)} MWh with a standard ice detector (heating used ${fmt(ic.eHeat, 1)} MWh).` : 'Keep producing with twin-controlled blade heating instead of stopping until the ice melts.',
    unit: `${eur(perDay * p)} per icing day`, fleet: A.fleet * A.ice * perDay * p, src: !!ic });
  // fatigue
  const ws = twin.wakeSummary(), rem = ws ? Math.max(3, ws.lifeYearsBase - twin.age) : 11;
  const ext = ws ? Math.max(0, ws.lifeYears - ws.lifeYearsBase) : 0.9, eLoss = ws ? ws.eLossYear : 280;
  const margin = AEP * p - 45000;
  const fYear = ext * margin * Math.pow(1.07, -rem) / rem - eLoss * p;
  if (ws && ws.days > 10) sim += fYear * ws.days / 365;
  rows.push({ k: 'Asset life: tower fatigue in a wake', did: ws ? `Load-aware control extends projected tower life by ${ext.toFixed(1)} years (to ${ws.lifeYears.toFixed(1)}) for ${fmt(eLoss, 0)} MWh/year of trimmed output.` : 'Trim thrust peaks only when turbulence is high, trading a little energy now for extra years of operation later.',
    unit: `${eur(fYear)} per turbine-year`, fleet: A.fleet * A.wake * fYear, src: !!(ws && ws.days > 10) });
  const benefit = rows.reduce((a, r) => a + r.fleet, 0), cost = A.fleet * A.cost;
  return { rows, benefit, cost, net: benefit - cost, ratio: benefit / cost, sim };
}
function renderValue() {
  const v = computeValue();
  $('#valueTable').innerHTML = `<thead><tr><th>Value lever</th><th>What the twin does</th><th class="n">Unit value</th><th class="n">Fleet per year</th></tr></thead><tbody>` +
    v.rows.map(r => `<tr><td><b>${r.k}</b><br><span class="src ${r.src ? 'sim' : 'est'}">${r.src ? 'from this simulation' : 'default estimate'}</span></td><td>${r.did}</td><td class="n">${r.unit}</td><td class="n"><b>${eur(r.fleet)}</b></td></tr>`).join('') + '</tbody>';
  $('#totals').innerHTML = [
    ['Fleet benefit per year', eur(v.benefit), 'good'], ['Ecosystem cost per year', eur(v.cost), ''], ['Net value per year', eur(v.net), v.net > 0 ? 'good' : 'alert'], ['Benefit per € spent', `${v.ratio.toFixed(1)} ×`, ''],
  ].map(([l, x, c]) => `<div class="ro ${c}"><div class="l">${l}</div><div class="v">${x}</div></div>`).join('');
}
renderValue();

// --------------------------------------------------------------- main loop
let last = performance.now(), uiT = 0, chartT = 0, specT = 0, valT = 0;
function circLerp(a, b, k) { let d = ((b - a + 540) % 360) - 180; return (a + d * k + 360) % 360; }
function frame(now) {
  const dt = Math.max(0.001, Math.min(0.1, (now - last) / 1000)); last = Math.max(last, now);
  if (playing) {
    acc += speed * dt; const n = Math.min(60, Math.floor(acc)); acc -= n;
    if (n > 0) {
      const ev = twin.advance(n);
      if (ev.length) {
        renderLog();
        for (const e of ev) { if (e.type === 'alert') ecoFlash.ai = performance.now(); if (e.type === 'action') ecoFlash.act = performance.now(); }
        if ($('#slowChk').checked && ev.some(e => e.type === 'alert')) setSpeed(6);
        renderValue();
      }
    }
  }
  const L = twin.last, F = twin.faults;
  // live dynamics follow the current 10-minute mean
  Uf += (L.U - Uf) * (1 - Math.exp(-dt / 2.5)); dirF = circLerp(dirF, L.dir, 1 - Math.exp(-dt / 3));
  dyn.Umean = Uf; dyn.TI = L.TI; dyn.rho = L.rho;
  const yawOff = F.yaw.on && !F.yaw.corrected ? F.yaw.offset : 0;
  dyn.yawDeg = yawOff; dyn.cpf = 1 - 0.38 * L.ice; dyn.derate = L.derate; dyn.imbalance = 0.35 * L.ice; dyn.parked = !!L.stopped;
  const steps = Math.max(1, Math.ceil(dt / 0.02));
  for (let i = 0; i < steps; i++) dyn.step(dt / steps);
  const out = dyn.out;
  const icingNow = F.icing.on && twin.day < F.icing.until + 0.3;
  scene.update(dt, {
    windDir: dirF, yawErr: yawOff, yawErrTrue: yawOff, azimuth: dyn.azimuth, pitch: out.beta, towerX: out.x, thrust: out.thrust,
    windSpeed: out.U, induction: out.stop ? 0 : out.a, ti: L.TI, ice: L.ice, heating: L.heating, snow: icingNow ? 1 : 0,
    bearingT: L.Tb, bearingResid: L.ewmaT, genRpm: out.genRpm, rpm: out.rpm, power: out.P, load: out.P / 5e6,
    wakeOn: F.wake.on, inWake: L.inWake,
  });
  scene.render();

  uiT += dt; chartT += dt; specT += dt; valT += dt;
  if (uiT > 0.2) {
    uiT = 0;
    const d = twin.day;
    $('#clockDay').textContent = `Day ${Math.floor(d)}`; $('#clockTime').textContent = fmtDay(d).split(' ')[2];
    $('#regionTxt').textContent = L.stopped ? 'stopped: maintenance' : out.stop ? (L.U < 3 ? 'waiting for wind' : 'storm stop') : (L.derate < 1 ? 'load-aware control' : out.beta > 0.5 ? 'full load, pitching' : 'partial load, tracking optimum');
    ro('wind', `${out.U.toFixed(1)}<small>m/s</small>`, `10-min ${L.U.toFixed(1)} · TI ${(L.TI * 100).toFixed(0)} % · ${Math.round(L.dir)}°`, L.inWake ? 'warn' : '');
    const pr = L.Pexp > 50 ? L.P / L.Pexp : 1;
    ro('power', `${(out.P / 1e6).toFixed(2)}<small>MW</small>`, `10-min ${(L.P / 1000).toFixed(2)} · twin ${(L.Pexp / 1000).toFixed(2)} MW`, pr < 0.9 && L.Pexp > 300 ? 'alert' : '');
    ro('rotor', `${out.rpm.toFixed(1)}<small>rpm</small>`, `pitch ${out.beta.toFixed(1)}° · tip-speed ratio ${out.lam.toFixed(1)}`);
    ro('thrust', `${fmt(out.thrust / 1000)}<small>kN</small>`, `tower-top sway ${(out.x * 100).toFixed(0)} cm`);
    const res = L.ewmaT;
    ro('bearing', `${L.Tb.toFixed(1)}<small>°C</small>`, `twin ${L.TbExp.toFixed(1)} °C · ${res >= 0 ? '+' : ''}${res.toFixed(1)}`, res > 3 ? 'alert' : res > 1.5 ? 'warn' : '');
    const vr = L.vib / L.vibExp;
    ro('vib', `${L.vib.toFixed(3)}<small>g</small>`, `${vr.toFixed(1)} × twin baseline`, vr > 2.2 ? 'alert' : vr > 1.6 ? 'warn' : '');
    ro('air', `${L.Tamb.toFixed(1)}<small>°C</small>`, `ρ ${L.rho.toFixed(3)} kg/m³ · RH ${Math.round(L.RH * 100)} %${L.ice > 0.03 ? ` · <span style="color:var(--ice)">ice ${(L.ice * 100).toFixed(0)} %</span>` : ''}`, L.ice > 0.1 ? 'warn' : '');
    const { rate } = lifeRates();
    ro('life', `${(twin.life.used * 100).toFixed(1)}<small>% used</small>`, `rate ${(rate * 20).toFixed(2)} × design`, rate * 20 > 1.2 ? 'warn' : '');
    // scenario cards
    const done = { bearing: F.bearing.stage >= 2, yaw: F.yaw.corrected, icing: !!twin.value.icing, wake: F.wake.detectedAt != null };
    $$('.scen-card').forEach(c => { const k = c.dataset.scen; if (c.classList.contains('running') && done[k]) { c.classList.remove('running'); c.classList.add('done'); c.querySelector('button').textContent = 'Run again'; } });
    updateEco(out);
  }
  if (chartT > 0.35) { chartT = 0; drawPower(); drawResid(); drawLife(); }
  if (specT > 0.25) { specT = 0; drawSpec(out); }
  if (valT > 2) { valT = 0; renderValue(); }
  requestAnimationFrame(frame);
}
renderLog();
requestAnimationFrame(frame);
window.__twin = () => twin; window.__scene = scene; window.__refresh = () => { renderLog(); renderValue(); };
