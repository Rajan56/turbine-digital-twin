// twin.js
// Operational layer of the digital twin: 10-minute SCADA records from the "physical" turbine
// (simulated truth, with faults and sensor noise) compared with what the physics twin expects.
// Residuals drive detection, diagnosis, remaining-life estimates and actions, and every action
// is priced against a counterfactual in which the twin does not exist.

import { T5, steady, rhoFromT, aep } from './physics.js';

export const REC_PER_DAY = 144;
const DT_DAY = 1 / REC_PER_DAY;

// ------------------------------------------------------------------ small helpers
export function mulberry32(seed) {
  return function () {
    let t = (seed += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gauss(r) { return Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r()); }
function normCdf(z) { // Abramowitz-Stegun
  const t = 1 / (1 + 0.2316419 * Math.abs(z));
  const d = 0.3989423 * Math.exp(-z * z / 2);
  const p = d * t * (0.3193815 + t * (-0.3565638 + t * (1.781478 + t * (-1.821256 + t * 1.330274))));
  return z > 0 ? 1 - p : p;
}
function gammaFn(z) {
  const g = 7, p = [0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313,
    -176.61502916214059, 12.507343278686905, -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7];
  z -= 1; let x = p[0]; for (let i = 1; i < g + 2; i++) x += p[i] / (z + i);
  const t = z + g + 0.5; return Math.sqrt(2 * Math.PI) * Math.pow(t, z + 0.5) * Math.exp(-t) * x;
}
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
export const fmtDay = (d) => `Day ${Math.floor(d)} ${String(Math.floor((d % 1) * 24)).padStart(2, '0')}:${String(Math.floor(((d * 24) % 1) * 6) * 10).padStart(2, '0')}`;

// ---------------------------------------------------- steady-state lookup (fast)
// The twin evaluates the physics model thousands of times per simulated day, so the
// steady-state solution is tabulated on a (wind speed, yaw, efficiency, derate) grid.
const cache = new Map();
function ss(U, rho, yaw, cpf, derate) {
  const Uq = Math.round(U * 20) / 20, yq = Math.round(yaw), cq = Math.round(cpf * 100) / 100, dq = Math.round(derate * 100) / 100;
  const key = `${Uq}|${yq}|${cq}|${dq}`;
  let s = cache.get(key);
  if (!s) { s = steady(Uq, { rho: 1.225, yawDeg: yq, cpf: cq, derate: dq }); cache.set(key, s); }
  // density correction (power ~ rho below rated)
  const f = rho / 1.225;
  const P = Math.min(s.P * f, T5.Prated * derate);
  return { ...s, P, thrust: s.thrust * f };
}

// ------------------------------------------------------------------- weather
// Pre-generated for a year so that the twin can use a (noisy) forecast of the future.
export function makeWeather(days, seed, meanU = 7.5, k = 2.1, baseTemp = 6) {
  const r = mulberry32(seed);
  const n = days * REC_PER_DAY;
  const W = { U: new Float32Array(n), dir: new Float32Array(n), T: new Float32Array(n), RH: new Float32Array(n), TI: new Float32Array(n) };
  const A = meanU / gammaFn(1 + 1 / k);
  const phi = Math.exp(-DT_DAY / 0.55); // ~13 h correlation of 10-min means
  let z = 0, zt = 0, dir = 225, zh = 0;
  const phiT = Math.exp(-DT_DAY / 2.5);
  for (let i = 0; i < n; i++) {
    const day = i * DT_DAY;
    z = phi * z + Math.sqrt(1 - phi * phi) * gauss(r);
    const diurnal = 1 + 0.06 * Math.sin(2 * Math.PI * (day - 0.3));
    const Pz = clamp(normCdf(z), 1e-6, 1 - 1e-6);
    W.U[i] = Math.max(0, A * Math.pow(-Math.log(1 - Pz), 1 / k) * diurnal);
    dir += 0.08 * (225 - dir) * DT_DAY * 24 + gauss(r) * 2.5;
    W.dir[i] = ((dir % 360) + 360) % 360;
    zt = phiT * zt + Math.sqrt(1 - phiT * phiT) * gauss(r);
    W.T[i] = baseTemp + 3.5 * Math.sin(2 * Math.PI * (day - 0.375)) + 3 * zt;
    zh = 0.97 * zh + 0.24 * gauss(r);
    W.RH[i] = clamp(0.78 + 0.1 * zh - 0.05 * Math.sin(2 * Math.PI * (day - 0.375)), 0.4, 1);
    const U = Math.max(W.U[i], 1);
    W.TI[i] = clamp(0.075 + 0.35 / U + 0.012 * gauss(r), 0.04, 0.35);
  }
  return W;
}

// ------------------------------------------------------------- fatigue model
// Tower-base damage-equivalent load per 10-minute record from the thrust sensitivity and turbulence.
const WOHLER_M = 4, H = T5.hubH;
// quasi-steady thrust response to turbulence: std of thrust over U ± sigma_u (Gauss-Hermite)
const GHX = [-2.020183, -0.958572, 0, 0.958572, 2.020183], GHW = [0.019953, 0.393619, 0.945309, 0.393619, 0.019953].map(w => w / Math.sqrt(Math.PI));
function thrustStats(U, TI, yaw, cpf, derate) {
  const sig = TI * U; let m = 0, m2 = 0;
  for (let k = 0; k < 5; k++) { const u = Math.max(0, U + Math.SQRT2 * sig * GHX[k]); const T = ss(u, 1.225, yaw, cpf, derate).thrust; m += GHW[k] * T; m2 += GHW[k] * T * T; }
  return { mean: m, std: Math.sqrt(Math.max(0, m2 - m * m)) };
}
function towerDEL(U, TI, yaw, cpf, derate, imbalance) {
  const { mean, std } = thrustStats(U, TI, yaw, cpf, derate);
  return H * (1.5 * std + 0.06 * mean * TI + 25000 * imbalance);
}
// design damage per record chosen so that the design climate consumes 1/20 of life per year
let DAMAGE_NORM = 1;
(function calibrate() {
  const meanU = 7.5, k = 2.1, A = meanU / gammaFn(1 + 1 / k);
  let E = 0; const dU = 0.1;
  for (let U = dU / 2; U < 25; U += dU) {
    const f = (k / A) * Math.pow(U / A, k - 1) * Math.exp(-Math.pow(U / A, k));
    const TIdesign = 0.075 + 0.35 / Math.max(U, 1) + 0.02; // design includes normal wake exposure
    E += Math.pow(towerDEL(U, TIdesign, 0, 1, 1, 0), WOHLER_M) * f * dU;
  }
  DAMAGE_NORM = 1 / (20 * 365 * REC_PER_DAY * E);
})();

// ================================================================= the twin
export class TwinSystem {
  constructor({ seed = 7, days = 400, meanU = 7.5 } = {}) {
    this.seed = seed;
    this.W = makeWeather(days, seed, meanU);
    this.rng = mulberry32(seed * 31 + 5);
    this.i = 0;                              // record index
    this.price = 55;                         // EUR / MWh
    this.age = 9;                            // turbine age [years]
    this.lifeUsed0 = 0.43;                   // share of 20-year design fatigue life used at day 0
    this.life = { used: this.lifeUsed0, usedBase: this.lifeUsed0, rate7: [], };
    this.faults = {
      bearing: { on: false, onset: 0, s: 0, stage: 0, repairAt: null, repairUntil: null, rul: null, failBase: null },
      yaw: { on: false, offset: 8, corrected: false, detectedAt: null, fixAt: null, est: null, window: [] },
      icing: { on: false, until: 0, ice: 0, iceBase: 0, heating: false, detected: false, stopBase: false },
      wake: { on: false, control: false, detectedAt: null },
    };
    this.ewmaT = 0; this.tbMeas = null; this.tbExp = null; this.hiVib = 0; this.hiT = 0; this.iceCount = 0;
    this.events = [];
    this.hist = [];                          // recent records (for charts)
    this.HMAX = 30 * REC_PER_DAY;
    this.energy = { twin: 0, base: 0, heating: 0 };  // MWh since start, with twin vs without
    this.value = { bearing: null, yaw: null, icing: null, wake: null };
    this.counters = { records: 0, sensorValues: 0, alerts: 0, actions: 0 };
    this.stopped = null; this.last = null;
  }
  get day() { return this.i * DT_DAY; }

  log(type, title, detail) {
    const e = { day: this.day, type, title, detail };
    this.events.unshift(e); if (this.events.length > 60) this.events.pop();
    if (type === 'alert') this.counters.alerts++;
    if (type === 'action') this.counters.actions++;
    this._newEvents.push(e);
  }

  // ---------------------------------------------------------- scenario control
  inject(name, opts = {}) {
    const f = this.faults[name];
    if (name === 'bearing') {
      Object.assign(f, { on: true, onset: this.day, s: 0, stage: 0, repairAt: null, repairUntil: null, rul: null, failBase: null });
      // counterfactual failure date: severity grows exponentially; failure when s reaches s_fail
      f.tau = 12.5; f.sFail = 12;
      f.failBase = f.onset + f.tau * Math.log(1 + f.sFail);
      this.log('fault', 'Fault injected: high-speed-shaft bearing wear starts',
        'Outer-race damage begins. On the real asset nobody sees this yet; it grows slowly with load.');
    }
    if (name === 'yaw') {
      Object.assign(f, { on: true, offset: opts.offset ?? f.offset, corrected: false, detectedAt: null, fixAt: null, est: null, window: [] });
      f.start = this.day;
      this.log('fault', `Fault injected: wind-vane offset of ${f.offset}°`,
        'After a sensor replacement the vane is mounted slightly off, so the nacelle tracks the wrong direction.');
    }
    if (name === 'icing') {
      const from = this.i, to = Math.min(this.W.U.length, this.i + Math.round(4.5 * REC_PER_DAY));
      for (let j = from; j < to; j++) {
        const x = (j - from) / (to - from);
        const env = Math.min(1, x * 8, (1 - x) * 6);
        this.W.T[j] = this.W.T[j] * (1 - env) + (-4.5 + 2 * Math.sin(2 * Math.PI * j / REC_PER_DAY)) * env;
        this.W.RH[j] = this.W.RH[j] * (1 - env) + 0.975 * env;
        this.W.U[j] = Math.max(this.W.U[j], 5.5 + 3 * env);
      }
      Object.assign(f, { on: true, until: to * DT_DAY, detected: false, heating: false, ice: 0, iceBase: 0, eTwin: 0, eBase: 0, eHeat: 0 });
      this.log('fault', 'Weather: freezing fog and in-cloud icing for 4 days',
        'Temperature falls to about -5 °C with near-saturated air: ideal conditions for rime ice on the blades.');
    }
    if (name === 'wake') {
      Object.assign(f, { on: true, control: false, detectedAt: null, start: this.day, dmg: 0, dmgBase: 0, eLoss: 0 });
      this.log('fault', 'Site change: a new turbine is built upwind in the south-west sector',
        'Its wake raises turbulence at this turbine whenever the wind blows from 200° to 250°, the prevailing direction.');
    }
  }
  resetAll() {
    const keep = { price: this.price, seed: this.seed };
    Object.assign(this, new TwinSystem({ seed: keep.seed }));
    this.price = keep.price;
  }

  // ------------------------------------------------------------- one record
  step() {
    if (this.i >= this.W.U.length - REC_PER_DAY * 40) { // extend weather if needed
      const more = makeWeather(200, this.seed + this.i);
      for (const key of Object.keys(this.W)) { const a = new Float32Array(this.W[key].length + more[key].length); a.set(this.W[key]); a.set(more[key], this.W[key].length); this.W[key] = a; }
    }
    const r = this.rng, i = this.i, day = this.day, W = this.W;
    const U = W.U[i], Tamb = W.T[i], RH = W.RH[i], dir = W.dir[i];
    const rho = rhoFromT(Tamb);
    const F = this.faults;
    const rec = { day, U, Tamb, RH, dir, rho };

    // ---- wake turbulence
    let TI = W.TI[i];
    const inWake = F.wake.on && dir > 200 && dir < 250;
    if (inWake) TI += 0.045 * Math.exp(-Math.pow((dir - 225) / 18, 2)) + 0.012;
    rec.TI = TI; rec.inWake = inWake;

    // ---- icing physics: accretion in sub-zero saturated air, natural melt above +0.5 °C
    const Fi = F.icing;
    let iceRate = 0;
    if (Tamb < -0.5 && RH > 0.93 && U > 3) iceRate = 0.018 * (U / 8) * (RH - 0.93) / 0.05;   // per record
    const melt = Tamb > 0.5 ? 0.02 * (Tamb - 0.5 + 0.5) : 0.0015;
    Fi.iceBase = clamp(Fi.iceBase + iceRate - melt, 0, 1);
    const heatOn = Fi.heating;
    Fi.ice = clamp(Fi.ice + iceRate * (heatOn ? 0.3 : 1) - melt - (heatOn ? 0.012 : 0), 0, 1);
    const cpfIce = 1 - 0.38 * Fi.ice, cpfIceBase = 1 - 0.38 * Fi.iceBase;
    const imbalance = 0.35 * Fi.ice;

    // ---- yaw
    const Fy = F.yaw;
    const yawErr = (Fy.on && !Fy.corrected ? Fy.offset : 0) + 1.5 * gauss(r); // tracking error around offset
    rec.yawErr = yawErr;

    // ---- load-aware control (twin action for wake)
    let derate = 1;
    const Fw = F.wake;
    if (Fw.on && Fw.control && TI > 0.14 && U > 8.5 && U < 15.5) derate = 0.85;
    rec.derate = derate;

    // ---- bearing degradation
    const Fb = F.bearing;
    if (Fb.on) Fb.s = Math.exp((day - Fb.onset) / Fb.tau) - 1;
    const repairing = Fb.repairAt != null && day >= Fb.repairAt && day < Fb.repairUntil;
    if (Fb.repairUntil != null && day >= Fb.repairUntil && Fb.on) {
      Fb.on = false; Fb.s = 0; this.log('ok', 'Repair complete: HSS bearing replaced up-tower',
        'Turbine back in service. Temperature and vibration return to the twin baseline.');
    }

    // ---- stop logic
    const iceStopTwin = false; // with the twin, blade heating keeps the turbine running
    const stopped = repairing;
    rec.stopped = stopped ? 'planned maintenance' : null;

    // ---- the physical turbine (truth)
    const truth = stopped ? { P: 0, thrust: ss(U, rho, 0, 1, 1).thrust * 0.05, rpm: 0, beta: 90 } : ss(U, rho, Math.abs(yawErr), cpfIce, derate);
    const heatKW = heatOn ? 125 : 0;
    rec.Ptrue = truth.P / 1000;
    // ---- sensors (10-minute averages)
    const Unac = U * (1 + 0.018 * gauss(r));
    const Pmeas = Math.max(0, rec.Ptrue * (1 + 0.03 * gauss(r)) + 8 * gauss(r));
    rec.Unac = Unac; rec.P = Pmeas; rec.rpm = truth.rpm || 0; rec.pitch = truth.beta || 0; rec.thrust = truth.thrust;
    // ---- the twin's expectation from measured inputs (assumes a healthy, aligned, clean rotor)
    const exp = ss(Unac, rho, 0, 1, derate);
    rec.Pexp = stopped ? 0 : exp.P / 1000;
    // ---- drivetrain temperature (first-order thermal lag, 40 min)
    const load = rec.Ptrue / 5000, loadExp = rec.Pexp / 5000;
    const Tnac = Tamb + 12;
    const tbSS = Tnac + 20 + 30 * Math.pow(load, 1.2) + (Fb.on ? 1.55 * Fb.s * (0.35 + 0.65 * load) : 0);
    const tbSSexp = Tnac + 20 + 30 * Math.pow(Math.max(loadExp, load * 0.98), 1.2);
    const aT = 1 - Math.exp(-10 / 40);
    this.tbMeas = this.tbMeas == null ? tbSS : this.tbMeas + aT * (tbSS - this.tbMeas);
    this.tbExp = this.tbExp == null ? tbSSexp : this.tbExp + aT * (tbSSexp - this.tbExp);
    rec.Tb = this.tbMeas + 0.45 * gauss(r); rec.TbExp = this.tbExp;
    // ---- vibration (condition monitoring): BPFO envelope amplitude [g]
    const vibBase = 0.03 + 0.02 * load;
    rec.vib = stopped ? 0.005 : (vibBase + (Fb.on ? 0.07 * Fb.s : 0)) * Math.exp(0.12 * gauss(r));
    rec.vibExp = stopped ? 0.005 : 0.03 + 0.02 * loadExp;
    rec.oneP = (0.02 + imbalance * 0.5) * (rec.rpm / 12.1) ** 2;
    rec.ice = Fi.ice; rec.heating = heatOn;

    // ---- fatigue
    const del = towerDEL(U, TI, Math.abs(yawErr), cpfIce, derate, imbalance);
    const dmg = Math.pow(del, WOHLER_M) * DAMAGE_NORM;
    const delBase = towerDEL(U, TI, Math.abs(yawErr), cpfIce, 1, imbalance);
    const dmgBase = Math.pow(delBase, WOHLER_M) * DAMAGE_NORM;
    this.life.used += dmg; this.life.usedBase += dmgBase;
    this.life.rate7.push(dmg); if (this.life.rate7.length > 7 * REC_PER_DAY) this.life.rate7.shift();
    rec.dmg = dmg; rec.del = del;
    if (Fw.on) { Fw.dmg += dmg; Fw.dmgBase += dmgBase; if (derate < 1) Fw.eLoss += (ss(U, rho, Math.abs(yawErr), cpfIce, 1).P - truth.P) / 1e6 / 6; }

    // ---- energy accounting: with twin vs counterfactual without twin
    const e = truth.P / 1e6 / 6 - heatKW / 1000 / 6; // MWh in 10 minutes
    this.energy.twin += e;
    // counterfactual: standard ice detector stops the turbine while ice is on the blades
    let eBase;
    const baseIceStop = Fi.iceBase > 0.2;
    const baseYaw = (Fy.on ? Fy.offset : 0);
    const baseBearingDown = Fb.failBase != null && day >= Fb.failBase && day < Fb.failBase + 35;
    if (baseIceStop || baseBearingDown) eBase = 0;
    else eBase = ss(U, rho, Math.abs(baseYaw + (yawErr - (Fy.on && !Fy.corrected ? Fy.offset : 0))), cpfIceBase, 1).P / 1e6 / 6;
    this.energy.base += eBase;
    if (Fi.on) { Fi.eTwin += e; Fi.eBase += eBase; Fi.eHeat += heatKW / 1000 / 6; }

    this.counters.records++; this.counters.sensorValues += 38;
    this.i++;
    this.last = rec;
    this.hist.push(rec); if (this.hist.length > this.HMAX) this.hist.shift();
    this.detect(rec);
    return rec;
  }

  // ----------------------------------------------------- detection and actions
  detect(rec) {
    const F = this.faults, day = rec.day;
    // ---------- bearing: vibration first, then temperature, then RUL and planned repair
    const Fb = F.bearing;
    const resid = rec.Tb - rec.TbExp;
    this.ewmaT = this.ewmaT + 0.05 * (resid - this.ewmaT);
    rec.ewmaT = this.ewmaT;
    if (Fb.on && !rec.stopped) {
      this.hiVib = rec.vib > 2.2 * rec.vibExp ? this.hiVib + 1 : 0;
      if (Fb.stage === 0 && this.hiVib >= 12) {
        Fb.stage = 1; Fb.warnAt = day;
        this.log('alert', 'Early warning: bearing defect signature in vibration',
          `Envelope spectrum shows the outer-race frequency (BPFO, 5.1 × shaft speed) at ${(rec.vib / rec.vibExp).toFixed(1)} × the twin baseline. Temperature still looks normal.`);
        this.log('action', 'Order bearing kit and book technicians', 'Lead time about 10 days, so ordering now avoids waiting later.');
      }
      this.hiT = this.ewmaT > 3 ? this.hiT + 1 : 0;
      if (Fb.stage === 1 && this.hiT >= 36) {
        Fb.stage = 2; Fb.confirmAt = day;
        // remaining life: fit log(residual) ~ linear in time on recent history
        const pts = this.hist.filter(h => h.day > Fb.warnAt - 2 && h.ewmaT > 0.5).map(h => [h.day, Math.log(h.ewmaT)]);
        let sx = 0, sy = 0, sxx = 0, sxy = 0; for (const [x, y] of pts) { sx += x; sy += y; sxx += x * x; sxy += x * y; }
        const n = pts.length, b = (n * sxy - sx * sy) / (n * sxx - sx * sx), a = (sy - b * sx) / n;
        const tCrit = (Math.log(15) - a) / Math.max(b, 1e-3);
        Fb.rul = Math.max(1, tCrit - day);
        // choose the lowest-wind 2-day window between parts arrival and RUL minus margin (noisy forecast)
        const earliest = Math.max(day + 1.5, Fb.warnAt + 10), latest = Math.max(earliest + 0.5, day + Fb.rul - 3);
        let best = earliest, bestE = Infinity;
        for (let d = earliest; d <= latest; d += 0.25) {
          let E = 0; const i0 = Math.round(d * REC_PER_DAY);
          for (let j = i0; j < i0 + 1.5 * REC_PER_DAY; j++) E += ss(this.W.U[j] * (1 + 0.1 * Math.sin(j)), 1.225, 0, 1, 1).P;
          if (E < bestE) { bestE = E; best = d; }
        }
        Fb.repairAt = best; Fb.repairUntil = best + 1.5;
        Fb.plannedLoss = bestE / 1e6 / 6;
        this.log('alert', `Confirmed: bearing running ${this.ewmaT.toFixed(1)} °C hotter than the twin expects`,
          `Trend fitted on the residual predicts the +15 °C trip limit in about ${Fb.rul.toFixed(0)} days.`);
        this.log('action', `Repair planned for ${fmtDay(best)} (low-wind window)`,
          `36 h stop chosen from the wind forecast; expected production lost ${Fb.plannedLoss.toFixed(0)} MWh.`);
        this.valueBearing();
      }
    }
    // ---------- yaw: power-curve residual over a rolling 3-day window (partial-load bins)
    const Fy = F.yaw;
    if (!rec.stopped && rec.Unac > 5 && rec.Unac < 10 && rec.Tamb > 1 && rec.Pexp > 200) {
      Fy.window.push({ day, r: rec.P / rec.Pexp });
    }
    while (Fy.window.length && Fy.window[0].day < day - 3) Fy.window.shift();
    if (Fy.window.length > 60) {
      const n = Fy.window.length; let m = 0; for (const w of Fy.window) m += w.r; m /= n;
      let v = 0; for (const w of Fy.window) v += (w.r - m) ** 2; const se = Math.sqrt(v / (n - 1) / n);
      rec.powerRatio = m; rec.powerRatioSE = se;
      if (Fy.on && !Fy.corrected && Fy.detectedAt == null && 1 - m > 0.012 && (1 - m) / se > 5 && n > 150) {
        Fy.detectedAt = day; Fy.est = Math.acos(Math.sqrt(Math.min(1, m))) * 180 / Math.PI;
        this.log('alert', `Power deficit ${(100 * (1 - m)).toFixed(1)} % below the twin in partial load`,
          `Pattern matches yaw misalignment of about ${Fy.est.toFixed(0)}° (loss follows cos² of the misalignment angle). Not icing: air temperature is above zero.`);
        Fy.fixAt = day + 0.5;
        this.log('action', 'Remote correction of the yaw offset scheduled', 'Controller offset updated after checking against the nacelle LiDAR; verification over the next days.');
        this.valueYaw();
      }
    }
    if (Fy.fixAt != null && !Fy.corrected && day >= Fy.fixAt) { Fy.corrected = true; this.log('ok', 'Yaw offset corrected', 'Nacelle now tracks the true wind direction; the power ratio returns towards 1.00.'); }

    // ---------- icing: fast residual in sub-zero conditions
    const Fi = F.icing;
    const ratio = rec.Pexp > 300 ? rec.P / rec.Pexp : 1;
    if (Fi.on) {
      this.iceCount = (rec.Tamb < 1 && ratio < 0.85 && rec.U > 4) ? this.iceCount + 1 : 0;
      if (!Fi.heating && this.iceCount >= 3) {
        Fi.heating = true; Fi.detected = true;
        this.log('alert', `Icing: power ${(100 * (1 - ratio)).toFixed(0)} % below the twin at ${rec.Tamb.toFixed(1)} °C`,
          'Temperature, humidity and the power residual together point to rime ice on the leading edges. 1P vibration is rising from uneven ice.');
        this.log('action', 'Blade heating switched on by the twin', 'Uses about 125 kW while running. Without the twin the ice detector would simply stop the turbine until the weather warms.');
      }
      if (Fi.heating && rec.Tamb > 1 && Fi.ice < 0.02) {
        Fi.heating = false;
        this.log('ok', 'Icing over: blade heating off', 'Blades clean; power back on the twin curve.');
      }
      if (day > Fi.until + 0.5 && !Fi.heating && !Fi.closed) { Fi.closed = true; this.valueIcing(); }
    }

    // ---------- tower fatigue: consumption rate versus design
    const Fw = F.wake;
    if (this.life.rate7.length >= 3 * REC_PER_DAY) {
      const rate = this.life.rate7.reduce((a, b) => a + b, 0) / this.life.rate7.length * REC_PER_DAY * 365; // share of life per year
      rec.lifeRate = rate;
      if (Fw.on && !Fw.control && Fw.detectedAt == null && day - Fw.start > 4 && rate > 1.25 / 20) {
        Fw.detectedAt = day; Fw.control = true;
        this.log('alert', `Tower fatigue consumed ${(rate * 20).toFixed(2)} × faster than design`,
          'Strain-derived damage is concentrated in the 200° to 250° sector, where turbulence has risen.');
        this.log('action', 'Load-aware control enabled in the wake sector', 'The twin trims the thrust peak around rated wind (about 15 % less power in those hours) when turbulence is high.');
      }
    }
    rec.lifeUsed = this.life.used;
  }

  // ------------------------------------------------------------- value models
  valueBearing() {
    const Fb = this.faults.bearing, W = this.W;
    // counterfactual: run to failure, then 35 days waiting for crane and gearbox
    let eLost = 0; const i0 = Math.round(Fb.failBase * REC_PER_DAY);
    for (let j = i0; j < i0 + 35 * REC_PER_DAY; j++) eLost += ss(W.U[j], rhoFromT(W.T[j]), 0, 1, 1).P / 1e6 / 6;
    this.value.bearing = { detectedDaysBeforeFailure: Fb.failBase - Fb.warnAt, confirmBefore: Fb.failBase - Fb.confirmAt,
      eLostBase: eLost, eLostPlanned: Fb.plannedLoss, costBase: 320000, costPlanned: 45000 };
  }
  valueYaw() {
    const Fy = this.faults.yaw;
    const a0 = aep(), a1 = aep({ yawDeg: Fy.offset });
    this.value.yaw = { lossPct: (a0 - a1) / a0, lossMWhYear: a0 - a1, monthsSaved: 6 - (Fy.detectedAt - Fy.start) / 30, est: Fy.est };
  }
  valueIcing() {
    const Fi = this.faults.icing;
    this.value.icing = { eTwin: Fi.eTwin, eBase: Fi.eBase, eHeat: Fi.eHeat, days: 4.5 };
  }
  wakeSummary() {
    const Fw = this.faults.wake; if (!Fw.on) return null;
    const days = this.day - Fw.start; if (days < 2) return null;
    const rate = Fw.dmg / days * 365, rateBase = Fw.dmgBase / days * 365;
    const rem = 1 - this.life.used;
    return { rate, rateBase, lifeYears: this.age + rem / Math.max(rate, 1e-6), lifeYearsBase: this.age + (1 - this.life.usedBase) / Math.max(rateBase, 1e-6),
      eLossYear: Fw.eLoss / days * 365, days };
  }

  advance(n) {
    this._newEvents = [];
    for (let k = 0; k < n; k++) this.step();
    return this._newEvents;
  }
}
