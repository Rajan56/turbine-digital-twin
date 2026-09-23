// physics.js
// Physics model of the NREL 5-MW reference wind turbine (Jonkman et al., 2009),
// land-based configuration. Aerodynamics use Heier's empirical Cp(λ, β) surface,
// rescaled so that its optimum matches the NREL 5-MW rotor (Cp 0.482 at λ 7.55).
// Rotor, drivetrain, pitch control and tower fore-aft motion are integrated in time.

export const T5 = {
  R: 63,                 // rotor radius [m]
  hubR: 1.5,             // hub radius [m]
  hubH: 90,              // hub height [m]
  towerH: 87.6,          // tower height [m]
  towerDBase: 6.0, towerDTop: 3.87,
  precone: 2.5, tilt: 5, // [deg]
  Prated: 5.0e6,         // electrical [W]
  eta: 0.944,            // generator efficiency
  gear: 97,
  wMin: 6.9 * Math.PI / 30,     // [rad/s]
  wRated: 12.1 * Math.PI / 30,  // [rad/s]
  Ucutin: 3, Urated: 11.4, Ucutout: 25,
  J: 38759236 + 534.116 * 97 * 97, // rotor + generator inertia at LSS [kg m2]
  cpMax: 0.482, lamOpt: 7.55,
  // tower first fore-aft mode
  fTower: 0.324, mTop: 110000 + 240000, mTower: 347460, zetaTower: 0.01,
  // pitch controller (NREL baseline, converted to low-speed shaft)
  KP: 0.01882681 * 97, KI: 0.008068634 * 97, thetaK: 6.302336 * Math.PI / 180,
  pitchRate: 8 * Math.PI / 180, pitchMax: 90 * Math.PI / 180,
};
T5.A = Math.PI * T5.R * T5.R;
T5.Qrated = T5.Prated / T5.eta / T5.wRated;               // LSS rated torque [N m]
T5.mEff = T5.mTop + 0.23 * T5.mTower;                      // modal mass
T5.kTower = Math.pow(2 * Math.PI * T5.fTower, 2) * T5.mEff;
T5.cTower = 2 * T5.zetaTower * Math.sqrt(T5.kTower * T5.mEff);

// ---------------------------------------------------------------- aerodynamics
function cpHeier(lam, betaDeg) {
  const b = Math.max(0, betaDeg);
  const inv = 1 / (lam + 0.08 * b) - 0.035 / (b * b * b + 1);
  if (inv <= 0) return 0;
  const li = 1 / inv;
  const cp = 0.5176 * (116 / li - 0.4 * b - 5) * Math.exp(-21 / li) + 0.0068 * lam;
  return Math.max(0, cp);
}
// find the optimum of the raw surface once, then rescale to the NREL rotor
let _lamH = 8, _cpH = 0.48;
(function calibrate() {
  let best = 0, bl = 0;
  for (let l = 4; l <= 12; l += 0.001) { const c = cpHeier(l, 0); if (c > best) { best = c; bl = l; } }
  _lamH = bl; _cpH = best;
})();
const cpRaw = (lam, bH) => T5.cpMax / _cpH * cpHeier(lam * _lamH / T5.lamOpt, bH);
// Steady-state blade pitch of the NREL 5-MW rotor above rated wind (Jonkman et al., 2009, Table 7-1).
const NREL_PITCH = [[11.4, 0], [12, 3.83], [13, 6.60], [14, 8.70], [15, 10.45], [16, 12.06], [17, 13.54], [18, 14.92],
  [19, 16.23], [20, 17.47], [21, 18.70], [22, 19.94], [23, 21.18], [24, 22.35], [25, 23.47]];
// Map true pitch to the equivalent pitch of the Heier surface so that the model reproduces the NREL pitch schedule.
const PMAP = [[0, 0]];
(function buildPitchMap() {
  const q = (U) => 0.5 * 1.225 * T5.A * U * U * U;
  const wr = 12.1 * Math.PI / 30, Pa = 5.0e6 / 0.944;
  for (const [U, bN] of NREL_PITCH.slice(1)) {
    const lam = wr * T5.R / U, target = Pa / q(U);
    let lo = 0, hi = 60;
    for (let i = 0; i < 60; i++) { const m = (lo + hi) / 2; (cpRaw(lam, m) > target ? lo = m : hi = m); }
    PMAP.push([bN, (lo + hi) / 2]);
  }
})();
function mapPitch(b) {
  if (b <= 0) return 0;
  for (let i = 1; i < PMAP.length; i++) {
    if (b <= PMAP[i][0]) { const [x0, y0] = PMAP[i - 1], [x1, y1] = PMAP[i]; return y0 + (y1 - y0) * (b - x0) / (x1 - x0); }
  }
  const [x0, y0] = PMAP[PMAP.length - 2], [x1, y1] = PMAP[PMAP.length - 1];
  return y1 + (y1 - y0) / (x1 - x0) * (b - x1);
}
export function cp(lam, betaDeg) {
  return cpRaw(lam, mapPitch(betaDeg));
}
// axial induction from Cp (momentum theory with a constant loss factor), then thrust coefficient
const LOSS = T5.cpMax / (16 / 27);
export function ctFromCp(c) {
  const target = Math.min(16 / 27, c / LOSS);
  let lo = 0, hi = 1 / 3;
  for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; (4 * m * (1 - m) * (1 - m) < target ? lo = m : hi = m); }
  const a = (lo + hi) / 2;
  return { a, ct: 4 * a * (1 - a) };
}
export const rhoFromT = (tempC, pPa = 101325 * Math.exp(-90 / 8434)) => pPa / (287.05 * (tempC + 273.15));

// K for optimal-torque tracking in region 2 (LSS)
export const Kopt = (rho) => 0.5 * rho * Math.PI * Math.pow(T5.R, 5) * T5.cpMax / Math.pow(T5.lamOpt, 3);

// ------------------------------------------------------ steady-state operating point
// yawDeg: misalignment between rotor axis and wind; cpf: aerodynamic efficiency factor (icing/erosion)
// derate: fraction of rated power allowed (load-aware control)
export function steady(U, { rho = 1.225, yawDeg = 0, cpf = 1, derate = 1, yawExp = 2 } = {}) {
  const out = { U, w: 0, rpm: 0, beta: 0, lam: 0, cp: 0, P: 0, Paero: 0, thrust: 0, Q: 0, region: 'parked', a: 0 };
  const cy = Math.pow(Math.cos(yawDeg * Math.PI / 180), yawExp);
  if (U < T5.Ucutin || U >= T5.Ucutout) {
    // parked / idling: small drag thrust on feathered blades
    out.beta = 90; out.thrust = 0.5 * rho * T5.A * U * U * 0.05; out.region = U < T5.Ucutin ? 'below cut-in' : 'storm stop';
    return out;
  }
  const q = 0.5 * rho * T5.A * U * U * U;
  const Pcap = T5.Prated / T5.eta * derate;
  let w = Math.min(T5.wRated, Math.max(T5.wMin, T5.lamOpt * U / T5.R));
  let lam = w * T5.R / U;
  let c = cp(lam, 0) * cpf * cy;
  let beta = 0;
  let region = w <= T5.wMin + 1e-6 ? 'region 1.5' : (w >= T5.wRated - 1e-6 ? 'region 2.5' : 'region 2');
  if (q * c > Pcap) {
    // above rated: pitch to hold power
    w = T5.wRated; lam = w * T5.R / U;
    let lo = 0, hi = 40;
    for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; (q * cp(lam, m) * cpf * cy > Pcap ? lo = m : hi = m); }
    beta = (lo + hi) / 2; c = Pcap / q; region = derate < 1 && U < T5.Urated + 3 ? 'derated' : 'region 3';
  }
  const Paero = q * c;
  const { a, ct } = ctFromCp(c / Math.max(cy, 1e-3));
  out.w = w; out.rpm = w * 30 / Math.PI; out.beta = beta; out.lam = lam; out.cp = c; out.Paero = Paero;
  out.P = Paero * T5.eta; out.Q = Paero / w; out.a = a;
  out.thrust = 0.5 * rho * T5.A * U * U * ct * Math.pow(Math.cos(yawDeg * Math.PI / 180), 2);
  out.region = region;
  return out;
}

// power curve table (electrical kW) for charts / energy yield
export function powerCurve(opts = {}, Umax = 26, dU = 0.25) {
  const pts = [];
  for (let U = 0; U <= Umax + 1e-9; U += dU) pts.push([U, steady(U, opts).P / 1000, steady(U, opts)]);
  return pts;
}

// annual energy (MWh) for a Weibull wind climate at hub height
export function aep({ meanU = 7.5, k = 2.1, avail = 0.97, ...opts } = {}) {
  const gamma = (z) => { // Lanczos approximation
    const g = 7, p = [0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313,
      -176.61502916214059, 12.507343278686905, -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7];
    if (z < 0.5) return Math.PI / (Math.sin(Math.PI * z) * gamma(1 - z));
    z -= 1; let x = p[0]; for (let i = 1; i < g + 2; i++) x += p[i] / (z + i);
    const t = z + g + 0.5; return Math.sqrt(2 * Math.PI) * Math.pow(t, z + 0.5) * Math.exp(-t) * x;
  };
  const A = meanU / gamma(1 + 1 / k);
  let E = 0; const dU = 0.1;
  for (let U = dU / 2; U < 30; U += dU) {
    const f = (k / A) * Math.pow(U / A, k - 1) * Math.exp(-Math.pow(U / A, k));
    E += steady(U, opts).P * f * dU;
  }
  return E * 8760 * avail / 1e6; // MWh
}

// ------------------------------------------------------------- time-domain model
// Rotor speed, pitch, generator torque, tower fore-aft motion; turbulent wind.
export class Dynamics {
  constructor() {
    this.w = T5.wMin; this.beta = 0; this.betaCmd = 0; this.integ = 0;
    this.x = 0; this.v = 0;             // tower-top displacement / velocity
    this.uTurb = 0;                      // turbulent component (rotor-effective)
    this.Umean = 8; this.TI = 0.12;
    this.rho = 1.225; this.yawDeg = 0; this.cpf = 1; this.derate = 1; this.imbalance = 0;
    this.azimuth = 0; this.t = 0;
    this.out = {};
    this.Qgen = 0;
    this.parked = false;
  }
  // one integration step
  step(dt, rnd = Math.random) {
    const L = 340.2; // Kaimal length scale (IEC 61400-1, hub height > 60 m)
    const U0 = Math.max(0.1, this.Umean);
    const tau = L / U0, sig = this.TI * U0;
    // Ornstein-Uhlenbeck process; rotor averaging reduces variance of the effective wind
    const n = Math.sqrt(-2 * Math.log(rnd() + 1e-12)) * Math.cos(2 * Math.PI * rnd());
    if (!(dt > 0)) return this.out;
    this.uTurb += (-this.uTurb / tau) * dt + sig * 0.85 * Math.sqrt(2 * dt / tau) * n;
    const U = Math.max(0, U0 + this.uTurb);
    const Urel = Math.max(0, U - this.v);          // aerodynamic damping from tower motion
    const stop = this.Umean < T5.Ucutin - 0.3 || this.Umean >= T5.Ucutout || this.parked;
    const cy = Math.pow(Math.cos(this.yawDeg * Math.PI / 180), 2);
    const lam = this.w * T5.R / Math.max(Urel, 0.5);
    const betaDeg = this.beta * 180 / Math.PI;
    const c = cp(lam, betaDeg) * this.cpf * cy;
    const Paero = 0.5 * this.rho * T5.A * Math.pow(Urel, 3) * c;
    const Qaero = Paero / Math.max(this.w, 0.05);
    // generator torque (LSS): region 1.5 ramp, region 2 K w^2, region 3 constant power
    let Qg;
    const K = Kopt(this.rho);
    const Pcap = T5.Prated / T5.eta * this.derate;
    const w = this.w;
    if (stop) Qg = 0;
    else if (w >= T5.wRated * 0.99 || this.beta > 0.01) Qg = Math.min(Pcap / Math.max(w, 0.1), 1.1 * T5.Qrated);
    else {
      const ramp = Math.min(1, Math.max(0, (w - 0.9 * T5.wMin) / (0.1 * T5.wMin)));
      Qg = Math.min(K * w * w * ramp, Pcap / Math.max(w, 0.1));
    }
    // generator torque rate limit (15 kN m/s at HSS)
    const dQmax = 15000 * T5.gear * dt;
    Qg = Math.max(this.Qgen - dQmax, Math.min(this.Qgen + dQmax, Qg));
    this.Qgen = Qg;
    // rotor
    this.w += (Qaero - Qg) / T5.J * dt;
    this.w = Math.max(0, this.w);
    // pitch controller (gain-scheduled PI on rotor speed, NREL baseline)
    const wRef = T5.wRated * (this.derate < 1 ? Math.max(0.93, Math.sqrt(this.derate)) : 1);
    if (stop) { this.betaCmd = T5.pitchMax; this.integ = T5.pitchMax / T5.KI; }
    else {
      const e = this.w - wRef;
      const GK = 1 / (1 + this.beta / T5.thetaK);
      this.integ = Math.max(0, Math.min(T5.pitchMax / (GK * T5.KI), this.integ + e * dt));
      this.betaCmd = Math.max(0, Math.min(T5.pitchMax, GK * (T5.KP * e + T5.KI * this.integ)));
    }
    const dB = Math.max(-T5.pitchRate * dt, Math.min(T5.pitchRate * dt, this.betaCmd - this.beta));
    this.beta += dB;
    // thrust and tower
    const { a, ct } = ctFromCp(c / Math.max(cy, 1e-3));
    let thrust = stop ? 0.5 * this.rho * T5.A * Urel * Urel * 0.05 : 0.5 * this.rho * T5.A * Urel * Urel * ct * cy;
    // 3P tower-shadow modulation and 1P mass/aerodynamic imbalance (e.g. uneven ice)
    thrust *= 1 - 0.015 * Math.cos(3 * this.azimuth);
    thrust += this.imbalance * 60000 * Math.pow(this.w / T5.wRated, 2) * Math.sin(this.azimuth);
    const acc = (thrust - T5.cTower * this.v - T5.kTower * this.x) / T5.mEff;
    this.v += acc * dt; this.x += this.v * dt;
    this.azimuth = (this.azimuth + this.w * dt) % (2 * Math.PI);
    this.t += dt;
    const Pel = Qg * this.w * T5.eta;
    this.out = { U, Urel, lam, cp: c, beta: this.beta * 180 / Math.PI, rpm: this.w * 30 / Math.PI, genRpm: this.w * 30 / Math.PI * T5.gear,
      P: Pel, Paero, thrust, x: this.x, acc, a, Qaero, Qg, towerBaseM: thrust * T5.hubH + T5.mTop * 9.81 * this.x, stop };
    return this.out;
  }
}
