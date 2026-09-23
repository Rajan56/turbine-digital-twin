// charts.js: small canvas chart helpers (no external libraries)

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export class Chart {
  constructor(canvas, opts = {}) {
    this.c = canvas; this.ctx = canvas.getContext('2d'); this.o = opts;
    this.pad = Object.assign({ l: 46, r: 12, t: 10, b: 28 }, opts.pad || {});
  }
  size() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = this.c.clientWidth, h = this.c.clientHeight;
    if (this.c.width !== Math.round(w * dpr) || this.c.height !== Math.round(h * dpr)) { this.c.width = Math.round(w * dpr); this.c.height = Math.round(h * dpr); }
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = w; this.h = h; return this;
  }
  frame({ x0, x1, y0, y1, xTicks, yTicks, xFmt = (v) => v, yFmt = (v) => v, xLabel, yLabel, logX = false }) {
    const { ctx, pad } = this.size();
    this.x0 = x0; this.x1 = x1; this.y0 = y0; this.y1 = y1; this.logX = logX;
    ctx.clearRect(0, 0, this.w, this.h);
    ctx.font = '11px system-ui, -apple-system, Segoe UI, sans-serif';
    ctx.strokeStyle = css('--grid'); ctx.fillStyle = css('--muted'); ctx.lineWidth = 1;
    for (const v of yTicks) {
      const y = Math.round(this.Y(v)) + 0.5; ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(this.w - pad.r, y); ctx.stroke();
      ctx.textAlign = 'right'; ctx.textBaseline = 'middle'; ctx.fillText(yFmt(v), pad.l - 6, y);
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (const v of xTicks) { const x = this.X(v); ctx.fillText(xFmt(v), x, this.h - pad.b + 6); }
    if (xLabel) { ctx.textAlign = 'right'; ctx.fillText(xLabel, this.w - pad.r, this.h - 13); }
    if (yLabel) { ctx.textAlign = 'left'; ctx.textBaseline = 'top'; ctx.fillText(yLabel, 4, 0); }
    ctx.strokeStyle = css('--axis'); ctx.beginPath(); ctx.moveTo(pad.l, this.h - pad.b + 0.5); ctx.lineTo(this.w - pad.r, this.h - pad.b + 0.5); ctx.stroke();
    return this;
  }
  X(v) {
    const { pad } = this;
    const f = this.logX ? (Math.log10(v) - Math.log10(this.x0)) / (Math.log10(this.x1) - Math.log10(this.x0)) : (v - this.x0) / (this.x1 - this.x0);
    return pad.l + f * (this.w - pad.l - pad.r);
  }
  Y(v) { const { pad } = this; return this.h - pad.b - (v - this.y0) / (this.y1 - this.y0) * (this.h - pad.t - pad.b); }
  line(pts, color, width = 2, dash = null) {
    const { ctx } = this; ctx.save(); ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = 'round';
    if (dash) ctx.setLineDash(dash);
    let started = false;
    for (const [x, y] of pts) { if (y == null || !isFinite(y)) { started = false; continue; } const px = this.X(x), py = this.Y(Math.max(this.y0, Math.min(this.y1, y))); started ? ctx.lineTo(px, py) : ctx.moveTo(px, py); started = true; }
    ctx.stroke(); ctx.restore();
  }
  area(pts, color) {
    const { ctx } = this; if (!pts.length) return; ctx.save(); ctx.beginPath(); ctx.fillStyle = color;
    ctx.moveTo(this.X(pts[0][0]), this.Y(this.y0));
    for (const [x, y] of pts) ctx.lineTo(this.X(x), this.Y(Math.max(this.y0, Math.min(this.y1, y))));
    ctx.lineTo(this.X(pts[pts.length - 1][0]), this.Y(this.y0)); ctx.closePath(); ctx.fill(); ctx.restore();
  }
  dots(pts, color, r = 1.8, alphaFn = null) {
    const { ctx } = this; ctx.save(); ctx.fillStyle = color;
    pts.forEach((p, i) => { if (p[1] == null) return; ctx.globalAlpha = alphaFn ? alphaFn(i, pts.length) : 0.7; ctx.beginPath(); ctx.arc(this.X(p[0]), this.Y(p[1]), r, 0, 6.2832); ctx.fill(); });
    ctx.restore();
  }
  hline(v, color, label, dash = [4, 4]) {
    const { ctx } = this; const y = this.Y(v); ctx.save(); ctx.strokeStyle = color; ctx.setLineDash(dash); ctx.beginPath(); ctx.moveTo(this.pad.l, y); ctx.lineTo(this.w - this.pad.r, y); ctx.stroke();
    if (label) { ctx.setLineDash([]); ctx.fillStyle = color; ctx.font = '11px system-ui, sans-serif'; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom'; ctx.fillText(label, this.w - this.pad.r - 2, y - 2); }
    ctx.restore();
  }
  vline(x, color, label, dash = [3, 3]) {
    const { ctx } = this; const px = this.X(x); ctx.save(); ctx.strokeStyle = color; ctx.setLineDash(dash); ctx.beginPath(); ctx.moveTo(px, this.pad.t); ctx.lineTo(px, this.h - this.pad.b); ctx.stroke();
    if (label) { ctx.setLineDash([]); ctx.fillStyle = color; ctx.font = '11px system-ui, sans-serif'; ctx.textAlign = 'left'; ctx.textBaseline = 'top'; ctx.fillText(label, px + 3, this.pad.t); }
    ctx.restore();
  }
  text(x, y, s, color, align = 'left') { const { ctx } = this; ctx.save(); ctx.fillStyle = color; ctx.font = '600 11px system-ui, sans-serif'; ctx.textAlign = align; ctx.textBaseline = 'middle'; ctx.fillText(s, this.X(x), this.Y(y)); ctx.restore(); }
}

// radix-2 FFT magnitude (real input)
export function fftMag(sig) {
  const n = sig.length, re = Float64Array.from(sig), im = new Float64Array(n);
  for (let i = 1, j = 0; i < n; i++) { let bit = n >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) { [re[i], re[j]] = [re[j], re[i]]; } }
  for (let len = 2; len <= n; len <<= 1) {
    const ang = -2 * Math.PI / len, wr = Math.cos(ang), wi = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let cr = 1, ci = 0;
      for (let k = 0; k < len / 2; k++) {
        const ar = re[i + k], ai = im[i + k], br = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci, bi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
        re[i + k] = ar + br; im[i + k] = ai + bi; re[i + k + len / 2] = ar - br; im[i + k + len / 2] = ai - bi;
        const t = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = t;
      }
    }
  }
  const out = new Float64Array(n / 2);
  for (let i = 0; i < n / 2; i++) out[i] = 2 * Math.hypot(re[i], im[i]) / n;
  return out;
}
