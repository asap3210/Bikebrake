'use strict';
// Prueba: el soporte pivota; se reajusta la inclinación sola con la bici quieta y horizontal.
const { Detector, G } = require('./detector.js');
function rng(seed) { return function () { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
function gauss(r) { return Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r()); }
const mm = (A, B) => A.map((r, i) => [0, 1, 2].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
const mul = (R, v) => [0, 1, 2].map(i => R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2]);
const deg = d => d * Math.PI / 180;
const Rx = a => [[1, 0, 0], [0, Math.cos(a), -Math.sin(a)], [0, Math.sin(a), Math.cos(a)]];
const Ry = a => [[Math.cos(a), 0, Math.sin(a)], [0, 1, 0], [-Math.sin(a), 0, Math.cos(a)]];
const Rz = a => [[Math.cos(a), -Math.sin(a), 0], [Math.sin(a), Math.cos(a), 0], [0, 0, 1]];
const R0 = mm(mm(Rz(deg(35)), Ry(deg(55))), Rx(deg(-20)));

// cuerpo: x adelante, y lateral, z arriba. R: cuerpo -> móvil.
class Sim {
  constructor(R, seed) { this.R = R; this.rnd = rng(seed); this.t = 0; this.v = 0; }
  run(det, dur, F, opts = {}) {
    const dt = 1 / 60, n = Math.round(dur * 60), seen = new Set(); let first = null;
    for (let i = 0; i < n; i++) {
      const sv = this.v > 0.3 ? Math.min(1, this.v / 3) : 0;
      const f = F + (this.v > 1 ? (opts.osc || 0) * Math.sin(2 * Math.PI * 2.4 * this.t) : 0);
      let a = f; if (this.v <= 0 && a < 0) { this.v = 0; a = 0; } else this.v = Math.max(0, this.v + a * dt);
      const mb = [f + gauss(this.rnd) * (0.03 + 0.5 * sv), gauss(this.rnd) * (0.03 + 0.5 * sv), G + gauss(this.rnd) * (0.03 + 1.0 * sv)];
      const m = mul(this.R, mb);
      if (i % 60 === 0) det.setGpsSpeed(this.t, Math.max(0, this.v));
      const s = det.update(this.t, m[0], m[1], m[2]);
      seen.add(s); if (s !== 0 && first === null) first = i * dt;
      this.t += dt;
    }
    return { seen: [...seen], first };
  }
}

let fails = 0;
const check = (name, ok, extra) => { if (!ok) fails++; console.log((ok ? 'OK    ' : 'FALLO ') + name + (extra ? '  ' + extra : '')); };

// calibración completa con R0
function calibrated() {
  const det = new Detector({ sensitivity: 1.0 }); det.startCalibration();
  const sim = new Sim(R0, 7);
  sim.run(det, 3, 0); sim.run(det, 4, 1.5, { osc: 0.8 });
  if (det.calibState !== 'done') throw new Error('no calibró: ' + det.calibState);
  return { det, saved: JSON.parse(JSON.stringify(det.calib)) };
}

for (const alpha of [0, 15, -20, 30]) {
  const { saved } = calibrated();
  const det = new Detector({ sensitivity: 1.0 }); det.setCalibration(saved);
  const R = mm(R0, Ry(deg(alpha)));                 // el soporte pivota alpha grados (eje lateral)
  const sim = new Sim(R, 11);
  check('pivote ' + alpha + '°: se pone en cola el reajuste', det.startQuick());
  sim.run(det, 3, 0);                               // bici quieta y horizontal
  check('pivote ' + alpha + '°: reajuste hecho', det.quickState === 'done');
  const r1 = sim.run(det, 5, 1.5, { osc: 0.8 });    // arranque
  check('pivote ' + alpha + '°: comprobación sin falsa alarma', det.calibState === 'done' && det.verify === null);
  const r2 = sim.run(det, 8, 0, { osc: 0.8 });      // crucero
  check('pivote ' + alpha + '°: sin frenar en crucero', r2.seen.length === 1 && r2.seen[0] === 0, 'estados ' + r2.seen);
  const r3 = sim.run(det, 2.5, -1.8);               // frenada
  check('pivote ' + alpha + '°: detecta la frenada', r3.seen.includes(1), 't=' + r3.first);
}

// sin reajuste (calibración vieja con el móvil pivotado 25°): el error existe
{
  const { saved } = calibrated();
  const det = new Detector({ sensitivity: 1.0 }); det.setCalibration(saved);
  const sim = new Sim(mm(R0, Ry(deg(25))), 11);
  sim.run(det, 3, 0); sim.run(det, 5, 1.5, { osc: 0.8 });
  const r = sim.run(det, 8, 0, { osc: 0.8 });
  console.log('(referencia) sin reajuste, pivote 25°: fwd medio en crucero =', det.fwd.toFixed(2), 'estados', r.seen);
}

// móvil recolocado girado (50° de giro vertical): debe avisar
{
  const { saved } = calibrated();
  const det = new Detector({ sensitivity: 1.0 }); det.setCalibration(saved);
  const sim = new Sim(mm(R0, Rz(deg(50))), 13);
  det.startQuick(); sim.run(det, 3, 0);
  sim.run(det, 4, 1.5, { osc: 0.4 }); sim.run(det, 2, 0, { osc: 0.4 });
  sim.run(det, 3, 1.5, { osc: 0.4 }); sim.run(det, 3, 0, { osc: 0.4 }); sim.run(det, 3, 1.5, { osc: 0.4 });
  check('giro 50°: avisa de calibrar de nuevo', det.calibState === 'mismatch');
}

// bici en movimiento al iniciar: no reajusta y sigue con la calibración anterior
{
  const { saved } = calibrated();
  const det = new Detector({ sensitivity: 1.0 }); det.setCalibration(saved);
  const sim = new Sim(R0, 17); sim.v = 6;
  det.startQuick(); sim.run(det, 22, 0, { osc: 0.8 });
  check('rodando: abandona el reajuste a los 20 s y mantiene la calibración', det.quickState === 'timeout' && det.isCalibrated());
}
console.log(fails ? 'FALLOS: ' + fails : 'TODO OK');
process.exitCode = fails ? 1 : 0;
