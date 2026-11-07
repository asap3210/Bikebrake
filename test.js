'use strict';
const { Detector, G } = require('./detector.js');

// --- utilidades ---------------------------------------------------------
function rng(seed) { return function () { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
function gauss(r) { return Math.sqrt(-2 * Math.log(r() + 1e-12)) * Math.cos(2 * Math.PI * r()); }
function rot(yaw, pitch, roll) {
  const [cy, sy, cp, sp, cr, sr] = [Math.cos(yaw), Math.sin(yaw), Math.cos(pitch), Math.sin(pitch), Math.cos(roll), Math.sin(roll)];
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr]
  ];
}
const mul = (R, v) => [0, 1, 2].map(i => R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2]);
const deg = d => d * Math.PI / 180;

// --- escenario ------------------------------------------------------------
// F = fuerza específica a lo largo de la carretera (pedal/freno/rozamiento), m/s²
// slope = pendiente en grados (+ subida), osc = amplitud del pedaleo, expect = estados admitidos
const S = (name, dur, F, slopeDeg, osc, expect, extra) => Object.assign({ name, dur, F, slopeDeg, osc, expect }, extra);
const slope10 = Math.atan(0.10) * 180 / Math.PI, down8 = -Math.atan(0.08) * 180 / Math.PI;
// Se calibra en un sitio con 4 % de bajada (hasta 4 % de error real): desfase de ~0.4 m/s² en llano.
const calSlope = -Math.atan(0.04) * 180 / Math.PI;
const makeScenario = (sens) => [
  S('calib: quieto (sitio inclinado)', 3.0, 'hold', calSlope, 0, null),
  S('calib: arranque', 4.0, 1.5, calSlope, 0.8, null),
  S('crucero llano', 14.0, 0.0, 0, 0.8, [0], { bumpAt: 4.0, slopeIs: [2.5, 5.5] }),
  S('FRENADA MODERADA llano (-1.2 m/s2, 2 s)', 2.0, -1.2, 0, 0.3, [0, 1], { mustSee: sens <= 1.0 ? 1 : undefined }),
  S('crucero tras frenar', 5.0, 1.5, 0, 0.8, [0, 1], {}),
  S('crucero llano (poner pendiente a 0)', 6.0, 0.0, 0, 0.8, [0], { level: true, slopeIs: [2.5, 6] }),
  S('rodar sin pedalear en llano', 3.0, -0.1, 0, 0, [0], { endMust: 0, slopeIs: [-1.5, 1.5] }),
  S('crucero para frenar', 4.0, 1.5, 0, 0.8, [0], {}),
  S('FRENADA en llano hasta parar', 8.0, -2.5, 0, 0, [0, 1, 2], { mustSee: 1 }),
  S('PARADO semáforo (8 s)', 8.0, 0, 0, 0, [1, 2], { endMust: 2, stopBy: 1.5 }),
  S('arranque', 4.0, 1.5, 0, 0.8, [2, 0], { endMust: 0 }),
  S('crucero llano 2', 4.0, 0.0, 0, 0.8, [0], { endMust: 0 }),
  S('RAMPA 10% pedaleando', 5.0, 1.2, slope10, 1.3, [0], { endMust: 0, slopeIs: [7.5, 12.5] }),
  S('RAMPA 10% a rueda libre (decel natural)', 3.5, -0.15, slope10, 0, [0], { endMust: 0 }),
  S('RAMPA 10% FRENANDO hasta parar', 4.0, -2.0, slope10, 0, [0, 1, 2], { mustSee: 1 }),
  S('PARADO en la rampa (6 s)', 6.0, 0, slope10, 0, [1, 2], { endMust: 2, stopBy: 2.0, slopeIs: [8, 12] }),
  S('arranque en rampa', 5.0, 2.8, slope10, 1.3, [2, 0], { endMust: 0 }),
  S('BAJADA 8% a rueda libre (acelera)', 16.0, -0.1, down8, 0, [0], { endMust: 0, slopeIs: [-9.5, -6] }),
  S('BAJADA 8% FRENANDO hasta parar', 9.0, -3.0, down8, 0, [0, 1, 2], { mustSee: 1 }),
  S('PARADO bajada (5 s)', 5.0, 'hold', down8, 0, [1, 2], { endMust: 2, stopBy: 2.5 })
];

function run(sens) {
  console.log('\n=== sensibilidad ' + sens + ' ===');
  const R = rot(deg(35), deg(55), deg(-20));
  const rnd = rng(42);
  const det = new Detector({ sensitivity: sens });
  det.startCalibration();
  const hz = 60, dt = 1 / hz;
  let t = 0, v = 0;
  const report = [];
  let failures = 0;

  for (const ph of makeScenario(sens)) {
    const seen = new Set(); let last = -1, firstTrans = null, endState = null, calAt = null, t2 = null;
    const t0 = t; let slopeEnd = null;
    const n = Math.round(ph.dur * hz);
    for (let i = 0; i < n; i++) {
      const lt = i * dt;
      const th = deg(ph.slopeDeg);
      let F = (ph.F === 'hold' ? G * Math.sin(th) : ph.F) + (v > 1 ? ph.osc * Math.sin(2 * Math.PI * 2.4 * t) : 0);
      if (ph.bumpAt !== undefined && lt > ph.bumpAt && lt < ph.bumpAt + 0.12) F += -7;  // bache/bordillo
      let a = F - G * Math.sin(th);
      if (v <= 0 && a < 0) { v = 0; a = 0; F = G * Math.sin(th); }          // parado, sujeto por el freno
      else v = Math.max(0, v + a * dt);
      const moving = v > 0.3;
      const sv = moving ? Math.min(1, v / 3) : 0;
      const mb = [
        F + G * 0 + G * Math.sin(th) * 0,   // F ya es la fuerza específica longitudinal
        gauss(rnd) * (0.03 + 0.5 * sv),
        G * Math.cos(th) + gauss(rnd) * (0.03 + 1.0 * sv)
      ];
      mb[0] += gauss(rnd) * (0.03 + 0.5 * sv);
      const m = mul(R, mb);
      if (i % hz === 0 && det.calibState === 'done') det.setGpsSpeed(t, Math.max(0, v + (v > 0 ? gauss(rnd) * 0.08 : 0)));
      const s = det.update(t, m[0], m[1], m[2]);
      if (ph.expect) {
        seen.add(s);
        if (s !== last && firstTrans === null && s !== 0) firstTrans = (t - t0).toFixed(2);
      }
      if (s === 2 && t2 === null) t2 = t - t0;
      last = s; endState = s; t += dt; slopeEnd = det.slopePct;
      if (det.calibState === 'done' && calAt === null) calAt = t;
    }
    if (ph.level) det.setLevel();
    if (!ph.expect) { report.push({ fase: ph.name, ok: det.calibState === 'done' || det.calibState === 'push' || det.calibState === 'rest', visto: '-', fin: det.calibState, v: v.toFixed(1) }); continue; }
    let ok = true;
    for (const s of seen) if (!ph.expect.includes(s)) ok = false;
    if (ph.mustSee !== undefined && !seen.has(ph.mustSee)) ok = false;
    if (ph.endMust !== undefined && endState !== ph.endMust) ok = false;
    if (ph.stopBy !== undefined && (t2 === null || t2 > ph.stopBy)) ok = false;
    if (ph.slopeIs && (slopeEnd < ph.slopeIs[0] || slopeEnd > ph.slopeIs[1])) ok = false;
    if (!ok) failures++;
    report.push({ fase: ph.name, ok, visto: [...seen].join(','), t1: firstTrans, t_parado: t2 === null ? null : t2.toFixed(2), fin: endState, pend: slopeEnd === null ? null : slopeEnd.toFixed(1), v: v.toFixed(1) });
  }
  console.log('Calibración:', det.calibState, 'f=', det.calib.f && det.calib.f.map(x => x.toFixed(2)));
  console.table(report);
  console.log('Eventos:'); det.events.forEach(e => console.log(' ', e.t.toFixed(1).padStart(6), e.msg));
  console.log(failures ? `FALLOS: ${failures}` : 'TODO OK');
  return failures;
}
process.exitCode = [1.0, 0.5, 1.6].reduce((n, sens) => n + run(sens), 0) ? 1 : 0;
