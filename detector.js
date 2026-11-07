/* Detector de frenada para bici.
 *
 * Idea clave: el acelerómetro mide "fuerza específica" (aceleración real menos gravedad).
 * Proyectada sobre el eje delantero de la bici: fwd = a_real + g·sen(pendiente).
 * A rueda libre da ~0 en llano, en rampa o en bajada (la rampa no cuenta como frenada);
 * solo el freno la hace claramente negativa.
 *
 * Pendiente: a velocidad constante fwd = g·sen(pendiente), así que
 *   pendiente ≈ (fwd lento − aceleración medida por GPS) / g
 * (sin GPS se supone velocidad constante). Es la pendiente respecto al sitio donde
 * calibraste; "setLevel()" la pone a 0 en una calle llana y corrige el desfase.
 *
 * Estados de la luz:
 *   0 RODANDO   luz tenue
 *   1 FRENANDO  luz fuerte (destellos al inicio, luego fija)
 *   2 PARADO    luz fuerte fija (semáforo): se queda encendida hasta que arrancas
 *   3 EMERGENCIA frenada muy fuerte, destello rápido
 *
 * Todo se alimenta con update(t, mx, my, mz): t en segundos, m = accelerationIncludingGravity (m/s²).
 */
(function (root) {
  'use strict';

  var G = 9.80665;

  function V(x, y, z) { return [x, y, z]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function norm(a) { return Math.sqrt(dot(a, a)); }
  function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function unit(a) { var n = norm(a); return n > 1e-9 ? scale(a, 1 / n) : null; }

  function Detector(opts) {
    opts = opts || {};
    this.cfg = {
      sensitivity: 1.0,       // multiplica los umbrales (menor = más sensible)
      brakeOn: -0.7,          // m/s² filtrados para entrar en FRENANDO
      brakeOff: -0.3,         // m/s² para salir
      brakeOnTime: 0.35,      // s acumulados bajo el umbral para confirmar (filtra baches)
      brakeOffTime: 0.80,
      emergency: -4.0,        // m/s² para EMERGENCIA
      fwdTau: 0.20,           // s, filtro de aceleración longitudinal
      medianWin: 0.22,        // s, ventana de mediana (quita baches de <0.1 s)
      slopeTau: 2.5,          // s, filtro lento para estimar la pendiente
      gpsAccTau: 1.5,         // s, filtro de la aceleración medida por GPS
      trim: 0,                // m/s² de desfase del eje (se aprende con setLevel)
      vibTau: 0.25,           // s, energía de vibración
      stillRms: 0.25,         // m/s² por debajo = quieto
      stillTime: 1.0,         // s quieto para declarar PARADO
      moveRms: 0.60,          // m/s² por encima = rodando
      moveTime: 0.6,
      startAccel: 0.8,        // m/s² positivos que cuentan como arranque
      startTime: 0.3,
      parkedTimeout: 600,     // s: tras 10 min parado, se apaga el aviso (bici aparcada)
      recentMoveWindow: 30,   // s: solo hay PARADO si te estabas moviendo hace poco
      gpsFresh: 3,            // s: antigüedad máxima de una velocidad GPS válida
      flipSign: false
    };
    for (var k in opts) this.cfg[k] = opts[k];

    this.calib = { u0: null, f: null };
    this.calibState = 'none'; // none | rest | push | done | failed
    this._cal = null;
    this.reset();
  }

  Detector.prototype.reset = function () {
    this.state = 0;
    this.t = null;
    this.gLP = null;          // paso bajo lento de m (pendiente)
    this.gFast = null;        // paso bajo rápido de m (vibración)
    this.fwdRaw = 0;
    this.fwd = 0;             // aceleración longitudinal filtrada y corregida (m/s²), negativa = frenando
    this.slopeInit = false;
    this.warmT0 = null;
    this.fwdInit = false;
    this.fwdMid = 0;          // fwd a escala de ~1 s y a escala de ~4 s: detectan cambios
    this.fwdLong = 0;
    this.aSlow = 0;
    this.medBuf = [];
    this.slopeSin = 0;        // seno de la pendiente estimada
    this.slopePct = 0;        // pendiente en %
    this.slopeSrc = 'none';   // none | gps | imu
    this.aGps = 0;            // aceleración por GPS, suavizada (m/s²)
    this._aUse = 0;
    this._gpsPrev = null;
    this.brakeAcc = 0;
    this.vib = 0;             // RMS de vibración (m/s²)
    this.lastMoving = -1e9;
    this.lastBrakeEnd = -1e9;
    this.stillSince = null;
    this.movingSince = null;
    this.brakeSince = null;
    this.releaseSince = null;
    this.startSince = null;
    this.stoppedAt = null;
    this.gps = { speed: null, t: -1e9 };
    this.events = [];
  };

  Detector.prototype.setGpsSpeed = function (t, speedMs) {
    var p = this._gpsPrev;
    if (p && t - p.t >= 0.4 && t - p.t <= 2.5) {
      var a = Math.max(-6, Math.min(6, (speedMs - p.v) / (t - p.t)));
      this.aGps += (1 - Math.exp(-(t - p.t) / this.cfg.gpsAccTau)) * (a - this.aGps);
      this.aSlow += (1 - Math.exp(-(t - p.t) / 2.5)) * (a - this.aSlow);
    } else if (!p || t - p.t > 5) { this.aGps = 0; this.aSlow = 0; }
    if (!p || t - p.t >= 0.4) this._gpsPrev = { t: t, v: speedMs };
    this.gps = { speed: speedMs, t: t };
  };

  /* Pone la pendiente actual a 0 (calle llana) y corrige el desfase del eje. */
  Detector.prototype.setLevel = function () {
    if (!this.isCalibrated() || !this.slopeInit) return false;
    var nt = Math.max(-1.5, Math.min(1.5, this.cfg.trim + G * this.slopeSin));
    this.fwd -= nt - this.cfg.trim;
    this.cfg.trim = nt;
    this._log(this.t, 'Pendiente puesta a 0');
    return true;
  };

  Detector.prototype.isCalibrated = function () { return !!(this.calib.f && this.calib.u0); };

  Detector.prototype.setCalibration = function (c) {
    if (c && c.f && c.u0) { this.calib = { f: c.f.slice(), u0: c.u0.slice() }; this.calibState = 'done'; }
  };

  /* Calibración: 1) quieto en llano 1.5 s  2) arrancar pedaleando recto ~3 s. */
  Detector.prototype.startCalibration = function () {
    this.calibState = 'rest';
    this._cal = { restSum: [0, 0, 0], restN: 0, restStart: null, u0: null, pushSum: [0, 0, 0], pushN: 0, pushStart: null, lastT: null, vibLP: null, vib: 0 };
    this.calib = { u0: null, f: null };
    this.cfg.trim = 0;
    this.slopeInit = false; this.warmT0 = null;
  };

  Detector.prototype._calStep = function (t, m) {
    var c = this._cal;
    if (c.lastT === null) { c.lastT = t; c.vibLP = m.slice(); return; }
    var dt = Math.max(1e-3, t - c.lastT); c.lastT = t;
    var a = 1 - Math.exp(-dt / 1.0);
    for (var i = 0; i < 3; i++) c.vibLP[i] += a * (m[i] - c.vibLP[i]);
    var hp = sub(m, c.vibLP);
    var av = 1 - Math.exp(-dt / 0.4);
    c.vib += av * (dot(hp, hp) - c.vib);
    var rms = Math.sqrt(Math.max(0, c.vib));

    if (this.calibState === 'rest') {
      if (rms > 0.35) { c.restStart = null; c.restSum = [0, 0, 0]; c.restN = 0; return; }
      if (c.restStart === null) c.restStart = t;
      for (var j = 0; j < 3; j++) c.restSum[j] += m[j];
      c.restN++;
      if (t - c.restStart >= 1.5 && c.restN > 20) {
        var mean = scale(c.restSum, 1 / c.restN);
        c.u0 = unit(mean);
        this.calibState = 'push';
      }
    } else if (this.calibState === 'push') {
      if (c.pushStart === null) {
        if (rms > 0.6) { c.pushStart = t; }
        else return;
      }
      // componente horizontal respecto a la vertical calibrada
      var along = dot(m, c.u0);
      var h = sub(m, scale(c.u0, along));
      for (var k = 0; k < 3; k++) c.pushSum[k] += h[k];
      c.pushN++;
      if (t - c.pushStart >= 3.0) {
        var meanH = scale(c.pushSum, 1 / c.pushN);
        if (norm(meanH) < 0.3) { this.calibState = 'failed'; return; }
        this.calib = { u0: c.u0, f: unit(meanH) };
        this.calibState = 'done';
      }
    }
  };

  Detector.prototype._log = function (t, msg) {
    this.events.push({ t: t, msg: msg });
    if (this.events.length > 50) this.events.shift();
  };

  Detector.prototype._set = function (t, s, why) {
    if (s !== this.state) {
      this.state = s;
      this._log(t, why);
      return true;
    }
    return false;
  };

  /* Devuelve el estado de la luz (0..3). */
  Detector.prototype.update = function (t, mx, my, mz) {
    var m = [mx, my, mz];
    var cfg = this.cfg;

    if (this.calibState === 'rest' || this.calibState === 'push') {
      this._calStep(t, m);
      this.t = t;
      return this.state;
    }

    var dt = this.t === null ? 0.016 : Math.max(1e-3, Math.min(0.2, t - this.t));
    this.t = t;

    // vibración (alta frecuencia, sin gravedad)
    if (this.gLP === null) { this.gLP = m.slice(); this.gFast = m.slice(); }
    var aG = 1 - Math.exp(-dt / 1.0);
    var aGf = 1 - Math.exp(-dt / 0.3);
    for (var i = 0; i < 3; i++) {
      this.gLP[i] += aG * (m[i] - this.gLP[i]);
      this.gFast[i] += aGf * (m[i] - this.gFast[i]);
    }
    var hp = sub(m, this.gFast);
    var aV = 1 - Math.exp(-dt / cfg.vibTau);
    this.vib += aV * (dot(hp, hp) - this.vib);
    var rms = Math.sqrt(Math.max(0, this.vib));

    if (!this.isCalibrated()) return this.state;

    // aceleración longitudinal (fuerza específica sobre el eje delantero)
    var sgn = cfg.flipSign ? -1 : 1;
    this.fwdRaw = sgn * dot(m, this.calib.f);
    // mediana corta: un bache (<0.1 s) no puede parecer una frenada
    var buf = this.medBuf;
    buf.push({ t: t, v: this.fwdRaw });
    while (buf.length > 1 && t - buf[0].t > cfg.medianWin) buf.shift();
    var srt = buf.map(function (x) { return x.v; }).sort(function (a, b) { return a - b; });
    var med = srt[srt.length >> 1];
    if (!this.fwdInit) { this.fwd = this.fwdMid = this.fwdLong = med; this.fwdInit = true; }
    this.fwdMid += (1 - Math.exp(-dt / 1.0)) * (med - this.fwdMid);
    this.fwdLong += (1 - Math.exp(-dt / 4.0)) * (med - this.fwdLong);
    var aF = 1 - Math.exp(-dt / cfg.fwdTau);
    this.fwd += aF * ((med - cfg.trim) - this.fwd);

    var gpsOk = this.gps.speed !== null && (t - this.gps.t) <= cfg.gpsFresh;
    var gpsMoving = gpsOk && this.gps.speed > 2.0;
    var gpsStill = gpsOk && this.gps.speed < 0.6;

    // pendiente: fwd menos la aceleración real medida por GPS, promediado despacio.
    // Si la velocidad cambia deprisa (pedaleo fuerte, frenada) se congela: el GPS llega
    // con retraso y falsearía el cálculo.
    this._aUse = (gpsOk && !gpsStill) ? this.aGps : 0;
    this.slopeSrc = gpsOk ? 'gps' : 'imu';
    var steady = Math.abs(this.fwdMid - this.fwdLong) < 0.5 &&
      (!(gpsOk && !gpsStill) || Math.abs(this.aGps - this.aSlow) < 0.4);
    var x = (med - cfg.trim - this._aUse) / G;
    if (this.warmT0 === null) this.warmT0 = t;
    if (steady && t - this.warmT0 > 3) {
      if (!this.slopeInit) { this.slopeSin = x; this.slopeInit = true; }
      else this.slopeSin += (1 - Math.exp(-dt / cfg.slopeTau)) * (x - this.slopeSin);
    }
    var sc = Math.max(-0.3, Math.min(0.3, this.slopeSin));
    this.slopePct = 100 * sc / Math.sqrt(1 - sc * sc);

    var sens = cfg.sensitivity;
    var still = rms < cfg.stillRms && !gpsMoving;
    var moving = rms > cfg.moveRms || gpsMoving;
    if (moving) this.lastMoving = t;

    // ---- tiempos acumulados ----
    if (still) { if (this.stillSince === null) this.stillSince = t; } else this.stillSince = null;
    if (rms > cfg.moveRms) { if (this.movingSince === null) this.movingSince = t; } else this.movingSince = null;

    var s0 = this.state;
    var braking = this.fwd < cfg.brakeOn * sens;
    var releasing = this.fwd > cfg.brakeOff * sens;
    var emergency = this.fwd < cfg.emergency * Math.max(sens, 0.85); // no tan sensible como el freno normal
    var longStill = this.stillSince !== null && (t - this.stillSince) >= cfg.stillTime && (!gpsOk || gpsStill);
    var rolling = this.movingSince !== null && (t - this.movingSince) >= cfg.moveTime;
    var recentlyMoved = (t - this.lastMoving) <= cfg.recentMoveWindow + cfg.stillTime + 1;

    if (this.fwd > cfg.startAccel) { if (this.startSince === null) this.startSince = t; } else this.startSince = null;
    var starting = this.startSince !== null && (t - this.startSince) >= cfg.startTime;

    switch (this.state) {
      case 0: // RODANDO
        // acumulador con fuga: el ruido del pedaleo no reinicia la cuenta
        if (braking && !longStill) this.brakeAcc += dt;
        else this.brakeAcc = Math.max(0, this.brakeAcc - 1.5 * dt);
        if (this.brakeAcc >= cfg.brakeOnTime) {
          this._set(t, emergency ? 3 : 1, emergency ? 'Frenada de emergencia' : 'Frenando');
          this.brakeAcc = 0;
        }
        if (this.state === 0 && longStill && recentlyMoved) {
          this._set(t, 2, 'Parado (luz encendida)');
          this.stoppedAt = t;
        }
        break;

      case 1: // FRENANDO
      case 3:
        if (emergency && this.state === 1) this._set(t, 3, 'Frenada de emergencia');
        if (longStill) {
          this._set(t, 2, 'Parado (luz encendida)');
          this.stoppedAt = t;
          this.releaseSince = null;
        } else if (releasing && rms > cfg.moveRms) {
          // soltó el freno y sigue rodando
          if (this.releaseSince === null) this.releaseSince = t;
          if (t - this.releaseSince >= cfg.brakeOffTime) {
            this._set(t, 0, 'Suelto el freno');
            this.releaseSince = null;
            this.lastBrakeEnd = t;
          }
        } else this.releaseSince = null;
        break;

      case 2: // PARADO
        var movedLong = this.movingSince !== null && (t - this.movingSince) >= 1.5;
        if ((rolling && starting) || movedLong || (rolling && gpsMoving)) {
          this._set(t, 0, 'En marcha');
        } else if (this.stoppedAt !== null && (t - this.stoppedAt) > cfg.parkedTimeout) {
          this._set(t, 0, 'Bici aparcada, aviso apagado');
        }
        break;
    }
    return this.state;
  };

  var api = { Detector: Detector, G: G };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.BikeDetector = api;
})(typeof window !== 'undefined' ? window : this);
