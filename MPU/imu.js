// MPU-6050 telemetry, calibration and motion-control UI.
// Protocol lines from the ESP32 (comma separated key=value unless noted):
//   IMU:roll,pitch,yaw,ax,ay,az,gx,gy,gz,temp
//   MPUHW:ok=1,addr=104
//   MPUCAL:gyro=1,accel=1,nr=0.0,np=0.0,ny=0.0,gbx=..,gby=..,gbz=..,abx=..,aby=..,abz=..
//   MPUCALPROG:kind=gyro,pct=50
//   MPUCALRES:kind=gyro,ok=1,msg=Gyro bias saved
//   MPUCTRL:on=1,gain=2.00,er=1,ep=1,ey=0,ir=0,ip=0,iy=0

const IMU_STALE_MS = 1500;

let imuData = { roll: 0, pitch: 0, yaw: 0, ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0, temp: 0 };
let imuLastPacketMs = 0;
let imuPacketTimes = [];
let imuHwOk = null;
let imuCal = { gyro: false, accel: false, nr: 0, np: 0, ny: 0, busy: "" };
let imuCtrl = { on: false, gain: 2, axes: [true, true, false], inv: [false, false, false] };
let imuGainDragging = false;
let imuGainTimer = null;

const IMU_AXIS_ROWS = [
  { key: "roll", label: "Roll", target: "Arm (S2)" },
  { key: "pitch", label: "Pitch", target: "Shoulder (S3)" },
  { key: "yaw", label: "Yaw", target: "Claw (S1)" }
];

function parseKv(body) {
  const out = {};
  body.split(",").forEach(part => {
    const eq = part.indexOf("=");
    if (eq > 0) out[part.substring(0, eq).trim()] = part.substring(eq + 1).trim();
  });
  return out;
}

function setText(id, value) {
  const el = document.getElementById(id);
  if (el) el.innerText = value;
}

function fmtSigned(v, digits) {
  const n = Number(v);
  if (!isFinite(n)) return "--";
  return (n >= 0 ? "+" : "") + n.toFixed(digits);
}

function handleImuLine(text) {
  if (text.startsWith("IMU:")) {
    const v = text.substring(4).split(",").map(Number);
    if (v.length >= 10 && v.every(n => isFinite(n))) {
      applyImuTelemetry(v);
    }
    return true;
  }
  if (text.startsWith("MPUHW:")) {
    const kv = parseKv(text.substring(6));
    imuHwOk = kv.ok === "1";
    renderImuLinkBadge();
    return true;
  }
  if (text.startsWith("MPUCALPROG:")) {
    const kv = parseKv(text.substring(11));
    const pct = Math.max(0, Math.min(100, parseInt(kv.pct, 10) || 0));
    imuCal.busy = kv.kind || "";
    renderCalProgress(pct);
    return true;
  }
  if (text.startsWith("MPUCALRES:")) {
    const kv = parseKv(text.substring(10));
    imuCal.busy = "";
    renderCalProgress(-1);
    const ok = kv.ok === "1";
    const msg = kv.msg || (ok ? "Calibration complete" : "Calibration failed");
    setText("cal-hint", msg);
    showToast(msg, ok ? "success" : "error");
    appendLog("MPU", `${kv.kind || "cal"}: ${msg}`);
    setCalButtonsDisabled(false);
    return true;
  }
  if (text.startsWith("MPUCAL:")) {
    const kv = parseKv(text.substring(7));
    imuCal.gyro = kv.gyro === "1";
    imuCal.accel = kv.accel === "1";
    imuCal.nr = parseFloat(kv.nr) || 0;
    imuCal.np = parseFloat(kv.np) || 0;
    imuCal.ny = parseFloat(kv.ny) || 0;
    renderCalStatus(kv);
    return true;
  }
  if (text.startsWith("MPUCTRL:")) {
    const kv = parseKv(text.substring(8));
    imuCtrl.on = kv.on === "1";
    const g = parseFloat(kv.gain);
    if (isFinite(g)) imuCtrl.gain = g;
    imuCtrl.axes = [kv.er === "1", kv.ep === "1", kv.ey === "1"];
    imuCtrl.inv = [kv.ir === "1", kv.ip === "1", kv.iy === "1"];
    renderCtrlState();
    return true;
  }
  return false;
}

function applyImuTelemetry(v) {
  imuData = { roll: v[0], pitch: v[1], yaw: v[2], ax: v[3], ay: v[4], az: v[5], gx: v[6], gy: v[7], gz: v[8], temp: v[9] };
  const now = Date.now();
  imuLastPacketMs = now;
  imuPacketTimes.push(now);
  while (imuPacketTimes.length > 0 && now - imuPacketTimes[0] > 1000) imuPacketTimes.shift();

  if (typeof Scene3D !== "undefined") {
    Scene3D.setOrientation(imuData.roll, imuData.pitch, imuData.yaw);
  }
  renderImuReadouts();
}

function renderImuReadouts() {
  setText("imu-roll", fmtSigned(imuData.roll, 1));
  setText("imu-pitch", fmtSigned(imuData.pitch, 1));
  setText("imu-yaw", fmtSigned(imuData.yaw, 1));
  setText("imu-ax", fmtSigned(imuData.ax, 3));
  setText("imu-ay", fmtSigned(imuData.ay, 3));
  setText("imu-az", fmtSigned(imuData.az, 3));
  setText("imu-gx", fmtSigned(imuData.gx, 1));
  setText("imu-gy", fmtSigned(imuData.gy, 1));
  setText("imu-gz", fmtSigned(imuData.gz, 1));
  setText("imu-temp", `${imuData.temp.toFixed(1)} C`);
  const mag = Math.sqrt(imuData.ax * imuData.ax + imuData.ay * imuData.ay + imuData.az * imuData.az);
  setText("imu-mag", `${mag.toFixed(3)} g`);
}

function renderImuLinkBadge() {
  const badge = document.getElementById("imu-link-badge");
  if (!badge) return;
  const age = Date.now() - imuLastPacketMs;
  let text;
  let cls;
  if (imuHwOk === false) {
    text = "Sensor not found";
    cls = "chip chip-bad";
  } else if (imuLastPacketMs > 0 && age < IMU_STALE_MS) {
    text = `${imuPacketTimes.length} Hz`;
    cls = "chip chip-ok";
  } else {
    text = "No IMU data";
    cls = "chip";
  }
  badge.innerText = text;
  badge.className = cls;
}

function renderCalStatus(kv) {
  const g = document.getElementById("cal-chip-gyro");
  const a = document.getElementById("cal-chip-accel");
  if (g) {
    g.innerText = imuCal.gyro ? "Gyro: calibrated" : "Gyro: not calibrated";
    g.className = imuCal.gyro ? "chip chip-ok" : "chip chip-warn";
  }
  if (a) {
    a.innerText = imuCal.accel ? "Accel: calibrated" : "Accel: not calibrated";
    a.className = imuCal.accel ? "chip chip-ok" : "chip chip-warn";
  }
  setText("cal-neutral", `Neutral  R ${fmtSigned(imuCal.nr, 1)}  P ${fmtSigned(imuCal.np, 1)}  Y ${fmtSigned(imuCal.ny, 1)}`);
  if (kv) {
    setText("cal-gyro-bias", `Gyro bias  ${fmtSigned(kv.gbx, 2)}  ${fmtSigned(kv.gby, 2)}  ${fmtSigned(kv.gbz, 2)} dps`);
    setText("cal-accel-bias", `Accel bias  ${fmtSigned(kv.abx, 3)}  ${fmtSigned(kv.aby, 3)}  ${fmtSigned(kv.abz, 3)} g`);
  }
}

function renderCalProgress(pct) {
  const wrap = document.getElementById("cal-progress");
  const bar = document.getElementById("cal-progress-bar");
  if (!wrap || !bar) return;
  if (pct < 0) {
    wrap.style.display = "none";
    bar.style.width = "0%";
    return;
  }
  wrap.style.display = "block";
  bar.style.width = `${pct}%`;
}

function setCalButtonsDisabled(disabled) {
  document.querySelectorAll(".cal-action").forEach(btn => {
    btn.disabled = disabled;
    btn.style.opacity = disabled ? "0.5" : "1";
  });
}

const CAL_HINTS = {
  gyro: "Keep the sensor stationary in its standing position (on pins). Sampling gyro bias for ~2 seconds...",
  accel: "Keep the sensor resting stationary on its pins. Sampling accelerometer bias...",
  neutral: "Current standing orientation locked as 0 deg neutral center!",
  yaw: "Yaw zeroed at current heading.",
  clear: "All MPU calibration cleared."
};

function imuCalibrate(kind) {
  if (!isConnected) {
    showToast("Connect to the ESP32 first.", "warn");
    return;
  }
  setText("cal-hint", CAL_HINTS[kind] || "Calibrating...");
  if (kind === "gyro" || kind === "accel") {
    setCalButtonsDisabled(true);
    renderCalProgress(0);
  }
  sendCommand(`mpu_cal ${kind}`);
  if (kind === "neutral" || kind === "yaw" || kind === "clear") {
    setTimeout(() => sendCommand("mpu_get"), 350);
  }
}

function toggleMpuControl() {
  if (!isConnected) {
    showToast("Connect to the ESP32 first.", "warn");
    return;
  }
  const next = !imuCtrl.on;
  imuCtrl.on = next;
  renderCtrlState();
  sendCommand(`mpu_ctrl ${next ? "on" : "off"}`);
  showToast(next ? "MPU control ON - arm follows the sensor" : "MPU control OFF", next ? "success" : "info");
}

function onMpuGainInput(val) {
  imuGainDragging = true;
  const g = parseFloat(val);
  if (!isFinite(g)) return;
  imuCtrl.gain = g;
  setText("ctrl-gain-val", `${g.toFixed(2)}x`);
  sendCommand(`mpu_gain ${g.toFixed(2)}`);
  if (imuGainTimer) clearTimeout(imuGainTimer);
  imuGainTimer = setTimeout(() => { imuGainDragging = false; }, 600);
}

function onMpuGainCommit(val) {
  const g = parseFloat(val);
  if (!isFinite(g)) return;
  imuCtrl.gain = g;
  sendCommand(`mpu_gain ${g.toFixed(2)} save`);
}

function applyMpuControlFlag(on) {
  if (imuCtrl.on === on) return;
  imuCtrl.on = on;
  renderCtrlState();
}

function toggleMpuAxis(i) {
  imuCtrl.axes[i] = !imuCtrl.axes[i];
  renderCtrlState();
  sendMpuAxes();
}

function toggleMpuInvert(i) {
  imuCtrl.inv[i] = !imuCtrl.inv[i];
  renderCtrlState();
  sendCommand(`mpu_inv ${imuCtrl.inv.map(b => (b ? 1 : 0)).join(" ")}`);
}

function sendMpuAxes() {
  sendCommand(`mpu_axes ${imuCtrl.axes.map(b => (b ? 1 : 0)).join(" ")}`);
}

function renderCtrlState() {
  const btn = document.getElementById("ctrl-toggle-btn");
  const txt = document.getElementById("ctrl-toggle-text");
  if (btn && txt) {
    txt.innerText = imuCtrl.on ? "MPU Control: ON" : "MPU Control: OFF";
    btn.className = "ctrl-toggle" + (imuCtrl.on ? " on" : "");
  }
  if (!imuGainDragging) {
    const slider = document.getElementById("ctrl-gain");
    if (slider) slider.value = imuCtrl.gain;
    setText("ctrl-gain-val", `${imuCtrl.gain.toFixed(2)}x`);
  }
  for (let i = 0; i < 3; i++) {
    const en = document.getElementById(`ctrl-en-${i}`);
    const inv = document.getElementById(`ctrl-inv-${i}`);
    if (en) {
      en.className = "mini-toggle" + (imuCtrl.axes[i] ? " on" : "");
      en.innerText = imuCtrl.axes[i] ? "Enabled" : "Disabled";
    }
    if (inv) {
      inv.className = "mini-toggle" + (imuCtrl.inv[i] ? " on-warn" : "");
      inv.innerText = imuCtrl.inv[i] ? "Inverted" : "Normal";
    }
  }
}

function buildAxisRows() {
  const host = document.getElementById("ctrl-axis-rows");
  if (!host) return;
  host.innerHTML = "";
  IMU_AXIS_ROWS.forEach((row, i) => {
    const div = document.createElement("div");
    div.className = "axis-row";
    div.innerHTML = `
      <div class="axis-name"><span>${row.label}</span><span class="axis-target">${row.target}</span></div>
      <button type="button" id="ctrl-en-${i}" class="mini-toggle" onclick="toggleMpuAxis(${i})">Enabled</button>
      <button type="button" id="ctrl-inv-${i}" class="mini-toggle" onclick="toggleMpuInvert(${i})">Normal</button>
    `;
    host.appendChild(div);
  });
}

function setSceneMode(mode) {
  if (typeof Scene3D !== "undefined") Scene3D.setMode(mode);
  const a = document.getElementById("seg-arm");
  const b = document.getElementById("seg-imu");
  if (a) a.className = "seg-btn" + (mode === "arm" ? " active" : "");
  if (b) b.className = "seg-btn" + (mode === "imu" ? " active" : "");
  setText("scene-hint", mode === "arm"
    ? "Arm model follows the live servo angles. Drag to orbit, pinch to zoom."
    : "GY-521 board stands upright on its pins. Drag to orbit, pinch to zoom.");
}

function resetSceneView() {
  if (typeof Scene3D !== "undefined") Scene3D.resetView();
}

function onImuLinkLost() {
  imuLastPacketMs = 0;
  imuPacketTimes = [];
  imuHwOk = null;
  imuCtrl.on = false;
  renderCtrlState();
  renderImuLinkBadge();
  renderCalProgress(-1);
  setCalButtonsDisabled(false);
}

function initImuUI() {
  buildAxisRows();
  renderCtrlState();
  renderCalStatus(null);
  renderImuReadouts();
  renderImuLinkBadge();
  renderCalProgress(-1);
  setSceneMode("imu");
  if (typeof Scene3D !== "undefined") {
    Scene3D.init(document.getElementById("scene-canvas"));
  }
  setInterval(() => {
    if (imuPacketTimes.length > 0) {
      const now = Date.now();
      while (imuPacketTimes.length > 0 && now - imuPacketTimes[0] > 1000) imuPacketTimes.shift();
    }
    renderImuLinkBadge();
  }, 500);
}