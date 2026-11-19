let servoAngles = [90, 90, 90];
let servoOffsets = [0, 0, 0];
let servoEnabled = [false, false, false];
let autoIdleEnabled = false;
let servoNames = ["Servo 1 — Claw", "Servo 2 — Arm", "Servo 3 — Shoulder"];
let isOffsetMode = false;

let isDraggingSlider = [false, false, false];
let lastSliderEditMs = [0, 0, 0];
let pendingServoCmds = {};
let servoThrottleTimer = null;

function initServoStorage() {
  try {
    const storedNames = localStorage.getItem("arm_servo_names_v2");
    if (storedNames) {
      const parsed = JSON.parse(storedNames);
      if (Array.isArray(parsed) && parsed.length === 3) servoNames = parsed;
    }
    const storedOffsets = localStorage.getItem("arm_servo_offsets");
    if (storedOffsets) {
      const parsed = JSON.parse(storedOffsets);
      if (Array.isArray(parsed) && parsed.length === 3) servoOffsets = parsed;
    }
  } catch (e) {}
  applyAutoIdleState(autoIdleEnabled);
  updateAllServoUI();
}

function updateServoStateBadges() {
  for (let i = 0; i < 3; i++) {
    const badge = document.getElementById(`s${i + 1}-state-badge`);
    if (!badge) continue;
    if (servoEnabled[i]) {
      badge.innerText = "ACTIVE";
      badge.style.background = "rgba(16,185,129,0.2)";
      badge.style.color = "#34d399";
      badge.style.borderColor = "rgba(16,185,129,0.4)";
    } else {
      badge.innerText = "IDLE";
      badge.style.background = "rgba(148,163,184,0.15)";
      badge.style.color = "#94a3b8";
      badge.style.borderColor = "rgba(148,163,184,0.3)";
    }
  }
}

function updateAllServoUI() {
  for (let i = 0; i < 3; i++) {
    const n = i + 1;
    const offEl = document.getElementById(`o${n}-val`);
    if (offEl) offEl.innerText = `${servoOffsets[i] > 0 ? "+" : ""}${servoOffsets[i]}°`;

    const sl = document.getElementById(`s${n}-slider`);
    const hdr = document.getElementById(`s${n}-header`);

    const btnA = document.getElementById(`s${n}-btn-a`);
    const btnB = document.getElementById(`s${n}-btn-b`);
    const btnC = document.getElementById(`s${n}-btn-c`);
    const btnD = document.getElementById(`s${n}-btn-d`);

    if (isOffsetMode) {
      if (sl) {
        sl.classList.add("offset-slider");
        sl.min = "-45";
        sl.max = "45";
        sl.value = servoOffsets[i];
      }
      if (hdr) {
        hdr.innerHTML = `${escapeHtml(servoNames[i])} Trim: <span id="s${n}-val" style="color:#f59e0b; font-family:monospace; font-weight:700;">${servoOffsets[i] > 0 ? "+" : ""}${servoOffsets[i]}°</span>`;
      }
      if (btnA) btnA.innerText = "-1 Trim";
      if (btnB) btnB.innerText = "+1 Trim";
      if (btnC) btnC.innerText = "-5°";
      if (btnD) btnD.innerText = "+5°";
    } else {
      if (sl) {
        sl.classList.remove("offset-slider");
        sl.min = "0";
        sl.max = "180";
        sl.value = servoAngles[i];
      }
      if (hdr) {
        hdr.innerHTML = `${escapeHtml(servoNames[i])}: <span id="s${n}-val" style="color:#38bdf8; font-family:monospace; font-weight:700;">${servoAngles[i]}°</span>`;
      }
      if (btnA) btnA.innerText = "-10°";
      if (btnB) btnB.innerText = "-1°";
      if (btnC) btnC.innerText = "+1°";
      if (btnD) btnD.innerText = "+10°";
    }
  }
  updateServoStateBadges();
}

function editServoName(idx) {
  const current = servoNames[idx] || `Servo ${idx + 1}`;
  const next = prompt(`Rename Servo ${idx + 1}:`, current);
  if (next && next.trim().length > 0) {
    servoNames[idx] = next.trim().substring(0, 28);
    try { localStorage.setItem("arm_servo_names_v2", JSON.stringify(servoNames)); } catch (e) {}
    updateAllServoUI();
  }
}

function toggleOffsetMode() {
  isOffsetMode = !isOffsetMode;
  const btn = document.getElementById("offset-mode-btn");
  if (btn) {
    if (isOffsetMode) {
      btn.innerText = "Trim: ON";
      btn.style.background = "#f59e0b";
      btn.style.color = "#000000";
      showToast("Calibration Trim Mode Active", "warn");
    } else {
      btn.innerText = "Trim: OFF";
      btn.style.background = "#1e293b";
      btn.style.color = "#f59e0b";
      showToast("Servo Angle Mode Active", "info");
    }
  }
  updateAllServoUI();
}

function toggleAutoIdle() {
  autoIdleEnabled = !autoIdleEnabled;
  applyAutoIdleState(autoIdleEnabled);
  sendCommand(`autoidle ${autoIdleEnabled ? "on" : "off"}`);
  showToast(autoIdleEnabled ? "Auto-Idle ON (relaxes after 3s)" : "Auto-Idle OFF (holds torque)", "info");
}

function applyAutoIdleState(enabled) {
  autoIdleEnabled = !!enabled;
  const btn = document.getElementById("auto-idle-btn");
  if (!btn) return;
  if (autoIdleEnabled) {
    btn.innerText = "Auto-Idle: ON";
    btn.style.background = "#10b981";
    btn.style.color = "#000000";
    btn.style.borderColor = "#10b981";
  } else {
    btn.innerText = "Auto-Idle: OFF";
    btn.style.background = "#1e293b";
    btn.style.color = "rgba(255,255,255,0.75)";
    btn.style.borderColor = "#475569";
  }
}

function onSliderStart(idx) {
  isDraggingSlider[idx] = true;
  lastSliderEditMs[idx] = Date.now();
}

function onServoSlider(idx, val) {
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  const n = idx + 1;

  lastSliderEditMs[idx] = Date.now();
  servoEnabled[idx] = true;
  updateServoStateBadges();

  if (isOffsetMode) {
    servoOffsets[idx] = parsed;
    const offEl = document.getElementById(`o${n}-val`);
    if (offEl) offEl.innerText = `${parsed > 0 ? "+" : ""}${parsed}°`;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${parsed > 0 ? "+" : ""}${parsed}°`;
    pendingServoCmds[`o${n}`] = `offset s${n} ${parsed}`;
    try { localStorage.setItem("arm_servo_offsets", JSON.stringify(servoOffsets)); } catch (e) {}
  } else {
    servoAngles[idx] = parsed;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${parsed}°`;
    pendingServoCmds[`s${n}`] = `s${n} ${parsed}`;
  }

  if (!servoThrottleTimer) {
    servoThrottleTimer = setTimeout(() => {
      servoThrottleTimer = null;
      const cmds = pendingServoCmds;
      pendingServoCmds = {};
      for (const k in cmds) {
        sendCommand(cmds[k]);
      }
    }, 80);
  }
}

function onSliderCommit(idx, val) {
  isDraggingSlider[idx] = false;
  lastSliderEditMs[idx] = Date.now();
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  const n = idx + 1;

  if (isOffsetMode) {
    delete pendingServoCmds[`o${n}`];
    servoOffsets[idx] = parsed;
    sendCommand(`offset s${n} ${parsed}`);
    try { localStorage.setItem("arm_servo_offsets", JSON.stringify(servoOffsets)); } catch (e) {}
  } else {
    delete pendingServoCmds[`s${n}`];
    servoAngles[idx] = parsed;
    sendCommand(`s${n} ${parsed}`);
  }
}

function servoStepAction(idx, normalDelta, trimDelta) {
  if (isOffsetMode) {
    if (trimDelta === -1 || trimDelta === 1) {
      adjustOffset(idx, trimDelta);
    } else {
      nudgeServo(idx, trimDelta);
    }
  } else {
    nudgeServo(idx, normalDelta);
  }
}

function nudgeServo(idx, delta) {
  lastSliderEditMs[idx] = Date.now();
  let next = (servoAngles[idx] || 90) + delta;
  if (next < 0) next = 0;
  if (next > 180) next = 180;
  servoAngles[idx] = next;
  servoEnabled[idx] = true;
  updateServoStateBadges();

  const n = idx + 1;
  if (!isOffsetMode) {
    const sl = document.getElementById(`s${n}-slider`);
    if (sl) sl.value = next;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${next}°`;
  }
  sendCommand(`s${n} ${next}`);
}

function adjustOffset(idx, delta) {
  lastSliderEditMs[idx] = Date.now();
  let next = (servoOffsets[idx] || 0) + delta;
  if (next < -45) next = -45;
  if (next > 45) next = 45;
  servoOffsets[idx] = next;
  servoEnabled[idx] = true;
  updateServoStateBadges();

  const n = idx + 1;
  const offEl = document.getElementById(`o${n}-val`);
  if (offEl) offEl.innerText = `${next > 0 ? "+" : ""}${next}°`;
  if (isOffsetMode) {
    const sl = document.getElementById(`s${n}-slider`);
    if (sl) sl.value = next;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${next > 0 ? "+" : ""}${next}°`;
  }
  sendCommand(`offset s${n} ${next}`);
  appendLog("Trim", `S${n} Offset -> ${next}°`);
  try { localStorage.setItem("arm_servo_offsets", JSON.stringify(servoOffsets)); } catch (e) {}
}

async function saveCalibrationOffsets() {
  const btn = document.getElementById("save-cal-btn");
  const txt = document.getElementById("save-cal-text");
  if (txt) txt.innerText = "Saving...";
  if (btn) btn.style.background = "#0284c7";

  try {
    localStorage.setItem("arm_servo_offsets", JSON.stringify(servoOffsets));
  } catch (e) {}

  await sendCommand(`cal_set ${servoOffsets[0]} ${servoOffsets[1]} ${servoOffsets[2]}`);
  if (isOffsetMode) toggleOffsetMode();
  else updateAllServoUI();

  appendLog("Calibration", `Saved Trim Offsets to NVS: S1=${servoOffsets[0]}, S2=${servoOffsets[1]}, S3=${servoOffsets[2]}`);
  showToast("Trim offsets saved to ESP32 Flash!", "success");

  if (txt) txt.innerText = "Saved to Flash";
  if (btn) btn.style.background = "#059669";
  setTimeout(() => {
    if (txt) txt.innerText = "Save Trim to Flash";
    if (btn) btn.style.background = "#10b981";
  }, 1500);
}

function homeAllServos() {
  servoAngles = [90, 90, 90];
  servoEnabled = [true, true, true];
  if (isOffsetMode) toggleOffsetMode();
  else updateAllServoUI();
  sendCommand("home");
  showToast("Homing all servos to 90°", "info");
}

function detachAllServos() {
  servoEnabled = [false, false, false];
  updateServoStateBadges();
  sendCommand("detach");
  showToast("Servos detached (Idle)", "warn");
}

function requestCalibrationSync() {
  sendCommand("cal_get");
  showToast("Fetching trim offsets...", "info");
}

function requestStatusSync() {
  sendCommand("status");
  sendCommand("mpu_get");
  showToast("Syncing arm state...", "info");
}

function applyReceivedAngles(s1, s2, s3, forceSync = false) {
  const incoming = [s1, s2, s3];
  const now = Date.now();
  for (let i = 0; i < 3; i++) {
    if (isNaN(incoming[i])) continue;
    if (!forceSync && (isDraggingSlider[i] || (now - lastSliderEditMs[i]) < 600)) {
      continue;
    }
    servoAngles[i] = incoming[i];
    const n = i + 1;
    if (!isOffsetMode) {
      const sl = document.getElementById(`s${n}-slider`);
      if (sl) sl.value = incoming[i];
      const lbl = document.getElementById(`s${n}-val`);
      if (lbl) lbl.innerText = `${incoming[i]}°`;
    }
  }
}

function applyReceivedOffsets(o1, o2, o3) {
  const incoming = [o1, o2, o3];
  for (let i = 0; i < 3; i++) {
    if (isNaN(incoming[i])) continue;
    servoOffsets[i] = incoming[i];
    const n = i + 1;
    const offEl = document.getElementById(`o${n}-val`);
    if (offEl) offEl.innerText = `${incoming[i] > 0 ? "+" : ""}${incoming[i]}°`;
  }
  if (isOffsetMode) updateAllServoUI();
}

function applyReceivedEnableStates(e1, e2, e3) {
  servoEnabled = [!!e1, !!e2, !!e3];
  updateServoStateBadges();
}
