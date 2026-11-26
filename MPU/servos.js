const DEFAULT_SERVO_NAMES = [
  "Servo 1 — Claw",
  "Servo 2 — Arm",
  "Servo 3 — Shoulder",
  "Scissor Lift (MG996R)"
];

const DEFAULT_BUTTON_LABELS = {
  home: "Home All (90°)",
  detach: "Detach (Idle)",
  sync: "Sync State",
  stop: "STOP Motion",
  clawMin: "Close",
  clawMid: "Mid",
  clawMax: "Open",
  scissorMin: "Min",
  scissorMid: "Mid",
  scissorMax: "Max"
};

let servoAngles = [90, 90, 90, 90];
let servoOffsets = [0, 0, 0, 0];
let servoPins = [1, 38, 39, 12];
let servoEnabled = [false, false, false, false];
let autoIdleEnabled = false;
let servoNames = [...DEFAULT_SERVO_NAMES];
let buttonLabels = { ...DEFAULT_BUTTON_LABELS };
let isOffsetMode = false;
let isEditNamesMode = false;
let pendingRenameTarget = null; // { kind: "servo" | "button", key: number | string }

// Claw range state (Servo 1 / idx 0)
let clawMin = 0;
let clawMax = 180;
let draftClawMin = 0;
let draftClawMax = 180;
let pendingClawRangeSync = false;

// Scissor lift range and proportional height state (Servo 4 / idx 3)
let scissorMin = 0;
let scissorMax = 180;
let draftMin = 0;
let draftMax = 180;
let scissorPercent = 50;
let pendingScissorRangeSync = false;

let isDraggingSlider = [false, false, false, false];
let lastSliderEditMs = [0, 0, 0, 0];
let isDraggingLift = false;
let pendingServoCmds = {};
let servoThrottleTimer = null;

// Global release protection prevents drag states from sticking on mobile devices
window.addEventListener("pointerup", () => {
  isDraggingLift = false;
  for (let i = 0; i < 4; i++) isDraggingSlider[i] = false;
});
window.addEventListener("touchend", () => {
  isDraggingLift = false;
  for (let i = 0; i < 4; i++) isDraggingSlider[i] = false;
});

function markPendingRangeSync() {
  pendingClawRangeSync = true;
  pendingScissorRangeSync = true;
}

function initServoStorage() {
  try {
    const storedNames = localStorage.getItem("arm_servo_names_v3");
    if (storedNames) {
      const parsed = JSON.parse(storedNames);
      if (Array.isArray(parsed) && parsed.length >= 3) {
        servoNames[0] = parsed[0] || DEFAULT_SERVO_NAMES[0];
        servoNames[1] = parsed[1] || DEFAULT_SERVO_NAMES[1];
        servoNames[2] = parsed[2] || DEFAULT_SERVO_NAMES[2];
        servoNames[3] = parsed[3] || DEFAULT_SERVO_NAMES[3];
      }
    }
    const storedBtnLabels = localStorage.getItem("arm_button_labels_v1");
    if (storedBtnLabels) {
      const parsed = JSON.parse(storedBtnLabels);
      if (parsed && typeof parsed === "object") {
        buttonLabels = { ...DEFAULT_BUTTON_LABELS, ...parsed };
      }
    }
    const storedOffsets = localStorage.getItem("arm_servo_offsets_v3");
    if (storedOffsets) {
      const parsed = JSON.parse(storedOffsets);
      if (Array.isArray(parsed)) {
        for (let i = 0; i < Math.min(4, parsed.length); i++) {
          servoOffsets[i] = parsed[i];
        }
      }
    }
    const storedClawRange = localStorage.getItem("claw_trim_range");
    if (storedClawRange) {
      const parsed = JSON.parse(storedClawRange);
      if (parsed && typeof parsed.min === "number" && typeof parsed.max === "number") {
        clawMin = Math.max(0, Math.min(179, Math.round(parsed.min)));
        clawMax = Math.max(clawMin + 1, Math.min(180, Math.round(parsed.max)));
        draftClawMin = clawMin;
        draftClawMax = clawMax;
      }
    }
    const storedRange = localStorage.getItem("scissor_lift_range");
    if (storedRange) {
      const parsed = JSON.parse(storedRange);
      if (parsed && typeof parsed.min === "number" && typeof parsed.max === "number") {
        scissorMin = Math.max(0, Math.min(179, Math.round(parsed.min)));
        scissorMax = Math.max(scissorMin + 1, Math.min(180, Math.round(parsed.max)));
        draftMin = scissorMin;
        draftMax = scissorMax;
      }
    }
  } catch (e) {}

  scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);
  applyAutoIdleState(autoIdleEnabled);
  updateEditModeUI();
  updateAllServoUI();
}

function percentToAngle(pct, minA, maxA) {
  const clampedPct = Math.max(0, Math.min(100, pct));
  const span = Math.max(1, maxA - minA);
  return Math.round(minA + (clampedPct / 100) * span);
}

function angleToPercent(angle, minA, maxA) {
  const span = Math.max(1, maxA - minA);
  const pct = Math.round(((angle - minA) / span) * 100);
  return Math.max(0, Math.min(100, pct));
}

function toggleEditNamesMode() {
  isEditNamesMode = !isEditNamesMode;
  updateEditModeUI();
  if (isEditNamesMode) {
    showToast("Edit Mode ON: Tap any servo name or preset button to rename", "warn");
  } else {
    closeRenameModal();
    showToast("Edit Mode OFF: Names locked", "info");
  }
}

function updateEditModeUI() {
  const banner = document.getElementById("edit-mode-banner");
  if (banner) {
    banner.style.display = isEditNamesMode ? "flex" : "none";
  }

  document.querySelectorAll(".edit-pill-btn").forEach(btn => {
    if (isEditNamesMode) {
      btn.classList.add("active-edit");
      btn.innerText = "Done";
    } else {
      btn.classList.remove("active-edit");
      btn.innerText = "Edit";
    }
  });

  document.querySelectorAll(".renameable-name, .renameable-btn").forEach(el => {
    if (isEditNamesMode) {
      el.classList.add("edit-target-active");
    } else {
      el.classList.remove("edit-target-active");
    }
  });
}

function onServoNameClick(idx) {
  if (!isEditNamesMode) return;
  openRenameModal("servo", idx);
}

function onQuickButtonClick(btnKey, actionFn) {
  if (isEditNamesMode) {
    openRenameModal("button", btnKey);
    return;
  }
  if (typeof actionFn === "function") {
    actionFn();
  }
}

function openRenameModal(kind, key) {
  const modal = document.getElementById("rename-modal");
  const title = document.getElementById("rename-modal-title");
  const input = document.getElementById("rename-modal-input");
  if (!modal || !title || !input) return;

  pendingRenameTarget = { kind, key };
  if (kind === "servo") {
    title.innerText = `Rename Servo ${key + 1}`;
    input.value = servoNames[key] || DEFAULT_SERVO_NAMES[key];
  } else {
    title.innerText = "Rename Button Label";
    input.value = buttonLabels[key] || DEFAULT_BUTTON_LABELS[key] || "";
  }

  modal.style.display = "flex";
  setTimeout(() => {
    input.focus();
    input.select();
  }, 40);
}

function closeRenameModal() {
  const modal = document.getElementById("rename-modal");
  if (modal) modal.style.display = "none";
  pendingRenameTarget = null;
}

function confirmRenameModal() {
  if (!pendingRenameTarget) {
    closeRenameModal();
    return;
  }
  const input = document.getElementById("rename-modal-input");
  const val = (input?.value || "").trim().substring(0, 28);
  if (!val) {
    showToast("Name cannot be empty", "warn");
    return;
  }

  if (pendingRenameTarget.kind === "servo") {
    const idx = pendingRenameTarget.key;
    servoNames[idx] = val;
    try { localStorage.setItem("arm_servo_names_v3", JSON.stringify(servoNames)); } catch (e) {}
  } else {
    const bKey = pendingRenameTarget.key;
    buttonLabels[bKey] = val;
    try { localStorage.setItem("arm_button_labels_v1", JSON.stringify(buttonLabels)); } catch (e) {}
  }

  closeRenameModal();
  updateAllServoUI();
  showToast("Name updated", "success");
}

function resetCurrentRenameItem() {
  if (!pendingRenameTarget) {
    closeRenameModal();
    return;
  }
  if (pendingRenameTarget.kind === "servo") {
    const idx = pendingRenameTarget.key;
    servoNames[idx] = DEFAULT_SERVO_NAMES[idx];
    try { localStorage.setItem("arm_servo_names_v3", JSON.stringify(servoNames)); } catch (e) {}
  } else {
    const bKey = pendingRenameTarget.key;
    buttonLabels[bKey] = DEFAULT_BUTTON_LABELS[bKey];
    try { localStorage.setItem("arm_button_labels_v1", JSON.stringify(buttonLabels)); } catch (e) {}
  }
  closeRenameModal();
  updateAllServoUI();
  showToast("Restored default name", "info");
}

function updateServoStateBadges() {
  for (let i = 0; i < 4; i++) {
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

function updatePinDisplay() {
  const p1Lbl = document.getElementById("s1-pin-label");
  const p2Lbl = document.getElementById("s2-pin-label");
  const p3Lbl = document.getElementById("s3-pin-label");
  const p4Lbl = document.getElementById("s4-pin-label");

  if (p1Lbl) p1Lbl.innerText = `GP${servoPins[0]}`;
  if (p2Lbl) p2Lbl.innerText = `GP${servoPins[1]}`;
  if (p3Lbl) p3Lbl.innerText = `GP${servoPins[2]}`;
  if (p4Lbl) p4Lbl.innerText = `GP${servoPins[3]}`;

  const inp1 = document.getElementById("pin-s1-input");
  const inp2 = document.getElementById("pin-s2-input");
  const inp3 = document.getElementById("pin-s3-input");
  const inp4 = document.getElementById("pin-s4-input");

  if (inp1) inp1.value = servoPins[0];
  if (inp2) inp2.value = servoPins[1];
  if (inp3) inp3.value = servoPins[2];
  if (inp4) inp4.value = servoPins[3];
}

function updateClawTrimUI() {
  const rangeBadge = document.getElementById("claw-range-badge");
  const trimBadge = document.getElementById("claw-trim-sync-badge");
  if (rangeBadge) rangeBadge.innerText = `Range: ${clawMin}° – ${clawMax}°`;

  const qMin = document.getElementById("claw-quick-min-lbl");
  const qMid = document.getElementById("claw-quick-mid-lbl");
  const qMax = document.getElementById("claw-quick-max-lbl");
  const midAngle = Math.round((clawMin + clawMax) / 2);
  if (qMin) qMin.innerText = `${buttonLabels.clawMin || "Close"} (${clawMin}°)`;
  if (qMid) qMid.innerText = `${buttonLabels.clawMid || "Mid"} (${midAngle}°)`;
  if (qMax) qMax.innerText = `${buttonLabels.clawMax || "Open"} (${clawMax}°)`;

  const minValEl = document.getElementById("claw-min-trim-val");
  const maxValEl = document.getElementById("claw-max-trim-val");
  const minSl = document.getElementById("claw-min-trim-slider");
  const maxSl = document.getElementById("claw-max-trim-slider");

  if (minValEl) minValEl.innerText = `${draftClawMin}°`;
  if (maxValEl) maxValEl.innerText = `${draftClawMax}°`;
  if (minSl) minSl.value = draftClawMin;
  if (maxSl) maxSl.value = draftClawMax;

  if (trimBadge) {
    const dirty = (draftClawMin !== clawMin) || (draftClawMax !== clawMax);
    if (dirty) {
      trimBadge.innerText = "Unsaved";
      trimBadge.className = "chip chip-warn";
    } else {
      trimBadge.innerText = "Saved";
      trimBadge.className = "chip chip-ok";
    }
  }
}

function updateScissorUI() {
  const liftSl = document.getElementById("lift-slider");
  const liftVal = document.getElementById("lift-val");
  const rangeBadge = document.getElementById("saved-range-badge");
  const trimBadge = document.getElementById("trim-sync-badge");

  if (liftSl && !isDraggingLift) liftSl.value = scissorPercent;
  if (liftVal) liftVal.innerText = `${scissorPercent}% (${servoAngles[3]}°)`;
  if (rangeBadge) rangeBadge.innerText = `Range: ${scissorMin}° – ${scissorMax}°`;

  const qMin = document.getElementById("quick-min-lbl");
  const qMid = document.getElementById("quick-mid-lbl");
  const qMax = document.getElementById("quick-max-lbl");
  const midAngle = Math.round((scissorMin + scissorMax) / 2);
  if (qMin) qMin.innerText = `${buttonLabels.scissorMin || "Min"} (${scissorMin}°)`;
  if (qMid) qMid.innerText = `${buttonLabels.scissorMid || "Mid"} (${midAngle}°)`;
  if (qMax) qMax.innerText = `${buttonLabels.scissorMax || "Max"} (${scissorMax}°)`;

  const minValEl = document.getElementById("min-trim-val");
  const maxValEl = document.getElementById("max-trim-val");
  const minSl = document.getElementById("min-trim-slider");
  const maxSl = document.getElementById("max-trim-slider");

  if (minValEl) minValEl.innerText = `${draftMin}°`;
  if (maxValEl) maxValEl.innerText = `${draftMax}°`;
  if (minSl) minSl.value = draftMin;
  if (maxSl) maxSl.value = draftMax;

  if (trimBadge) {
    const dirty = (draftMin !== scissorMin) || (draftMax !== scissorMax);
    if (dirty) {
      trimBadge.innerText = "Unsaved";
      trimBadge.className = "chip chip-warn";
    } else {
      trimBadge.innerText = "Saved";
      trimBadge.className = "chip chip-ok";
    }
  }
}

function updateQuickButtonsLabels() {
  const homeLbl = document.getElementById("quick-home-lbl");
  const detachLbl = document.getElementById("quick-detach-lbl");
  const syncLbl = document.getElementById("quick-sync-lbl");
  const stopLbl = document.getElementById("quick-stop-lbl");

  if (homeLbl) homeLbl.innerText = buttonLabels.home || DEFAULT_BUTTON_LABELS.home;
  if (detachLbl) detachLbl.innerText = buttonLabels.detach || DEFAULT_BUTTON_LABELS.detach;
  if (syncLbl) syncLbl.innerText = buttonLabels.sync || DEFAULT_BUTTON_LABELS.sync;
  if (stopLbl) stopLbl.innerText = buttonLabels.stop || DEFAULT_BUTTON_LABELS.stop;

  const s4Hdr = document.getElementById("s4-header");
  if (s4Hdr) s4Hdr.innerText = servoNames[3] || DEFAULT_SERVO_NAMES[3];
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
      const minBound = (i === 0) ? clawMin : 0;
      const maxBound = (i === 0) ? clawMax : 180;
      if (i === 0) {
        servoAngles[0] = Math.max(minBound, Math.min(maxBound, servoAngles[0]));
      }
      if (sl) {
        sl.classList.remove("offset-slider");
        sl.min = String(minBound);
        sl.max = String(maxBound);
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

  updateQuickButtonsLabels();
  updateClawTrimUI();
  updateScissorUI();
  updateServoStateBadges();
  updatePinDisplay();
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
    try { localStorage.setItem("arm_servo_offsets_v3", JSON.stringify(servoOffsets)); } catch (e) {}
  } else {
    const clamped = (idx === 0) ? Math.max(clawMin, Math.min(clawMax, parsed)) : parsed;
    servoAngles[idx] = clamped;
    const valEl = document.getElementById(`s${n}-val`);
    if (valEl) valEl.innerText = `${clamped}°`;
    pendingServoCmds[`s${n}`] = `s${n} ${clamped}`;
  }

  triggerThrottledSend();
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
    try { localStorage.setItem("arm_servo_offsets_v3", JSON.stringify(servoOffsets)); } catch (e) {}
  } else {
    delete pendingServoCmds[`s${n}`];
    const clamped = (idx === 0) ? Math.max(clawMin, Math.min(clawMax, parsed)) : parsed;
    servoAngles[idx] = clamped;
    sendCommand(`s${n} ${clamped}`);
  }
}

// Claw (S1) Both-Sides End-Stop Trimming Handlers
function moveClawToMin() {
  lastSliderEditMs[0] = Date.now();
  servoAngles[0] = clawMin;
  servoEnabled[0] = true;
  updateAllServoUI();
  sendCommand(`s1 ${clawMin}`);
  showToast(`Claw to Closed Min (${clawMin}°)`, "info");
}

function moveClawToMid() {
  lastSliderEditMs[0] = Date.now();
  const mid = Math.round((clawMin + clawMax) / 2);
  servoAngles[0] = mid;
  servoEnabled[0] = true;
  updateAllServoUI();
  sendCommand(`s1 ${mid}`);
  showToast(`Claw to Mid (${mid}°)`, "info");
}

function moveClawToMax() {
  lastSliderEditMs[0] = Date.now();
  servoAngles[0] = clawMax;
  servoEnabled[0] = true;
  updateAllServoUI();
  sendCommand(`s1 ${clawMax}`);
  showToast(`Claw to Open Max (${clawMax}°)`, "info");
}

function onClawMinTrimSlider(val) {
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  setClawMinTrimAngle(parsed, true);
}

function nudgeClawMinTrim(delta) {
  setClawMinTrimAngle(draftClawMin + delta, false);
}

function setClawMinTrimAngle(target, throttled) {
  lastSliderEditMs[0] = Date.now();
  let next = Math.max(0, Math.min(179, target));
  if (next >= draftClawMax) {
    next = draftClawMax - 1;
    showToast("Closed side must be below open side", "warn");
  }
  draftClawMin = next;
  servoAngles[0] = next;
  servoEnabled[0] = true;
  updateClawTrimUI();
  updateServoStateBadges();

  if (throttled) {
    pendingServoCmds["claw_test"] = `claw_test ${next}`;
    triggerThrottledSend();
  } else {
    delete pendingServoCmds["claw_test"];
    sendCommand(`claw_test ${next}`);
  }
}

function onClawMaxTrimSlider(val) {
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  setClawMaxTrimAngle(parsed, true);
}

function nudgeClawMaxTrim(delta) {
  setClawMaxTrimAngle(draftClawMax + delta, false);
}

function setClawMaxTrimAngle(target, throttled) {
  lastSliderEditMs[0] = Date.now();
  let next = Math.max(1, Math.min(180, target));
  if (next <= draftClawMin) {
    next = draftClawMin + 1;
    showToast("Open side must be above closed side", "warn");
  }
  draftClawMax = next;
  servoAngles[0] = next;
  servoEnabled[0] = true;
  updateClawTrimUI();
  updateServoStateBadges();

  if (throttled) {
    pendingServoCmds["claw_test"] = `claw_test ${next}`;
    triggerThrottledSend();
  } else {
    delete pendingServoCmds["claw_test"];
    sendCommand(`claw_test ${next}`);
  }
}

function testClawTrimEnd(which) {
  lastSliderEditMs[0] = Date.now();
  const target = (which === "min") ? draftClawMin : draftClawMax;
  servoAngles[0] = target;
  servoEnabled[0] = true;
  updateClawTrimUI();
  updateServoStateBadges();
  sendCommand(`claw_test ${target}`);
}

async function saveClawRange() {
  const btn = document.getElementById("save-claw-range-btn");
  const txt = document.getElementById("save-claw-range-text");
  if (txt) txt.innerText = "Saving...";
  if (btn) btn.style.background = "#0284c7";

  clawMin = draftClawMin;
  clawMax = draftClawMax;
  pendingClawRangeSync = false;
  servoAngles[0] = Math.max(clawMin, Math.min(clawMax, servoAngles[0]));

  try {
    localStorage.setItem("claw_trim_range", JSON.stringify({ min: clawMin, max: clawMax }));
  } catch (e) {}

  await sendCommand(`claw_range_set ${clawMin} ${clawMax}`);
  updateAllServoUI();

  appendLog("Calibration", `Saved Claw Range: ${clawMin}° – ${clawMax}°`);
  showToast(`Saved Claw Trim (${clawMin}°–${clawMax}°) to Flash!`, "success");

  if (txt) txt.innerText = "Saved to Flash";
  if (btn) btn.style.background = "#059669";
  setTimeout(() => {
    if (txt) txt.innerText = "Save Claw Trim";
    if (btn) btn.style.background = "#10b981";
  }, 1500);
}

async function resetClawRange() {
  draftClawMin = 0;
  draftClawMax = 180;
  clawMin = 0;
  clawMax = 180;
  pendingClawRangeSync = false;

  try {
    localStorage.setItem("claw_trim_range", JSON.stringify({ min: 0, max: 180 }));
  } catch (e) {}

  await sendCommand("claw_range_set 0 180");
  updateAllServoUI();
  appendLog("Calibration", "Reset Claw Range to 0° – 180°");
  showToast("Claw range reset to 0°–180°", "info");
}

// Scissor Lift UI Handlers (Servo 4 / Default GPIO 12)
function onLiftSliderStart() {
  isDraggingLift = true;
  lastSliderEditMs[3] = Date.now();
}

function onLiftSlider(pctVal) {
  const parsedPct = parseInt(pctVal, 10);
  if (isNaN(parsedPct)) return;

  lastSliderEditMs[3] = Date.now();
  servoEnabled[3] = true;
  scissorPercent = Math.max(0, Math.min(100, parsedPct));
  servoAngles[3] = percentToAngle(scissorPercent, scissorMin, scissorMax);

  const valEl = document.getElementById("lift-val");
  if (valEl) valEl.innerText = `${scissorPercent}% (${servoAngles[3]}°)`;
  updateServoStateBadges();

  pendingServoCmds["s4"] = `s4 ${servoAngles[3]}`;
  triggerThrottledSend();
}

function onLiftSliderCommit(pctVal) {
  isDraggingLift = false;
  lastSliderEditMs[3] = Date.now();
  const parsedPct = parseInt(pctVal, 10);
  if (isNaN(parsedPct)) return;

  delete pendingServoCmds["s4"];
  scissorPercent = Math.max(0, Math.min(100, parsedPct));
  servoAngles[3] = percentToAngle(scissorPercent, scissorMin, scissorMax);

  const valEl = document.getElementById("lift-val");
  if (valEl) valEl.innerText = `${scissorPercent}% (${servoAngles[3]}°)`;
  sendCommand(`s4 ${servoAngles[3]}`);
}

function nudgeLiftPercent(deltaPct) {
  lastSliderEditMs[3] = Date.now();
  scissorPercent = Math.max(0, Math.min(100, scissorPercent + deltaPct));
  servoAngles[3] = percentToAngle(scissorPercent, scissorMin, scissorMax);
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand(`s4 ${servoAngles[3]}`);
}

function moveToSavedMin() {
  lastSliderEditMs[3] = Date.now();
  servoAngles[3] = scissorMin;
  scissorPercent = 0;
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand("min");
  showToast(`Scissor to Min (${scissorMin}°)`, "info");
}

function moveToSavedMid() {
  lastSliderEditMs[3] = Date.now();
  const mid = Math.round((scissorMin + scissorMax) / 2);
  servoAngles[3] = mid;
  scissorPercent = 50;
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand("mid");
  showToast(`Scissor to Mid (${mid}°)`, "info");
}

function moveToSavedMax() {
  lastSliderEditMs[3] = Date.now();
  servoAngles[3] = scissorMax;
  scissorPercent = 100;
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand("max");
  showToast(`Scissor to Max (${scissorMax}°)`, "info");
}

function onMinTrimSlider(val) {
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  setMinTrimAngle(parsed, true);
}

function nudgeMinTrim(delta) {
  setMinTrimAngle(draftMin + delta, false);
}

function setMinTrimAngle(target, throttled) {
  lastSliderEditMs[3] = Date.now();
  let next = Math.max(0, Math.min(179, target));
  if (next >= draftMax) {
    next = draftMax - 1;
    showToast("Bottom end must be below top end", "warn");
  }
  draftMin = next;
  servoAngles[3] = next;
  scissorPercent = 0;
  servoEnabled[3] = true;
  updateAllServoUI();

  if (throttled) {
    pendingServoCmds["scissor_test"] = `scissor_test ${next}`;
    triggerThrottledSend();
  } else {
    delete pendingServoCmds["scissor_test"];
    sendCommand(`scissor_test ${next}`);
  }
}

function onMaxTrimSlider(val) {
  const parsed = parseInt(val, 10);
  if (isNaN(parsed)) return;
  setMaxTrimAngle(parsed, true);
}

function nudgeMaxTrim(delta) {
  setMaxTrimAngle(draftMax + delta, false);
}

function setMaxTrimAngle(target, throttled) {
  lastSliderEditMs[3] = Date.now();
  let next = Math.max(1, Math.min(180, target));
  if (next <= draftMin) {
    next = draftMin + 1;
    showToast("Top end must be above bottom end", "warn");
  }
  draftMax = next;
  servoAngles[3] = next;
  scissorPercent = 100;
  servoEnabled[3] = true;
  updateAllServoUI();

  if (throttled) {
    pendingServoCmds["scissor_test"] = `scissor_test ${next}`;
    triggerThrottledSend();
  } else {
    delete pendingServoCmds["scissor_test"];
    sendCommand(`scissor_test ${next}`);
  }
}

function testTrimEnd(which) {
  lastSliderEditMs[3] = Date.now();
  const target = (which === "min") ? draftMin : draftMax;
  servoAngles[3] = target;
  scissorPercent = (which === "min") ? 0 : 100;
  servoEnabled[3] = true;
  updateAllServoUI();
  sendCommand(`scissor_test ${target}`);
}

async function saveScissorRange() {
  const btn = document.getElementById("save-scissor-range-btn");
  const txt = document.getElementById("save-scissor-range-text");
  if (txt) txt.innerText = "Saving...";
  if (btn) btn.style.background = "#0284c7";

  scissorMin = draftMin;
  scissorMax = draftMax;
  pendingScissorRangeSync = false;
  scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);

  try {
    localStorage.setItem("scissor_lift_range", JSON.stringify({ min: scissorMin, max: scissorMax }));
  } catch (e) {}

  await sendCommand(`range_set ${scissorMin} ${scissorMax}`);
  updateAllServoUI();

  appendLog("Calibration", `Saved Scissor Lift Range: ${scissorMin}° – ${scissorMax}°`);
  showToast(`Saved Scissor Trim (${scissorMin}°–${scissorMax}°) to Flash!`, "success");

  if (txt) txt.innerText = "Saved to Flash";
  if (btn) btn.style.background = "#059669";
  setTimeout(() => {
    if (txt) txt.innerText = "Save Trim to Flash";
    if (btn) btn.style.background = "#10b981";
  }, 1500);
}

async function resetScissorRange() {
  draftMin = 0;
  draftMax = 180;
  scissorMin = 0;
  scissorMax = 180;
  pendingScissorRangeSync = false;
  scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);

  try {
    localStorage.setItem("scissor_lift_range", JSON.stringify({ min: 0, max: 180 }));
  } catch (e) {}

  await sendCommand("range_set 0 180");
  updateAllServoUI();
  appendLog("Calibration", "Reset Scissor Lift Range to 0° – 180°");
  showToast("Scissor range reset to 0°–180°", "info");
}

function triggerThrottledSend() {
  if (!servoThrottleTimer) {
    servoThrottleTimer = setTimeout(() => {
      servoThrottleTimer = null;
      const cmds = pendingServoCmds;
      pendingServoCmds = {};
      for (const k in cmds) {
        sendCommand(cmds[k]);
      }
    }, 50);
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
  const minBound = (idx === 0) ? clawMin : 0;
  const maxBound = (idx === 0) ? clawMax : 180;
  let next = (servoAngles[idx] || 90) + delta;
  if (next < minBound) next = minBound;
  if (next > maxBound) next = maxBound;
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
  try { localStorage.setItem("arm_servo_offsets_v3", JSON.stringify(servoOffsets)); } catch (e) {}
}

async function saveCalibrationOffsets() {
  const btn = document.getElementById("save-cal-btn");
  const txt = document.getElementById("save-cal-text");
  if (txt) txt.innerText = "Saving...";
  if (btn) btn.style.background = "#0284c7";

  try {
    localStorage.setItem("arm_servo_offsets_v3", JSON.stringify(servoOffsets));
  } catch (e) {}

  await sendCommand(`cal_set ${servoOffsets[0]} ${servoOffsets[1]} ${servoOffsets[2]} ${servoOffsets[3]}`);
  if (isOffsetMode) toggleOffsetMode();
  else updateAllServoUI();

  appendLog("Calibration", `Saved Arm Trim: S1=${servoOffsets[0]}, S2=${servoOffsets[1]}, S3=${servoOffsets[2]}, S4=${servoOffsets[3]}`);
  showToast("Trim offsets saved to ESP32 Flash!", "success");

  if (txt) txt.innerText = "Saved to Flash";
  if (btn) btn.style.background = "#059669";
  setTimeout(() => {
    if (txt) txt.innerText = "Save Trim to Flash";
    if (btn) btn.style.background = "#10b981";
  }, 1500);
}

function homeAllServos() {
  const now = Date.now();
  for (let i = 0; i < 4; i++) lastSliderEditMs[i] = now;
  const clawHome = Math.max(clawMin, Math.min(clawMax, 90));
  const scisHome = Math.max(scissorMin, Math.min(scissorMax, 90));
  servoAngles = [clawHome, 90, 90, scisHome];
  servoEnabled = [true, true, true, true];
  scissorPercent = angleToPercent(scisHome, scissorMin, scissorMax);
  if (isOffsetMode) toggleOffsetMode();
  else updateAllServoUI();
  sendCommand("home");
  showToast("Homing all 4 servos to 90°", "info");
}

function detachAllServos() {
  servoEnabled = [false, false, false, false];
  updateServoStateBadges();
  sendCommand("detach");
  showToast("All servos detached (Idle)", "warn");
}

function requestCalibrationSync() {
  markPendingRangeSync();
  sendCommand("cal_get");
  showToast("Fetching trim offsets, claw & scissor ranges...", "info");
}

function requestStatusSync() {
  markPendingRangeSync();
  sendCommand("status");
  sendCommand("mpu_get");
  showToast("Syncing arm & scissor state...", "info");
}

function requestPinSync() {
  sendCommand("pins");
  showToast("Fetching hardware pin configuration...", "info");
}

function saveHardwarePins() {
  const p1 = parseInt(document.getElementById("pin-s1-input")?.value, 10);
  const p2 = parseInt(document.getElementById("pin-s2-input")?.value, 10);
  const p3 = parseInt(document.getElementById("pin-s3-input")?.value, 10);
  const p4 = parseInt(document.getElementById("pin-s4-input")?.value, 10);

  if (isNaN(p1) || isNaN(p2) || isNaN(p3) || isNaN(p4)) {
    showToast("Invalid GPIO numbers entered", "error");
    return;
  }

  servoPins = [p1, p2, p3, p4];
  updatePinDisplay();
  sendCommand(`pins ${p1} ${p2} ${p3} ${p4}`);
  appendLog("Config", `Saved Hardware Pins: S1=${p1}, S2=${p2}, S3=${p3}, S4=${p4}`);
  showToast(`Hardware pins applied: S1=${p1}, S2=${p2}, S3=${p3}, S4=${p4}`, "success");
}

function applyReceivedPins(p1, p2, p3, p4) {
  if (isNaN(p1) || isNaN(p2) || isNaN(p3) || isNaN(p4)) return;
  servoPins = [p1, p2, p3, p4];
  updatePinDisplay();
}

function applyReceivedAngles(s1, s2, s3, s4, forceSync = false) {
  const incoming = [s1, s2, s3, (s4 !== undefined ? s4 : servoAngles[3])];
  const now = Date.now();

  for (let i = 0; i < 4; i++) {
    if (isNaN(incoming[i])) continue;
    if (!forceSync && (isDraggingSlider[i] || (i === 3 && isDraggingLift) || (now - lastSliderEditMs[i]) < 800)) {
      continue;
    }
    const clampedVal = (i === 0) ? Math.max(clawMin, Math.min(clawMax, incoming[i])) : incoming[i];
    servoAngles[i] = clampedVal;
    const n = i + 1;
    if (i < 3 && !isOffsetMode) {
      const sl = document.getElementById(`s${n}-slider`);
      if (sl) sl.value = clampedVal;
      const lbl = document.getElementById(`s${n}-val`);
      if (lbl) lbl.innerText = `${clampedVal}°`;
    }
  }

  if (!isDraggingLift && (now - lastSliderEditMs[3]) >= 800) {
    scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);
  }
  updateScissorUI();
}

function applyReceivedOffsets(o1, o2, o3, o4, forceSync = false) {
  const incoming = [o1, o2, o3, (o4 !== undefined ? o4 : servoOffsets[3])];
  const now = Date.now();
  let changed = false;

  for (let i = 0; i < 4; i++) {
    if (isNaN(incoming[i])) continue;
    if (!forceSync && (isDraggingSlider[i] || (now - lastSliderEditMs[i]) < 800)) {
      continue;
    }
    if (servoOffsets[i] !== incoming[i]) {
      servoOffsets[i] = incoming[i];
      changed = true;
    }
    const n = i + 1;
    const offEl = document.getElementById(`o${n}-val`);
    if (offEl) offEl.innerText = `${incoming[i] > 0 ? "+" : ""}${incoming[i]}°`;
  }
  if (changed && isOffsetMode) updateAllServoUI();
}

function applyReceivedClawRange(minA, maxA, forceDraft = false) {
  if (isNaN(minA) || isNaN(maxA)) return;
  const nextMin = Math.max(0, Math.min(179, minA));
  const nextMax = Math.max(nextMin + 1, Math.min(180, maxA));
  const savedChanged = (nextMin !== clawMin) || (nextMax !== clawMax);
  const hasUnsavedDraft = (draftClawMin !== clawMin) || (draftClawMax !== clawMax);

  if (!savedChanged && !forceDraft && !pendingClawRangeSync) {
    return;
  }

  clawMin = nextMin;
  clawMax = nextMax;

  if (forceDraft || pendingClawRangeSync || !hasUnsavedDraft) {
    draftClawMin = clawMin;
    draftClawMax = clawMax;
    pendingClawRangeSync = false;
  }

  try {
    localStorage.setItem("claw_trim_range", JSON.stringify({ min: clawMin, max: clawMax }));
  } catch (e) {}
  updateAllServoUI();
}

function applyReceivedRange(minA, maxA, forceDraft = false) {
  if (isNaN(minA) || isNaN(maxA)) return;
  const nextMin = Math.max(0, Math.min(179, minA));
  const nextMax = Math.max(nextMin + 1, Math.min(180, maxA));
  const savedChanged = (nextMin !== scissorMin) || (nextMax !== scissorMax);
  const hasUnsavedDraft = (draftMin !== scissorMin) || (draftMax !== scissorMax);

  if (!savedChanged && !forceDraft && !pendingScissorRangeSync) {
    return;
  }

  scissorMin = nextMin;
  scissorMax = nextMax;

  if (forceDraft || pendingScissorRangeSync || !hasUnsavedDraft) {
    draftMin = scissorMin;
    draftMax = scissorMax;
    pendingScissorRangeSync = false;
  }

  scissorPercent = angleToPercent(servoAngles[3], scissorMin, scissorMax);
  try {
    localStorage.setItem("scissor_lift_range", JSON.stringify({ min: scissorMin, max: scissorMax }));
  } catch (e) {}
  updateScissorUI();
}

function applyReceivedEnableStates(e1, e2, e3, e4) {
  servoEnabled = [!!e1, !!e2, !!e3, (e4 !== undefined ? !!e4 : servoEnabled[3])];
  updateServoStateBadges();
}