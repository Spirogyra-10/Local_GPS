(() => {
  "use strict";

  // ---------------------------------------------------------------------------
  // DOM References
  // ---------------------------------------------------------------------------
  const mapContainer = document.getElementById("mapContainer");
  const mapCanvas = document.getElementById("mapCanvas");
  const mapCtx = mapCanvas.getContext("2d");

  const chartContainer = document.getElementById("chartContainer");
  const chartCanvas = document.getElementById("distChartCanvas");
  const chartCtx = chartCanvas.getContext("2d");

  const connBadge = document.getElementById("connBadge");
  const connText = document.getElementById("connText");
  const modeBadge = document.getElementById("modeBadge");
  const hzBadge = document.getElementById("hzBadge");
  const logChip = document.getElementById("logChip");
  const btnTheme = document.getElementById("btnTheme");

  const eventToast = document.getElementById("eventToast");
  const btnAutoScale = document.getElementById("btnAutoScale");
  const btnZoomOut = document.getElementById("btnZoomOut");
  const btnZoomIn = document.getElementById("btnZoomIn");
  const btnFit = document.getElementById("btnFit");

  const scaleBar = document.getElementById("scaleBar");
  const scaleBarLabel = document.getElementById("scaleBarLabel");
  const viewportRangeText = document.getElementById("viewportRangeText");

  const heroQuality = document.getElementById("heroQuality");
  const posXEl = document.getElementById("posX");
  const posYEl = document.getElementById("posY");
  const metricRangeEl = document.getElementById("metricRange");
  const metricAngleEl = document.getElementById("metricAngle");

  const distLabels = [
    document.getElementById("distLabel1"),
    document.getElementById("distLabel2"),
    document.getElementById("distLabel3"),
  ];
  const distVals = [
    document.getElementById("distVal1"),
    document.getElementById("distVal2"),
    document.getElementById("distVal3"),
  ];
  const distBars = [
    document.getElementById("distBar1"),
    document.getElementById("distBar2"),
    document.getElementById("distBar3"),
  ];

  const chartRangeBadge = document.getElementById("chartRangeBadge");
  const mappingBadge = document.getElementById("mappingBadge");
  const calibText = document.getElementById("calibText");

  const btnRecalib = document.getElementById("btnRecalib");
  const btnSwap = document.getElementById("btnSwap");
  const btnClearTrail = document.getElementById("btnClearTrail");

  const chkAutoScale = document.getElementById("chkAutoScale");
  const chkCircles = document.getElementById("chkCircles");
  const chkDistLines = document.getElementById("chkDistLines");
  const chkRangeRings = document.getElementById("chkRangeRings");
  const chkFilter = document.getElementById("chkFilter");

  const sliderHistory = document.getElementById("sliderHistory");
  const historyVal = document.getElementById("historyVal");

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  const PAD_LEFT = 58;
  const PAD_RIGHT = 24;
  const PAD_TOP = 24;
  const PAD_BOTTOM = 42;

  const COLORS = {
    s1: "#f43f5e",
    s2: "#f59e0b",
    s3: "#a855f7",
    anchorFill: "#2563eb",
    anchorStroke: "#93c5fd",
    robot: "#22c55e",
    robotStroke: "#14532d",
    traj: [56, 189, 248],
  };

  let mapCssW = 640;
  let mapCssH = 480;
  let chartCssW = 340;
  let chartCssH = 148;

  let latestState = null;
  let sseAlive = false;
  let lastSeenEvent = "";
  let toastTimer = null;

  const MAX_DIST_HISTORY = 300;
  const distHistory = [];

  // View & auto-scale state
  let autoScale = true;
  let showCircles = true;
  let showDistLines = true;
  let showRangeRings = true;

  let viewInitialized = false;
  let currentView = { cx: 1.0, cy: 1.0, span: 4.0 };
  let targetView = { cx: 1.0, cy: 1.0, span: 4.0 };
  let shrinkHoldTimer = 0.0;
  let chartMaxY = 4.0;

  // Pointer drag pan state
  let isDragging = false;
  let dragStartX = 0;
  let dragStartY = 0;
  let dragStartCx = 0;
  let dragStartCy = 0;

  // ---------------------------------------------------------------------------
  // Theme Handling
  // ---------------------------------------------------------------------------
  function applyTheme(theme) {
    document.documentElement.setAttribute("data-theme", theme);
    btnTheme.textContent = theme === "dark" ? "☀ Light" : "🌙 Dark";
    try {
      localStorage.setItem("local_gps_theme", theme);
    } catch (_) {}
  }

  (function initTheme() {
    let saved = "dark";
    try {
      const val = localStorage.getItem("local_gps_theme");
      if (val === "light" || val === "dark") saved = val;
    } catch (_) {}
    applyTheme(saved);
  })();

  btnTheme.addEventListener("click", () => {
    const cur = document.documentElement.getAttribute("data-theme") || "dark";
    applyTheme(cur === "dark" ? "light" : "dark");
  });

  // ---------------------------------------------------------------------------
  // Toast Notifications
  // ---------------------------------------------------------------------------
  function showToast(msg) {
    if (!msg) return;
    eventToast.textContent = msg;
    eventToast.classList.remove("hidden");
    if (toastTimer) clearTimeout(toastTimer);
    toastTimer = setTimeout(() => {
      eventToast.classList.add("hidden");
    }, 2800);
  }

  // ---------------------------------------------------------------------------
  // Canvas HiDPI Resizing
  // ---------------------------------------------------------------------------
  function resizeCanvas(canvas, container, setSize) {
    const rect = container.getBoundingClientRect();
    const w = Math.max(1, Math.floor(rect.width));
    const h = Math.max(1, Math.floor(rect.height));
    const dpr = Math.min(window.devicePixelRatio || 1, 2.5);
    const pw = Math.round(w * dpr);
    const ph = Math.round(h * dpr);
    if (canvas.width !== pw || canvas.height !== ph) {
      canvas.width = pw;
      canvas.height = ph;
    }
    setSize(w, h, dpr);
  }

  let mapDpr = window.devicePixelRatio || 1;
  let chartDpr = window.devicePixelRatio || 1;

  function refreshCanvasSizes() {
    resizeCanvas(mapCanvas, mapContainer, (w, h, dpr) => {
      mapCssW = w;
      mapCssH = h;
      mapDpr = dpr;
    });
    resizeCanvas(chartCanvas, chartContainer, (w, h, dpr) => {
      chartCssW = w;
      chartCssH = h;
      chartDpr = dpr;
    });
  }

  if (typeof ResizeObserver !== "undefined") {
    const ro = new ResizeObserver(refreshCanvasSizes);
    ro.observe(mapContainer);
    ro.observe(chartContainer);
  } else {
    window.addEventListener("resize", refreshCanvasSizes);
  }
  refreshCanvasSizes();

  // ---------------------------------------------------------------------------
  // Math & Nice Ticks Helpers
  // ---------------------------------------------------------------------------
  function niceStep(rawStep) {
    if (!Number.isFinite(rawStep) || rawStep <= 0) return 1.0;
    const exp = Math.floor(Math.log10(rawStep));
    const base = Math.pow(10, exp);
    const frac = rawStep / base;
    let nice;
    if (frac <= 1.2) nice = 1;
    else if (frac <= 2.5) nice = 2;
    else if (frac <= 5.5) nice = 5;
    else nice = 10;
    return nice * base;
  }

  function niceCeil(val) {
    if (!Number.isFinite(val) || val <= 0) return 1.0;
    const exp = Math.floor(Math.log10(val));
    const base = Math.pow(10, exp);
    const frac = val / base;
    let nice;
    if (frac <= 1.0) nice = 1.0;
    else if (frac <= 1.5) nice = 1.5;
    else if (frac <= 2.0) nice = 2.0;
    else if (frac <= 3.0) nice = 3.0;
    else if (frac <= 4.0) nice = 4.0;
    else if (frac <= 5.0) nice = 5.0;
    else if (frac <= 6.0) nice = 6.0;
    else if (frac <= 8.0) nice = 8.0;
    else nice = 10.0;
    return nice * base;
  }

  function formatMeters(val, step) {
    if (Math.abs(val) < 1e-9) return "0";
    if (step !== undefined && step < 0.1) return val.toFixed(2);
    if (step !== undefined && step < 1.0) return val.toFixed(1);
    return Number.isInteger(Math.round(val * 100) / 100) ? String(Math.round(val)) : val.toFixed(1);
  }

  function formatRingLabel(r) {
    if (r < 1.0) return `${r.toFixed(1)}m`;
    if (Math.abs(r - Math.round(r)) < 1e-6) return `${Math.round(r)}m`;
    return `${r.toFixed(1)}m`;
  }

  // ---------------------------------------------------------------------------
  // Coordinate Transforms & Distance-Adaptive Auto-Scale
  // ---------------------------------------------------------------------------
  function getPlotBox() {
    const plotW = mapCssW - PAD_LEFT - PAD_RIGHT;
    const plotH = mapCssH - PAD_TOP - PAD_BOTTOM;
    return { plotW, plotH };
  }

  function worldToScreen(wx, wy, view, plotW, plotH) {
    const pxPerM = Math.min(plotW, plotH) / view.span;
    const sx = PAD_LEFT + plotW * 0.5 + (wx - view.cx) * pxPerM;
    const sy = PAD_TOP + plotH * 0.5 - (wy - view.cy) * pxPerM;
    return [sx, sy];
  }

  function screenToWorld(sx, sy, view, plotW, plotH) {
    const pxPerM = Math.min(plotW, plotH) / view.span;
    const wx = view.cx + (sx - (PAD_LEFT + plotW * 0.5)) / pxPerM;
    const wy = view.cy - (sy - (PAD_TOP + plotH * 0.5)) / pxPerM;
    return [wx, wy];
  }

  function collectKeyPoints(state) {
    const pts = [];
    const anchors = Array.isArray(state?.anchors) && state.anchors.length >= 3
      ? state.anchors
      : [[0, 0], [0, 1], [1, 0]];
    for (const a of anchors) {
      if (Array.isArray(a) && Number.isFinite(a[0]) && Number.isFinite(a[1])) {
        pts.push([a[0], a[1]]);
      }
    }

    const pos = state?.pos;
    const hasPos = Array.isArray(pos) && pos.length >= 2 && Number.isFinite(pos[0]) && Number.isFinite(pos[1]);
    if (hasPos) {
      pts.push([pos[0], pos[1]]);
    }

    const dists = Array.isArray(state?.distances) ? state.distances : [];
    let maxValidDist = 0;
    for (const d of dists) {
      if (Number.isFinite(d) && d > maxValidDist) maxValidDist = d;
    }

    if (!hasPos && maxValidDist > 0) {
      pts.push([maxValidDist * 0.85, maxValidDist * 0.85]);
    }
    return pts;
  }

  function computeIdealView(state, plotW, plotH) {
    const pts = collectKeyPoints(state);
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const [x, y] of pts) {
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
    if (!Number.isFinite(minX)) {
      minX = 0; maxX = 1; minY = 0; maxY = 1;
    }

    const boxW = Math.max(0.2, maxX - minX);
    const boxH = Math.max(0.2, maxY - minY);
    const aspect = plotW / Math.max(1, plotH);
    const padding = 1.45;

    const needSpanX = aspect >= 1 ? (boxW * padding) / aspect : boxW * padding;
    const needSpanY = aspect >= 1 ? boxH * padding : boxH * padding * aspect;

    const d12 = state?.calib?.d12 || 1.0;
    const d13 = state?.calib?.d13 || 1.0;
    const baseMax = Math.max(d12, d13, 0.5);
    const minFloorSpan = Math.max(1.2, baseMax * 1.45);

    const idealSpan = Math.min(80.0, Math.max(minFloorSpan, needSpanX, needSpanY));
    const idealCx = (minX + maxX) * 0.5;
    const idealCy = (minY + maxY) * 0.5;
    return { cx: idealCx, cy: idealCy, span: idealSpan };
  }

  function setAutoScale(enabled, snapNow = false) {
    autoScale = Boolean(enabled);
    chkAutoScale.checked = autoScale;
    btnAutoScale.classList.toggle("active", autoScale);
    btnAutoScale.textContent = autoScale ? "自動スケール: ON" : "自動スケール: OFF";

    if (autoScale && latestState) {
      const { plotW, plotH } = getPlotBox();
      if (plotW > 0 && plotH > 0) {
        const ideal = computeIdealView(latestState, plotW, plotH);
        targetView = { ...ideal };
        shrinkHoldTimer = 0.0;
        if (snapNow) {
          currentView = { ...ideal };
          viewInitialized = true;
        }
      }
    }
  }

  function updateAutoScaleTarget(state, dt, plotW, plotH) {
    if (plotW <= 0 || plotH <= 0) return;
    const ideal = computeIdealView(state, plotW, plotH);

    if (!viewInitialized) {
      currentView = { ...ideal };
      targetView = { ...ideal };
      viewInitialized = true;
      return;
    }

    if (!autoScale) return;

    const pts = collectKeyPoints(state);
    const pxPerM = Math.min(plotW, plotH) / targetView.span;
    const halfW = (plotW / pxPerM) * 0.5;
    const halfH = (plotH / pxPerM) * 0.5;

    let nearEdge = false;
    for (const [x, y] of pts) {
      if (Math.abs(x - targetView.cx) > halfW * 0.70 || Math.abs(y - targetView.cy) > halfH * 0.70) {
        nearEdge = true;
        break;
      }
    }

    if (nearEdge || ideal.span > targetView.span * 1.08) {
      targetView = { ...ideal };
      shrinkHoldTimer = 0.0;
    } else if (ideal.span < targetView.span * 0.78) {
      shrinkHoldTimer += dt;
      if (shrinkHoldTimer >= 0.6) {
        targetView = { ...ideal };
        shrinkHoldTimer = 0.0;
      }
    } else {
      shrinkHoldTimer = 0.0;
      const shift = Math.hypot(ideal.cx - targetView.cx, ideal.cy - targetView.cy);
      if (shift > targetView.span * 0.12) {
        targetView.cx = ideal.cx;
        targetView.cy = ideal.cy;
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Map Pointer Interaction (Zoom & Pan)
  // ---------------------------------------------------------------------------
  mapCanvas.addEventListener(
    "wheel",
    (e) => {
      e.preventDefault();
      const { plotW, plotH } = getPlotBox();
      if (plotW <= 0 || plotH <= 0) return;

      const rect = mapCanvas.getBoundingClientRect();
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;

      if (autoScale) setAutoScale(false);

      const [wx, wy] = screenToWorld(sx, sy, targetView, plotW, plotH);
      const factor = e.deltaY > 0 ? 1.15 : 1 / 1.15;
      const newSpan = Math.min(120.0, Math.max(0.3, targetView.span * factor));

      const newPxPerM = Math.min(plotW, plotH) / newSpan;
      targetView.span = newSpan;
      targetView.cx = wx - (sx - (PAD_LEFT + plotW * 0.5)) / newPxPerM;
      targetView.cy = wy + (sy - (PAD_TOP + plotH * 0.5)) / newPxPerM;
      currentView = { ...targetView };
    },
    { passive: false }
  );

  mapCanvas.addEventListener("pointerdown", (e) => {
    if (e.button !== 0) return;
    isDragging = true;
    dragStartX = e.clientX;
    dragStartY = e.clientY;
    dragStartCx = targetView.cx;
    dragStartCy = targetView.cy;
    mapCanvas.setPointerCapture(e.pointerId);
  });

  mapCanvas.addEventListener("pointermove", (e) => {
    if (!isDragging) return;
    const dx = e.clientX - dragStartX;
    const dy = e.clientY - dragStartY;
    if (Math.hypot(dx, dy) < 2) return;

    const { plotW, plotH } = getPlotBox();
    if (plotW <= 0 || plotH <= 0) return;

    if (autoScale) setAutoScale(false);
    const pxPerM = Math.min(plotW, plotH) / targetView.span;
    targetView.cx = dragStartCx - dx / pxPerM;
    targetView.cy = dragStartCy + dy / pxPerM;
    currentView.cx = targetView.cx;
    currentView.cy = targetView.cy;
  });

  function stopDragging(e) {
    if (!isDragging) return;
    isDragging = false;
    try {
      mapCanvas.releasePointerCapture(e.pointerId);
    } catch (_) {}
  }

  mapCanvas.addEventListener("pointerup", stopDragging);
  mapCanvas.addEventListener("pointercancel", stopDragging);

  mapCanvas.addEventListener("dblclick", () => {
    setAutoScale(true);
    showToast("自動スケールに戻しました");
  });

  function zoomByFactor(factor) {
    if (autoScale) setAutoScale(false);
    targetView.span = Math.min(120.0, Math.max(0.3, targetView.span * factor));
  }

  btnAutoScale.addEventListener("click", () => {
    setAutoScale(!autoScale);
    showToast(autoScale ? "自動スケール: ON" : "自動スケール: OFF");
  });
  chkAutoScale.addEventListener("change", () => {
    setAutoScale(chkAutoScale.checked);
  });
  btnZoomIn.addEventListener("click", () => zoomByFactor(0.75));
  btnZoomOut.addEventListener("click", () => zoomByFactor(1.33));
  btnFit.addEventListener("click", () => {
    setAutoScale(true);
    showToast("全体表示 (自動スケール)");
  });

  // ---------------------------------------------------------------------------
  // Server Actions & Controls
  // ---------------------------------------------------------------------------
  async function sendAction(payload) {
    try {
      const res = await fetch("/api/action", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      if (!res.ok) return;
      const data = await res.json();
      if (data && data.ok && data.state) {
        handleIncomingState(data.state, false);
      }
    } catch (err) {
      console.error("Action failed:", err);
    }
  }

  btnRecalib.addEventListener("click", () => sendAction({ action: "recalib" }));
  btnSwap.addEventListener("click", () => sendAction({ action: "toggle_swap" }));
  btnClearTrail.addEventListener("click", () => sendAction({ action: "clear_trajectory" }));

  chkCircles.addEventListener("change", () => {
    showCircles = chkCircles.checked;
  });
  chkDistLines.addEventListener("change", () => {
    showDistLines = chkDistLines.checked;
  });
  chkRangeRings.addEventListener("change", () => {
    showRangeRings = chkRangeRings.checked;
  });
  chkFilter.addEventListener("change", () => {
    sendAction({ action: "set_filter", enabled: chkFilter.checked });
  });

  let historySendTimer = null;
  sliderHistory.addEventListener("input", () => {
    const val = Number(sliderHistory.value);
    historyVal.textContent = String(val);
    if (historySendTimer) clearTimeout(historySendTimer);
    historySendTimer = setTimeout(() => {
      sendAction({ action: "set_history", history_len: val });
    }, 120);
  });

  window.addEventListener("keydown", (e) => {
    const tag = document.activeElement ? document.activeElement.tagName : "";
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (e.metaKey || e.ctrlKey || e.altKey) return;

    if (e.key === "c" || e.key === "C") {
      sendAction({ action: "recalib" });
    } else if (e.key === "r" || e.key === "R") {
      sendAction({ action: "toggle_swap" });
    } else if (e.key === "a" || e.key === "A") {
      setAutoScale(true);
      showToast("自動スケール: ON");
    } else if (e.key === "x" || e.key === "X") {
      sendAction({ action: "clear_trajectory" });
    }
  });

  // ---------------------------------------------------------------------------
  // DOM Telemetry Update
  // ---------------------------------------------------------------------------
  function updateDomFromState(state) {
    // Connection badge
    connBadge.className = "pill";
    if (!sseAlive) {
      connBadge.classList.add("pill-disconnected");
      connText.textContent = "SERVER RECONNECTING...";
    } else if (state.is_sim) {
      connBadge.classList.add("pill-sim");
      connText.textContent = "SIMULATION";
    } else if (state.connected) {
      connBadge.classList.add("pill-connected");
      connText.textContent = `CONNECTED ${state.port || ""}`.trim();
    } else {
      connBadge.classList.add("pill-disconnected");
      connText.textContent = `DISCONNECTED ${state.port || ""}`.trim();
    }

    // Mode badge
    const mode = state.mode || "ERROR";
    modeBadge.textContent = mode;
    modeBadge.className = "pill pill-mode";
    if (mode === "3_ANCHORS") {
      modeBadge.classList.add("ok");
      heroQuality.textContent = "3点測位 (高精度)";
    } else if (mode.startsWith("FALLBACK_")) {
      modeBadge.classList.add("warn");
      heroQuality.textContent = `2点補完 (${mode.replace("FALLBACK_", "")})`;
    } else {
      modeBadge.classList.add("err");
      heroQuality.textContent = "測距探索中";
    }

    hzBadge.textContent = `${Number(state.hz || 0).toFixed(1)} Hz`;

    if (state.log_file) {
      const parts = state.log_file.split(/[\\/]/);
      const fname = parts[parts.length - 1];
      logChip.textContent = `LOG: ${fname}`;
      logChip.title = state.log_file;
    } else {
      logChip.textContent = "LOG: OFF";
      logChip.title = "";
    }

    // Position Hero
    const pos = state.pos;
    if (Array.isArray(pos) && pos.length >= 2 && Number.isFinite(pos[0]) && Number.isFinite(pos[1])) {
      posXEl.textContent = pos[0].toFixed(3);
      posYEl.textContent = pos[1].toFixed(3);
      const r = Math.hypot(pos[0], pos[1]);
      const deg = (Math.atan2(pos[1], pos[0]) * 180) / Math.PI;
      metricRangeEl.textContent = `${r.toFixed(2)} m`;
      metricAngleEl.textContent = `${deg.toFixed(1)} °`;
    } else {
      posXEl.textContent = "--.---";
      posYEl.textContent = "--.---";
      metricRangeEl.textContent = "--.-- m";
      metricAngleEl.textContent = "---.- °";
    }

    // Anchor Labels & Distances
    const p2Port = state.is_swapped ? "P3" : "P2";
    const p3Port = state.is_swapped ? "P2" : "P3";
    distLabels[0].innerHTML = `<i class="color-dot s1"></i>Slave 1 (原点)`;
    distLabels[1].innerHTML = `<i class="color-dot s2"></i>Slave 2 [${p2Port}]`;
    distLabels[2].innerHTML = `<i class="color-dot s3"></i>Slave 3 [${p3Port}]`;

    const dists = Array.isArray(state.distances) ? state.distances : [-1, -1, -1];
    let maxBarScale = 1.0;
    for (const d of dists) {
      if (Number.isFinite(d) && d > maxBarScale) maxBarScale = d;
    }
    maxBarScale = Math.max(1.0, maxBarScale * 1.15);

    for (let i = 0; i < 3; i++) {
      const d = dists[i];
      if (Number.isFinite(d) && d > 0) {
        distVals[i].textContent = `${d.toFixed(2)} m`;
        distVals[i].classList.remove("lost");
        const pct = Math.max(2, Math.min(100, (d / maxBarScale) * 100));
        distBars[i].style.width = `${pct.toFixed(1)}%`;
      } else {
        distVals[i].textContent = "LOST";
        distVals[i].classList.add("lost");
        distBars[i].style.width = "0%";
      }
    }

    // Calibration & Controls
    mappingBadge.textContent = state.is_swapped ? "SWAPPED (P3→S2, P2→S3)" : "NORMAL (P2→S2, P3→S3)";
    mappingBadge.className = `mapping-badge ${state.is_swapped ? "swapped" : "normal"}`;

    if (state.calib) {
      calibText.textContent =
        `S2(Y): ${Number(state.calib.d12).toFixed(2)} m (Nom: ${Number(state.calib.nominal_d12).toFixed(2)}m)  |  ` +
        `S3(X): ${Number(state.calib.d13).toFixed(2)} m (Nom: ${Number(state.calib.nominal_d13).toFixed(2)}m)`;
    }

    if (typeof state.use_filter === "boolean" && chkFilter.checked !== state.use_filter) {
      chkFilter.checked = state.use_filter;
    }
    if (typeof state.history_len === "number" && document.activeElement !== sliderHistory) {
      sliderHistory.value = String(state.history_len);
      historyVal.textContent = String(state.history_len);
    }

    if (state.last_event && state.last_event !== lastSeenEvent) {
      lastSeenEvent = state.last_event;
      showToast(state.last_event);
    }
  }

  function handleIncomingState(state, pushChart = true) {
    latestState = state;
    if (pushChart && Array.isArray(state.distances) && state.distances.length >= 3) {
      distHistory.push([
        Number(state.distances[0]),
        Number(state.distances[1]),
        Number(state.distances[2]),
      ]);
      if (distHistory.length > MAX_DIST_HISTORY) {
        distHistory.shift();
      }
    }
    updateDomFromState(state);
  }

  // ---------------------------------------------------------------------------
  // SSE Connection
  // ---------------------------------------------------------------------------
  const evtSource = new EventSource("/api/stream");
  evtSource.onopen = () => {
    sseAlive = true;
    if (latestState) updateDomFromState(latestState);
  };
  evtSource.onmessage = (event) => {
    sseAlive = true;
    try {
      const parsed = JSON.parse(event.data);
      handleIncomingState(parsed, true);
    } catch (_) {}
  };
  evtSource.onerror = () => {
    sseAlive = false;
    if (latestState) {
      updateDomFromState(latestState);
    } else {
      connBadge.className = "pill pill-disconnected";
      connText.textContent = "SERVER RECONNECTING...";
    }
  };

  // ---------------------------------------------------------------------------
  // Theme Color Reader for Canvas
  // ---------------------------------------------------------------------------
  function getCssColors() {
    const cs = getComputedStyle(document.documentElement);
    return {
      bg: cs.getPropertyValue("--bg-canvas").trim() || "#090e1a",
      textMain: cs.getPropertyValue("--text-main").trim() || "#f1f5f9",
      textMuted: cs.getPropertyValue("--text-muted").trim() || "#94a3b8",
      border: cs.getPropertyValue("--border").trim() || "#26344b",
      gridMajor: cs.getPropertyValue("--grid-major").trim() || "rgba(148,163,184,0.17)",
      gridMinor: cs.getPropertyValue("--grid-minor").trim() || "rgba(148,163,184,0.06)",
      axisZero: cs.getPropertyValue("--axis-zero").trim() || "rgba(226,232,240,0.36)",
      ringStroke: cs.getPropertyValue("--ring-stroke").trim() || "rgba(56,189,248,0.22)",
      ringText: cs.getPropertyValue("--ring-text").trim() || "rgba(148,163,184,0.72)",
      hudBg: cs.getPropertyValue("--hud-bg").trim() || "rgba(15,23,42,0.84)",
    };
  }

  function drawPillLabel(ctx, text, cx, cy, bgColor, textColor, borderColor = null) {
    ctx.save();
    ctx.font = "600 11px Inter, Noto Sans JP, sans-serif";
    const metrics = ctx.measureText(text);
    const padX = 7;
    const w = metrics.width + padX * 2;
    const h = 20;
    const x = cx - w * 0.5;
    const y = cy - h * 0.5;

    ctx.beginPath();
    if (typeof ctx.roundRect === "function") {
      ctx.roundRect(x, y, w, h, 5);
    } else {
      ctx.rect(x, y, w, h);
    }
    ctx.fillStyle = bgColor;
    ctx.fill();
    if (borderColor) {
      ctx.strokeStyle = borderColor;
      ctx.lineWidth = 1;
      ctx.stroke();
    }
    ctx.fillStyle = textColor;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, cx, cy + 0.5);
    ctx.restore();
  }

  // ---------------------------------------------------------------------------
  // Render 2D Positioning Map
  // ---------------------------------------------------------------------------
  function renderMap(nowMs, dt) {
    const { plotW, plotH } = getPlotBox();
    if (plotW <= 10 || plotH <= 10) return;

    const theme = getCssColors();
    const ctx = mapCtx;

    ctx.save();
    ctx.setTransform(mapDpr, 0, 0, mapDpr, 0, 0);
    ctx.fillStyle = theme.bg;
    ctx.fillRect(0, 0, mapCssW, mapCssH);

    // Update target & smoothly ease current view
    if (latestState) {
      updateAutoScaleTarget(latestState, dt, plotW, plotH);
    }
    const zoomSpeed = targetView.span > currentView.span ? 8.0 : 3.0;
    const zAlpha = 1 - Math.exp(-zoomSpeed * dt);
    const pAlpha = 1 - Math.exp(-6.0 * dt);
    currentView.span += (targetView.span - currentView.span) * zAlpha;
    currentView.cx += (targetView.cx - currentView.cx) * pAlpha;
    currentView.cy += (targetView.cy - currentView.cy) * pAlpha;

    const pxPerM = Math.min(plotW, plotH) / currentView.span;
    const visibleW = plotW / pxPerM;
    const visibleH = plotH / pxPerM;

    const minWorldX = currentView.cx - visibleW * 0.5;
    const maxWorldX = currentView.cx + visibleW * 0.5;
    const minWorldY = currentView.cy - visibleH * 0.5;
    const maxWorldY = currentView.cy + visibleH * 0.5;

    const majorStep = niceStep(80 / pxPerM);
    const minorStep = majorStep / 5;

    // --- Clip to Plot Box ---
    ctx.save();
    ctx.beginPath();
    ctx.rect(PAD_LEFT, PAD_TOP, plotW, plotH);
    ctx.clip();

    // 1. Minor & Major Grid
    ctx.lineWidth = 1;
    ctx.strokeStyle = theme.gridMinor;
    ctx.beginPath();
    const startMinorX = Math.floor(minWorldX / minorStep) * minorStep;
    for (let x = startMinorX; x <= maxWorldX + minorStep * 0.5; x += minorStep) {
      const [sx] = worldToScreen(x, 0, currentView, plotW, plotH);
      ctx.moveTo(sx, PAD_TOP);
      ctx.lineTo(sx, PAD_TOP + plotH);
    }
    const startMinorY = Math.floor(minWorldY / minorStep) * minorStep;
    for (let y = startMinorY; y <= maxWorldY + minorStep * 0.5; y += minorStep) {
      const [, sy] = worldToScreen(0, y, currentView, plotW, plotH);
      ctx.moveTo(PAD_LEFT, sy);
      ctx.lineTo(PAD_LEFT + plotW, sy);
    }
    ctx.stroke();

    ctx.strokeStyle = theme.gridMajor;
    ctx.beginPath();
    const startMajorX = Math.floor(minWorldX / majorStep) * majorStep;
    for (let x = startMajorX; x <= maxWorldX + majorStep * 0.5; x += majorStep) {
      const [sx] = worldToScreen(x, 0, currentView, plotW, plotH);
      ctx.moveTo(sx, PAD_TOP);
      ctx.lineTo(sx, PAD_TOP + plotH);
    }
    const startMajorY = Math.floor(minWorldY / majorStep) * majorStep;
    for (let y = startMajorY; y <= maxWorldY + majorStep * 0.5; y += majorStep) {
      const [, sy] = worldToScreen(0, y, currentView, plotW, plotH);
      ctx.moveTo(PAD_LEFT, sy);
      ctx.lineTo(PAD_LEFT + plotW, sy);
    }
    ctx.stroke();

    // X=0 and Y=0 origin axes
    const [ox, oy] = worldToScreen(0, 0, currentView, plotW, plotH);
    ctx.strokeStyle = theme.axisZero;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(ox, PAD_TOP);
    ctx.lineTo(ox, PAD_TOP + plotH);
    ctx.moveTo(PAD_LEFT, oy);
    ctx.lineTo(PAD_LEFT + plotW, oy);
    ctx.stroke();

    // 2. Concentric Range Rings centered at Slave 1 (0,0) in 1st quadrant
    if (showRangeRings) {
      const ringStep = majorStep;
      const cornerDist = Math.max(
        Math.hypot(minWorldX, minWorldY),
        Math.hypot(maxWorldX, minWorldY),
        Math.hypot(minWorldX, maxWorldY),
        Math.hypot(maxWorldX, maxWorldY)
      );
      const maxRings = Math.min(35, Math.ceil(cornerDist / ringStep));
      ctx.save();
      ctx.strokeStyle = theme.ringStroke;
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 5]);
      ctx.font = "600 10px Inter, Noto Sans JP, sans-serif";
      ctx.fillStyle = theme.ringText;
      ctx.textAlign = "left";
      ctx.textBaseline = "bottom";

      for (let i = 1; i <= maxRings; i++) {
        const r = i * ringStep;
        const rPx = r * pxPerM;
        if (rPx < 24) continue;
        ctx.beginPath();
        ctx.arc(ox, oy, rPx, -Math.PI / 2, 0, false);
        ctx.stroke();

        const ang = -Math.PI / 4;
        const lx = ox + rPx * Math.cos(ang) + 3;
        const ly = oy + rPx * Math.sin(ang) - 2;
        if (lx > PAD_LEFT + 8 && lx < PAD_LEFT + plotW - 28 && ly > PAD_TOP + 12 && ly < PAD_TOP + plotH - 8) {
          ctx.fillText(formatRingLabel(r), lx, ly);
        }
      }
      ctx.restore();
    }

    const state = latestState;
    const anchors = Array.isArray(state?.anchors) && state.anchors.length >= 3
      ? state.anchors
      : [[0, 0], [0, 1], [1, 0]];
    const dists = Array.isArray(state?.distances) ? state.distances : [-1, -1, -1];
    const pos = state?.pos;
    const hasPos = Array.isArray(pos) && pos.length >= 2 && Number.isFinite(pos[0]) && Number.isFinite(pos[1]);

    // 3. Anchor Baseline Legs (S1-S2 & S1-S3)
    const aScreen = anchors.map(([ax, ay]) => worldToScreen(ax, ay, currentView, plotW, plotH));
    ctx.save();
    ctx.strokeStyle = "rgba(59, 130, 246, 0.55)";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(aScreen[0][0], aScreen[0][1]);
    ctx.lineTo(aScreen[1][0], aScreen[1][1]);
    ctx.moveTo(aScreen[0][0], aScreen[0][1]);
    ctx.lineTo(aScreen[2][0], aScreen[2][1]);
    ctx.stroke();
    ctx.restore();

    // 4. Ranging Circles
    if (showCircles) {
      const circleColors = [COLORS.s1, COLORS.s2, COLORS.s3];
      for (let i = 0; i < 3; i++) {
        const d = dists[i];
        if (Number.isFinite(d) && d > 0) {
          ctx.save();
          ctx.strokeStyle = circleColors[i];
          ctx.globalAlpha = 0.52;
          ctx.lineWidth = 1.6;
          ctx.setLineDash([5, 5]);
          ctx.beginPath();
          ctx.arc(aScreen[i][0], aScreen[i][1], d * pxPerM, 0, Math.PI * 2);
          ctx.stroke();
          ctx.restore();
        }
      }
    }

    // 5. Distance Connector Lines (Anchor -> Robot)
    if (showDistLines && hasPos) {
      const [rx, ry] = worldToScreen(pos[0], pos[1], currentView, plotW, plotH);
      const lineColors = [COLORS.s1, COLORS.s2, COLORS.s3];
      for (let i = 0; i < 3; i++) {
        const d = dists[i];
        if (!Number.isFinite(d) || d <= 0) continue;
        ctx.save();
        ctx.strokeStyle = lineColors[i];
        ctx.globalAlpha = 0.72;
        ctx.lineWidth = 1.4;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(aScreen[i][0], aScreen[i][1]);
        ctx.lineTo(rx, ry);
        ctx.stroke();
        ctx.restore();

        const mx = (aScreen[i][0] + rx) * 0.5;
        const my = (aScreen[i][1] + ry) * 0.5;
        if (Math.hypot(rx - aScreen[i][0], ry - aScreen[i][1]) > 56) {
          drawPillLabel(ctx, `${d.toFixed(2)}m`, mx, my, theme.hudBg, lineColors[i], theme.border);
        }
      }
    }

    // 6. Trajectory Polyline (fading gradient)
    const traj = Array.isArray(state?.trajectory) ? state.trajectory : [];
    if (traj.length >= 2) {
      ctx.save();
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      const [r, g, b] = COLORS.traj;
      for (let i = 1; i < traj.length; i++) {
        const p0 = traj[i - 1];
        const p1 = traj[i];
        if (!Array.isArray(p0) || !Array.isArray(p1)) continue;
        const [x0, y0] = worldToScreen(p0[0], p0[1], currentView, plotW, plotH);
        const [x1, y1] = worldToScreen(p1[0], p1[1], currentView, plotW, plotH);
        const t = i / (traj.length - 1);
        ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, ${(0.12 + 0.82 * t).toFixed(3)})`;
        ctx.lineWidth = 1.6 + 2.0 * t;
        ctx.beginPath();
        ctx.moveTo(x0, y0);
        ctx.lineTo(x1, y1);
        ctx.stroke();
      }
      ctx.restore();
    }

    // 7. Anchor Markers
    const p2Port = state?.is_swapped ? "P3" : "P2";
    const p3Port = state?.is_swapped ? "P2" : "P3";
    const anchorLabels = [
      `S1 (0, 0)`,
      `S2 [${p2Port}] (${anchors[1][0].toFixed(2)}, ${anchors[1][1].toFixed(2)})`,
      `S3 [${p3Port}] (${anchors[2][0].toFixed(2)}, ${anchors[2][1].toFixed(2)})`,
    ];
    for (let i = 0; i < 3; i++) {
      const [ax, ay] = aScreen[i];
      ctx.save();
      ctx.fillStyle = COLORS.anchorFill;
      ctx.strokeStyle = COLORS.anchorStroke;
      ctx.lineWidth = 2;
      const sz = 14;
      ctx.beginPath();
      if (typeof ctx.roundRect === "function") {
        ctx.roundRect(ax - sz * 0.5, ay - sz * 0.5, sz, sz, 4);
      } else {
        ctx.rect(ax - sz * 0.5, ay - sz * 0.5, sz, sz);
      }
      ctx.fill();
      ctx.stroke();

      ctx.font = "700 11px Inter, Noto Sans JP, sans-serif";
      const label = anchorLabels[i];
      const tw = ctx.measureText(label).width + 12;
      let lx = ax + 12;
      let ly = ay - 12;
      if (i === 0) {
        lx = ax + 12;
        ly = ay + 18;
      }
      if (lx + tw > PAD_LEFT + plotW - 4) lx = ax - tw - 10;
      if (ly < PAD_TOP + 14) ly = ay + 20;
      if (ly > PAD_TOP + plotH - 8) ly = ay - 14;

      drawPillLabel(ctx, label, lx + tw * 0.5 - 6, ly, theme.hudBg, theme.textMain, theme.border);
      ctx.restore();
    }

    // 8. Robot Marker
    if (hasPos) {
      const [rx, ry] = worldToScreen(pos[0], pos[1], currentView, plotW, plotH);
      const pulse = (Math.sin(nowMs * 0.006) + 1) * 0.5;
      ctx.save();
      ctx.beginPath();
      ctx.arc(rx, ry, 14 + pulse * 10, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(34, 197, 94, ${(0.24 - pulse * 0.16).toFixed(3)})`;
      ctx.fill();

      ctx.beginPath();
      ctx.arc(rx, ry, 9.5, 0, Math.PI * 2);
      ctx.fillStyle = COLORS.robot;
      ctx.fill();
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = "#ffffff";
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(rx, ry, 3, 0, Math.PI * 2);
      ctx.fillStyle = "#ffffff";
      ctx.fill();

      const coordText = `(${pos[0].toFixed(2)}, ${pos[1].toFixed(2)}) m`;
      ctx.font = "700 11px Inter, Noto Sans JP, sans-serif";
      const cw = ctx.measureText(coordText).width + 16;
      const pillCx = Math.max(PAD_LEFT + cw * 0.5 + 4, Math.min(PAD_LEFT + plotW - cw * 0.5 - 4, rx));
      const pillCy = ry - 24 > PAD_TOP + 14 ? ry - 24 : ry + 26;
      drawPillLabel(ctx, coordText, pillCx, pillCy, "#14532d", "#ecfdf5", "#22c55e");
      ctx.restore();
    }

    ctx.restore(); // End Plot Clip

    // --- Plot Border & Axis Ticks ---
    ctx.strokeStyle = theme.border;
    ctx.lineWidth = 1;
    ctx.strokeRect(PAD_LEFT, PAD_TOP, plotW, plotH);

    ctx.fillStyle = theme.textMuted;
    ctx.font = "600 11px Inter, Noto Sans JP, sans-serif";

    // X axis labels
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let x = startMajorX; x <= maxWorldX + majorStep * 0.5; x += majorStep) {
      const [sx] = worldToScreen(x, 0, currentView, plotW, plotH);
      if (sx < PAD_LEFT + 6 || sx > PAD_LEFT + plotW - 6) continue;
      ctx.beginPath();
      ctx.moveTo(sx, PAD_TOP + plotH);
      ctx.lineTo(sx, PAD_TOP + plotH + 5);
      ctx.stroke();
      ctx.fillText(formatMeters(x, majorStep), sx, PAD_TOP + plotH + 7);
    }
    ctx.fillText("X [m]", PAD_LEFT + plotW * 0.5, PAD_TOP + plotH + 23);

    // Y axis labels
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let y = startMajorY; y <= maxWorldY + majorStep * 0.5; y += majorStep) {
      const [, sy] = worldToScreen(0, y, currentView, plotW, plotH);
      if (sy < PAD_TOP + 6 || sy > PAD_TOP + plotH - 6) continue;
      ctx.beginPath();
      ctx.moveTo(PAD_LEFT - 5, sy);
      ctx.lineTo(PAD_LEFT, sy);
      ctx.stroke();
      ctx.fillText(formatMeters(y, majorStep), PAD_LEFT - 8, sy);
    }

    ctx.save();
    ctx.translate(15, PAD_TOP + plotH * 0.5);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("Y [m]", 0, 0);
    ctx.restore();

    // Update Scale Bar HUD & Viewport Range Text
    const barMeters = niceStep(95 / pxPerM);
    const barPx = Math.max(24, Math.min(180, Math.round(barMeters * pxPerM)));
    scaleBar.style.width = `${barPx}px`;
    scaleBarLabel.textContent = barMeters < 1.0 ? `${barMeters.toFixed(1)} m` : `${formatMeters(barMeters, barMeters)} m`;
    viewportRangeText.textContent =
      `表示範囲: ${visibleW.toFixed(1)} m × ${visibleH.toFixed(1)} m (${autoScale ? "AUTO" : "MANUAL"})`;

    ctx.restore();
  }

  // ---------------------------------------------------------------------------
  // Render Distance Time-Series Chart (Dynamic Y-Axis Auto-Scaling)
  // ---------------------------------------------------------------------------
  function renderChart(dt) {
    const w = chartCssW;
    const h = chartCssH;
    if (w <= 20 || h <= 20) return;

    const theme = getCssColors();
    const ctx = chartCtx;

    const cLeft = 44;
    const cRight = 10;
    const cTop = 10;
    const cBottom = 20;
    const pw = w - cLeft - cRight;
    const ph = h - cTop - cBottom;
    if (pw <= 10 || ph <= 10) return;

    let maxD = 0;
    for (const sample of distHistory) {
      for (let k = 0; k < 3; k++) {
        if (Number.isFinite(sample[k]) && sample[k] > maxD) {
          maxD = sample[k];
        }
      }
    }
    const targetMaxY = Math.max(1.0, niceCeil(maxD * 1.15));
    const alpha = 1 - Math.exp(-6.0 * dt);
    chartMaxY += (targetMaxY - chartMaxY) * alpha;
    chartRangeBadge.textContent = `Y軸: 0〜${chartMaxY.toFixed(1)} m`;

    ctx.save();
    ctx.setTransform(chartDpr, 0, 0, chartDpr, 0, 0);
    ctx.fillStyle = theme.bg;
    ctx.fillRect(0, 0, w, h);

    // Grid & Y labels
    const ticks = 4;
    ctx.font = "600 10px Inter, Noto Sans JP, sans-serif";
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";

    for (let i = 0; i <= ticks; i++) {
      const frac = i / ticks;
      const val = chartMaxY * (1 - frac);
      const y = cTop + ph * frac;
      ctx.strokeStyle = i === ticks ? theme.border : theme.gridMajor;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cLeft, y);
      ctx.lineTo(cLeft + pw, y);
      ctx.stroke();

      ctx.fillStyle = theme.textMuted;
      ctx.fillText(`${val.toFixed(1)}m`, cLeft - 5, y);
    }

    // Plot series
    const seriesColors = [COLORS.s1, COLORS.s2, COLORS.s3];
    const n = distHistory.length;
    if (n >= 2) {
      const offset = MAX_DIST_HISTORY - n;
      for (let k = 0; k < 3; k++) {
        ctx.save();
        ctx.strokeStyle = seriesColors[k];
        ctx.lineWidth = 1.8;
        ctx.lineJoin = "round";
        ctx.beginPath();
        let penDown = false;
        for (let i = 0; i < n; i++) {
          const d = distHistory[i][k];
          if (!Number.isFinite(d) || d <= 0) {
            penDown = false;
            continue;
          }
          const x = cLeft + ((offset + i) / (MAX_DIST_HISTORY - 1)) * pw;
          const clamped = Math.min(chartMaxY, Math.max(0, d));
          const y = cTop + ph * (1 - clamped / chartMaxY);
          if (!penDown) {
            ctx.moveTo(x, y);
            penDown = true;
          } else {
            ctx.lineTo(x, y);
          }
        }
        ctx.stroke();
        ctx.restore();
      }
    } else {
      ctx.fillStyle = theme.textMuted;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.font = "500 11px Inter, Noto Sans JP, sans-serif";
      ctx.fillText("データ受信待機中...", cLeft + pw * 0.5, cTop + ph * 0.5);
    }

    ctx.fillStyle = theme.textMuted;
    ctx.font = "500 10px Inter, Noto Sans JP, sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText("-15s", cLeft, cTop + ph + 5);
    ctx.textAlign = "right";
    ctx.fillText("現在", cLeft + pw, cTop + ph + 5);

    ctx.restore();
  }

  // ---------------------------------------------------------------------------
  // Animation Loop
  // ---------------------------------------------------------------------------
  let prevFrameTime = performance.now();
  function animationLoop(now) {
    const dt = Math.min(0.1, Math.max(0.001, (now - prevFrameTime) / 1000));
    prevFrameTime = now;

    renderMap(now, dt);
    renderChart(dt);
    requestAnimationFrame(animationLoop);
  }
  requestAnimationFrame(animationLoop);
})();
