/**
 * ROS 2 Bag Visualization Studio — Frontend Application
 * Synchronizes multi-camera video streams, 2D/3D spatial sensors,
 * telemetry time-series charts, and all-topic live inspection.
 */

(() => {
  "use strict";

  const PALETTE = [
    "#38bdf8", // sky
    "#10b981", // emerald
    "#f59e0b", // amber
    "#f43f5e", // rose
    "#a855f7", // purple
    "#22d3ee", // cyan
    "#84cc16", // lime
    "#fb923c", // orange
    "#ec4899", // pink
    "#6366f1", // indigo
    "#14b8a6", // teal
    "#eab308", // yellow
  ];

  // Application State
  const state = {
    bag: null,
    discoveredBags: [],
    playing: false,
    playSpeed: 1.0,
    loop: true,
    currentTime: 0.0,
    duration: 0.0,
    lastRafTs: 0,

    // Camera view settings
    camViewMode: "multi", // "single" | "multi"
    camGridCols: "auto",
    depthColormap: "turbo",
    selectedCamIds: new Set(),
    soloCamId: null,
    camTileElements: new Map(), // camId -> { img, frameLabel, deltaBadge, lastMsgId, loadingMsgId }

    // Spatial view settings
    spatialMode: "trajectory", // "trajectory" | "pointcloud" | "laserscan"
    selectedPcTopics: new Set(),
    pcColorMode: "camera",
    orbit: {
      yaw: -0.65,
      pitch: 0.45,
      distance: 4.2,
      panX: 0,
      panY: 0,
      zoom2D: 1.0,
      offset2DX: 0,
      offset2DY: 0,
      dragging: false,
      dragButton: 0,
      lastX: 0,
      lastY: 0,
    },

    // Charts settings
    chartStreamMode: "cursor", // "cursor" | "progressive"
    chartModes: {
      1: "odom_vel",
      2: "imu_euler",
      3: "joint_pos",
      4: "battery",
    },

    // Synchronized snapshot from backend (/api/sync)
    syncSnapshot: null,
    syncFetchInFlight: false,
    lastSyncFetchTime: -1,
    lastSyncWallMs: 0,

    // Topics inspector
    topicSearch: "",
    topicCategory: "all",
    expandedTopics: new Set(),
  };

  // DOM Elements
  const el = {
    workspaceGrid: document.getElementById("workspaceGrid"),
    robotProfileBadge: document.getElementById("robotProfileBadge"),
    bagNameDisplay: document.getElementById("bagNameDisplay"),
    bagDurationDisplay: document.getElementById("bagDurationDisplay"),
    bagTopicsCount: document.getElementById("bagTopicsCount"),
    bagMsgsCount: document.getElementById("bagMsgsCount"),
    bagCamsCount: document.getElementById("bagCamsCount"),
    tabTopicsBadge: document.getElementById("tabTopicsBadge"),

    // Camera controls
    activeCamerasCountLabel: document.getElementById("activeCamerasCountLabel"),
    camModeControl: document.getElementById("camModeControl"),
    camGridColsSelect: document.getElementById("camGridColsSelect"),
    selectAllRgbBtn: document.getElementById("selectAllRgbBtn"),
    selectAllDepthBtn: document.getElementById("selectAllDepthBtn"),
    selectAllCamsBtn: document.getElementById("selectAllCamsBtn"),
    depthColormapSelect: document.getElementById("depthColormapSelect"),
    cameraSelectorStrip: document.getElementById("cameraSelectorStrip"),
    cameraGrid: document.getElementById("cameraGrid"),

    // Spatial controls
    spatialTabs: document.getElementById("spatialTabs"),
    tabBtnPointcloud: document.getElementById("tabBtnPointcloud"),
    tabBtnLaserscan: document.getElementById("tabBtnLaserscan"),
    resetSpatialViewBtn: document.getElementById("resetSpatialViewBtn"),
    pcSelectorStrip: document.getElementById("pcSelectorStrip"),
    pcTopicPills: document.getElementById("pcTopicPills"),
    pcColorModeSelect: document.getElementById("pcColorModeSelect"),
    spatialCanvas: document.getElementById("spatialCanvas"),
    spatialHud: document.getElementById("spatialHud"),

    // Charts
    chartStreamModeControl: document.getElementById("chartStreamModeControl"),
    chartSelects: {
      1: document.getElementById("chart1Select"),
      2: document.getElementById("chart2Select"),
      3: document.getElementById("chart3Select"),
      4: document.getElementById("chart4Select"),
    },
    chartCanvases: {
      1: document.getElementById("chartCanvas1"),
      2: document.getElementById("chartCanvas2"),
      3: document.getElementById("chartCanvas3"),
      4: document.getElementById("chartCanvas4"),
    },
    chartLegends: {
      1: document.getElementById("chartLegend1"),
      2: document.getElementById("chartLegend2"),
      3: document.getElementById("chartLegend3"),
      4: document.getElementById("chartLegend4"),
    },

    // Topics Inspector
    inspectorTopicsCount: document.getElementById("inspectorTopicsCount"),
    topicSearchInput: document.getElementById("topicSearchInput"),
    topicCategoryFilter: document.getElementById("topicCategoryFilter"),
    topicsInspectorList: document.getElementById("topicsInspectorList"),

    // Transport
    jumpStartBtn: document.getElementById("jumpStartBtn"),
    stepBackBtn: document.getElementById("stepBackBtn"),
    playPauseBtn: document.getElementById("playPauseBtn"),
    playIcon: document.getElementById("playIcon"),
    pauseIcon: document.getElementById("pauseIcon"),
    playBtnLabel: document.getElementById("playBtnLabel"),
    stepFwdBtn: document.getElementById("stepFwdBtn"),
    currentTimeDisplay: document.getElementById("currentTimeDisplay"),
    totalTimeDisplay: document.getElementById("totalTimeDisplay"),
    timelineSlider: document.getElementById("timelineSlider"),
    timelineEventsLayer: document.getElementById("timelineEventsLayer"),
    epochTimeDisplay: document.getElementById("epochTimeDisplay"),
    playbackSpeedSelect: document.getElementById("playbackSpeedSelect"),
    loopToggleBtn: document.getElementById("loopToggleBtn"),

    // Modal
    openBagModalBtn: document.getElementById("openBagModalBtn"),
    closeBagModalBtn: document.getElementById("closeBagModalBtn"),
    bagModalBackdrop: document.getElementById("bagModalBackdrop"),
    uploadDropZone: document.getElementById("uploadDropZone"),
    uploadDb3Input: document.getElementById("uploadDb3Input"),
    uploadFolderInput: document.getElementById("uploadFolderInput"),
    uploadProgressWrap: document.getElementById("uploadProgressWrap"),
    uploadProgressFill: document.getElementById("uploadProgressFill"),
    uploadProgressText: document.getElementById("uploadProgressText"),
    customBagPathInput: document.getElementById("customBagPathInput"),
    loadCustomPathBtn: document.getElementById("loadCustomPathBtn"),
    discoveredBagsList: document.getElementById("discoveredBagsList"),
  };

  // Utility: Binary search on sorted array `arr` for latest value <= target
  function bisectRightLe(arr, target) {
    if (!arr || arr.length === 0) return -1;
    let lo = 0;
    let hi = arr.length;
    while (lo < hi) {
      const mid = (lo + hi) >>> 1;
      if (arr[mid] <= target) lo = mid + 1;
      else hi = mid;
    }
    return Math.max(0, lo - 1);
  }

  function formatSec(sec) {
    return sec.toFixed(3).padStart(6, "0") + "s";
  }

  function formatEpochTime(startNs, relSec) {
    if (!startNs) return "--:--:--.---";
    const ms = Math.floor(startNs / 1e6 + relSec * 1000);
    const d = new Date(ms);
    if (isNaN(d.getTime())) return `${relSec.toFixed(3)}s`;
    const hh = String(d.getUTCHours()).padStart(2, "0");
    const mm = String(d.getUTCMinutes()).padStart(2, "0");
    const ss = String(d.getUTCSeconds()).padStart(2, "0");
    const mmm = String(d.getUTCMilliseconds()).padStart(3, "0");
    return `${hh}:${mm}:${ss}.${mmm} UTC`;
  }

  // Initialize Bag Data into UI
  function applyLoadedBag(bagData, discoveredBags) {
    state.bag = bagData;
    if (discoveredBags) state.discoveredBags = discoveredBags;
    setPlaying(false);

    state.currentTime = 0.0;
    state.duration = bagData.duration_s || 1.0;
    state.soloCamId = null;

    // Header metadata
    el.robotProfileBadge.textContent = bagData.robot_profile || "ROS 2 Bag";
    el.bagNameDisplay.textContent = bagData.bag_name;
    el.bagDurationDisplay.textContent = `${bagData.duration_s.toFixed(2)}s`;
    el.bagTopicsCount.textContent = bagData.topic_count;
    el.bagMsgsCount.textContent = bagData.total_messages.toLocaleString();
    el.bagCamsCount.textContent = bagData.cameras.length;
    el.tabTopicsBadge.textContent = bagData.topic_count;

    // Timeline slider
    el.timelineSlider.min = "0";
    el.timelineSlider.max = String(state.duration);
    el.timelineSlider.step = "0.005";
    el.timelineSlider.value = "0";
    el.totalTimeDisplay.textContent = formatSec(state.duration);
    renderTimelineEvents();

    // Default camera selection: prefer RGB cameras first (or all if <= 4)
    state.selectedCamIds.clear();
    const rgbCams = bagData.cameras.filter((c) => !c.is_depth);
    if (rgbCams.length > 0) {
      rgbCams.forEach((c) => state.selectedCamIds.add(c.id));
    } else {
      bagData.cameras.forEach((c) => state.selectedCamIds.add(c.id));
    }

    // Spatial sensor tabs availability
    const pcTopics = bagData.spatial_topics.filter((t) => t.category === "pointcloud");
    const scanTopics = bagData.spatial_topics.filter((t) => t.category === "laserscan");

    state.selectedPcTopics.clear();
    pcTopics.forEach((t) => state.selectedPcTopics.add(t.name));

    el.tabBtnPointcloud.classList.toggle("disabled", pcTopics.length === 0);
    el.tabBtnLaserscan.classList.toggle("disabled", scanTopics.length === 0);

    // Choose smart default spatial view and chart 3 mode based on robot sensors
    if (scanTopics.length > 0 && pcTopics.length === 0) {
      setSpatialMode("laserscan");
    } else if (pcTopics.length > 0) {
      setSpatialMode("trajectory");
    } else {
      setSpatialMode("trajectory");
    }

    const ts = bagData.timeseries || {};
    if (ts.joints && Object.keys(ts.joints).length > 0) {
      const firstJointTopic = Object.values(ts.joints)[0];
      if (firstJointTopic.names && firstJointTopic.names.length <= 2 && ts.sensor_state && Object.keys(ts.sensor_state).length > 0) {
        state.chartModes[3] = "encoders";
        el.chartSelects[3].value = "encoders";
      } else {
        state.chartModes[3] = "joint_pos";
        el.chartSelects[3].value = "joint_pos";
      }
    }

    renderPointCloudPills(pcTopics);
    renderCameraSelectorPills();
    rebuildCameraGrid();
    renderDiscoveredBagsList();

    // Trigger initial sync update
    seekToTime(0.0, true);
  }

  function renderTimelineEvents() {
    el.timelineEventsLayer.innerHTML = "";
    if (!state.bag || !state.bag.events) return;
    state.bag.events.forEach((ev) => {
      const pct = Math.max(0, Math.min(100, (ev.t / state.duration) * 100));
      const pin = document.createElement("div");
      pin.className = "timeline-event-Pin";
      pin.style.left = `${pct}%`;
      pin.title = `${ev.event} @ ${ev.t.toFixed(2)}s`;
      el.timelineEventsLayer.appendChild(pin);
    });
  }

  // Camera Stream Selector & Grid
  function renderCameraSelectorPills() {
    el.cameraSelectorStrip.innerHTML = "";
    if (!state.bag || !state.bag.cameras.length) {
      el.activeCamerasCountLabel.textContent = "0 / 0 active";
      return;
    }

    state.bag.cameras.forEach((cam) => {
      const isSelected = state.selectedCamIds.has(cam.id);
      const pill = document.createElement("button");
      pill.className = `cam-pill ${cam.is_depth ? "depth-pill" : ""} ${isSelected ? "active" : ""}`;
      pill.title = `${cam.name} (${cam.message_count} frames, ${cam.hz} Hz)`;
      pill.innerHTML = `
        <span class="cam-pill-dot"></span>
        <span>${cam.label}</span>
        <span class="cam-pill-hz">${cam.hz}Hz</span>
      `;
      pill.addEventListener("click", () => {
        state.soloCamId = null;
        if (state.camViewMode === "single") {
          state.selectedCamIds.clear();
          state.selectedCamIds.add(cam.id);
        } else {
          if (state.selectedCamIds.has(cam.id)) {
            if (state.selectedCamIds.size > 1) {
              state.selectedCamIds.delete(cam.id);
            }
          } else {
            state.selectedCamIds.add(cam.id);
          }
        }
        renderCameraSelectorPills();
        rebuildCameraGrid();
        updateCameraFrames(state.currentTime);
      });
      el.cameraSelectorStrip.appendChild(pill);
    });

    const visibleCount = getVisibleCameras().length;
    el.activeCamerasCountLabel.textContent = `${visibleCount} / ${state.bag.cameras.length} active`;
  }

  function getVisibleCameras() {
    if (!state.bag || !state.bag.cameras) return [];
    if (state.soloCamId !== null) {
      return state.bag.cameras.filter((c) => c.id === state.soloCamId);
    }
    const list = state.bag.cameras.filter((c) => state.selectedCamIds.has(c.id));
    if (state.camViewMode === "single") {
      return list.slice(0, 1);
    }
    return list;
  }

  function rebuildCameraGrid() {
    el.cameraGrid.innerHTML = "";
    state.camTileElements.clear();

    const visibleCams = getVisibleCameras();
    const count = visibleCams.length;

    // Determine grid columns class
    el.cameraGrid.classList.remove("grid-1", "grid-2", "grid-3", "grid-4");
    let cols = 1;
    if (state.soloCamId !== null || state.camViewMode === "single" || count <= 1) {
      cols = 1;
    } else if (state.camGridCols !== "auto") {
      cols = parseInt(state.camGridCols, 10) || 2;
    } else {
      if (count === 2) cols = 2;
      else if (count <= 4) cols = 2;
      else if (count <= 6) cols = 3;
      else cols = 4;
    }
    el.cameraGrid.classList.add(`grid-${cols}`);

    if (count === 0) {
      el.cameraGrid.innerHTML = `<div class="empty-state">Select at least one camera topic above to view stream</div>`;
      return;
    }

    visibleCams.forEach((cam) => {
      const tile = document.createElement("div");
      tile.className = "camera-tile";

      const isSolo = state.soloCamId === cam.id;
      tile.innerHTML = `
        <div class="camera-tile-header">
          <div class="camera-tile-title">
            <span>${cam.label}</span>
            <span class="camera-type-tag ${cam.is_depth ? "depth" : ""}">${cam.is_depth ? "DEPTH 16UC1" : "RGB"}</span>
          </div>
          <div class="camera-tile-actions">
            <button class="cam-solo-btn">${isSolo ? "Exit Solo" : "Maximize"}</button>
          </div>
        </div>
        <div class="camera-img-wrap">
          <img class="camera-img" alt="${cam.label}" />
        </div>
        <div class="camera-tile-footer">
          <span class="cam-frame-info">Frame 0 / ${cam.message_count}</span>
          <span class="sync-delta-badge">Δt 0.0ms</span>
        </div>
      `;

      const soloBtn = tile.querySelector(".cam-solo-btn");
      soloBtn.addEventListener("click", (e) => {
        e.stopPropagation();
        if (state.soloCamId === cam.id) {
          state.soloCamId = null;
        } else {
          state.soloCamId = cam.id;
        }
        renderCameraSelectorPills();
        rebuildCameraGrid();
        updateCameraFrames(state.currentTime);
      });

      const imgEl = tile.querySelector(".camera-img");
      const frameLabel = tile.querySelector(".cam-frame-info");
      const deltaBadge = tile.querySelector(".sync-delta-badge");

      state.camTileElements.set(cam.id, {
        cam,
        img: imgEl,
        frameLabel,
        deltaBadge,
        lastMsgId: null,
        lastColormap: null,
        loadingMsgId: null,
      });

      el.cameraGrid.appendChild(tile);
    });
  }

  function updateCameraFrames(relTime) {
    if (!state.bag) return;
    const cmap = state.depthColormap;

    state.camTileElements.forEach((tileObj) => {
      const { cam, img, frameLabel, deltaBadge } = tileObj;
      const tArr = cam.frames.t;
      const idArr = cam.frames.id;
      if (!tArr || tArr.length === 0) return;

      const idx = bisectRightLe(tArr, relTime);
      const msgId = idArr[idx];
      const frameT = tArr[idx];
      const deltaMs = (relTime - frameT) * 1000.0;

      frameLabel.textContent = `Frame ${idx + 1}/${tArr.length} · ${frameT.toFixed(2)}s`;
      deltaBadge.textContent = `Δt ${deltaMs >= 0 ? "+" : ""}${deltaMs.toFixed(0)}ms`;
      deltaBadge.classList.toggle("warn", Math.abs(deltaMs) > 180);

      const effectiveCmap = cam.is_depth ? cmap : "turbo";
      if (tileObj.lastMsgId === msgId && tileObj.lastColormap === effectiveCmap) {
        return;
      }

      if (tileObj.loadingMsgId === msgId) {
        return;
      }

      tileObj.loadingMsgId = msgId;
      const url = `/api/frame?id=${msgId}&colormap=${effectiveCmap}`;
      const preloader = new Image();
      preloader.onload = () => {
        img.src = url;
        tileObj.lastMsgId = msgId;
        tileObj.lastColormap = effectiveCmap;
        if (tileObj.loadingMsgId === msgId) {
          tileObj.loadingMsgId = null;
        }
      };
      preloader.onerror = () => {
        if (tileObj.loadingMsgId === msgId) {
          tileObj.loadingMsgId = null;
        }
      };
      preloader.src = url;

      // Lookahead prefetch of next frame while playing
      if (state.playing && idx + 1 < idArr.length) {
        const nextId = idArr[idx + 1];
        const prefetchImg = new Image();
        prefetchImg.src = `/api/frame?id=${nextId}&colormap=${effectiveCmap}`;
      }
    });
  }

  // Spatial 2D / 3D Viewer
  function renderPointCloudPills(pcTopics) {
    el.pcTopicPills.innerHTML = "";
    pcTopics.forEach((t, idx) => {
      const shortName = t.name
        .replace("/ma_raihan/cameras/", "")
        .replace("/points_native", "")
        .replace("_", " ");
      const active = state.selectedPcTopics.has(t.name);
      const btn = document.createElement("button");
      btn.className = `cam-pill ${active ? "active" : ""}`;
      const color = PALETTE[idx % PALETTE.length];
      btn.innerHTML = `
        <span class="cam-pill-dot" style="background:${active ? color : "#475569"}"></span>
        <span>${shortName}</span>
      `;
      btn.addEventListener("click", () => {
        if (state.selectedPcTopics.has(t.name)) {
          if (state.selectedPcTopics.size > 1) state.selectedPcTopics.delete(t.name);
        } else {
          state.selectedPcTopics.add(t.name);
        }
        renderPointCloudPills(pcTopics);
        drawSpatialCanvas();
      });
      el.pcTopicPills.appendChild(btn);
    });
  }

  function setSpatialMode(mode) {
    state.spatialMode = mode;
    el.spatialTabs.querySelectorAll(".spatial-tab").forEach((btn) => {
      btn.classList.toggle("active", btn.dataset.view === mode);
    });
    el.pcSelectorStrip.classList.toggle("hidden", mode !== "pointcloud");
    drawSpatialCanvas();
  }

  function resizeCanvasToDisplaySize(canvas) {
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(10, Math.floor(rect.width * dpr));
    const height = Math.max(10, Math.floor(rect.height * dpr));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    return { width, height, dpr };
  }

  function drawSpatialCanvas() {
    const canvas = el.spatialCanvas;
    if (!canvas) return;
    const { width, height, dpr } = resizeCanvasToDisplaySize(canvas);
    const ctx = canvas.getContext("2d");
    ctx.save();
    ctx.scale(dpr, dpr);
    const w = width / dpr;
    const h = height / dpr;

    ctx.fillStyle = "#090d16";
    ctx.fillRect(0, 0, w, h);

    if (!state.bag) {
      ctx.restore();
      return;
    }

    if (state.spatialMode === "trajectory") {
      drawOdometryTrajectory2D(ctx, w, h);
    } else if (state.spatialMode === "pointcloud") {
      drawPointCloud3D(ctx, w, h);
    } else if (state.spatialMode === "laserscan") {
      drawLaserScan2D(ctx, w, h);
    }

    ctx.restore();
  }

  function drawOdometryTrajectory2D(ctx, w, h) {
    const odomMap = (state.bag.timeseries && state.bag.timeseries.odometry) || {};
    const odomNames = Object.keys(odomMap);
    if (odomNames.length === 0) {
      ctx.fillStyle = "#64748b";
      ctx.font = "12px sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("No Odometry topic in this bag", w / 2, h / 2);
      el.spatialHud.textContent = "Odometry: N/A";
      return;
    }

    const topicName = odomNames[0];
    const series = odomMap[topicName];
    const xs = series.x;
    const ys = series.y;
    const ts = series.t;
    if (!xs || xs.length === 0) return;

    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (let i = 0; i < xs.length; i++) {
      if (xs[i] < minX) minX = xs[i];
      if (xs[i] > maxX) maxX = xs[i];
      if (ys[i] < minY) minY = ys[i];
      if (ys[i] > maxY) maxY = ys[i];
    }

    const spanX = Math.max(0.4, maxX - minX);
    const spanY = Math.max(0.4, maxY - minY);
    const pad = 42;
    const baseScale = Math.min((w - pad * 2) / spanX, (h - pad * 2) / spanY) * state.orbit.zoom2D;
    const cx = (minX + maxX) / 2;
    const cy = (minY + maxY) / 2;

    const toScreen = (x, y) => [
      w / 2 + (x - cx) * baseScale + state.orbit.offset2DX,
      h / 2 - (y - cy) * baseScale + state.orbit.offset2DY,
    ];

    // Subtle metric grid
    const gridStep = spanX > 5 ? 1.0 : spanX > 1.5 ? 0.5 : 0.2;
    ctx.strokeStyle = "#141e30";
    ctx.lineWidth = 1;
    const startGX = Math.floor((minX - 2) / gridStep) * gridStep;
    const endGX = Math.ceil((maxX + 2) / gridStep) * gridStep;
    for (let gx = startGX; gx <= endGX; gx += gridStep) {
      const [sx] = toScreen(gx, cy);
      if (sx >= 0 && sx <= w) {
        ctx.beginPath();
        ctx.moveTo(sx, 0);
        ctx.lineTo(sx, h);
        ctx.stroke();
      }
    }
    const startGY = Math.floor((minY - 2) / gridStep) * gridStep;
    const endGY = Math.ceil((maxY + 2) / gridStep) * gridStep;
    for (let gy = startGY; gy <= endGY; gy += gridStep) {
      const [, sy] = toScreen(cx, gy);
      if (sy >= 0 && sy <= h) {
        ctx.beginPath();
        ctx.moveTo(0, sy);
        ctx.lineTo(w, sy);
        ctx.stroke();
      }
    }

    // Full future trajectory (dimmed)
    ctx.strokeStyle = "rgba(56, 189, 248, 0.22)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < xs.length; i++) {
      const [px, py] = toScreen(xs[i], ys[i]);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();

    // Traversed trajectory up to state.currentTime
    const curIdx = bisectRightLe(ts, state.currentTime);
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 3;
    ctx.beginPath();
    for (let i = 0; i <= curIdx; i++) {
      const [px, py] = toScreen(xs[i], ys[i]);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();

    // Start marker
    const [s0x, s0y] = toScreen(xs[0], ys[0]);
    ctx.fillStyle = "#10b981";
    ctx.beginPath();
    ctx.arc(s0x, s0y, 5, 0, Math.PI * 2);
    ctx.fill();

    // Current robot pose & heading arrow
    const curX = xs[curIdx];
    const curY = ys[curIdx];
    const curZ = series.z[curIdx];
    const curYawDeg = series.yaw[curIdx];
    const curYawRad = (curYawDeg * Math.PI) / 180.0;
    const [rx, ry] = toScreen(curX, curY);

    // Heading arrow
    const arrowLen = 22;
    const ax = rx + Math.cos(curYawRad) * arrowLen;
    const ay = ry - Math.sin(curYawRad) * arrowLen;
    ctx.strokeStyle = "#f59e0b";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(ax, ay);
    ctx.stroke();

    // Robot body circle
    ctx.fillStyle = "#f59e0b";
    ctx.beginPath();
    ctx.arc(rx, ry, 6.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1.5;
    ctx.stroke();

    el.spatialHud.innerHTML = `
      <div><strong>Topic:</strong> ${topicName}</div>
      <div><strong>Pose:</strong> X=${curX.toFixed(3)}m, Y=${curY.toFixed(3)}m, Z=${curZ.toFixed(3)}m</div>
      <div><strong>Heading:</strong> Yaw=${curYawDeg.toFixed(1)}° | Vel: vx=${series.vx[curIdx].toFixed(3)}m/s, wz=${series.wz[curIdx].toFixed(3)}rad/s</div>
    `;
  }

  function drawPointCloud3D(ctx, w, h) {
    const pcs = (state.syncSnapshot && state.syncSnapshot.pointclouds) || {};
    const pcTopics = Object.keys(pcs).filter((k) => state.selectedPcTopics.has(k));

    const yaw = state.orbit.yaw;
    const pitch = state.orbit.pitch;
    const dist = state.orbit.distance;
    const cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    const cosP = Math.cos(pitch), sinP = Math.sin(pitch);
    const fov = Math.min(w, h) * 0.9;

    const project3D = (x, y, z) => {
      // Rotate around Z (yaw), then X/Y (pitch)
      const rx = x * cosY - y * sinY;
      const ry = x * sinY + y * cosY;
      const rz = z;

      const cy = ry * cosP - rz * sinP;
      const cz = ry * sinP + rz * cosP + dist;
      if (cz <= 0.1) return null;
      const sx = w / 2 + (rx / cz) * fov + state.orbit.panX;
      const sy = h / 2 - (cy / cz) * fov + state.orbit.panY;
      return [sx, sy, cz];
    };

    // Draw 3D ground grid (z = 0, x & y in -2..2)
    ctx.strokeStyle = "#182235";
    ctx.lineWidth = 1;
    for (let g = -2.0; g <= 2.001; g += 0.5) {
      const p1 = project3D(g, -2, 0);
      const p2 = project3D(g, 2, 0);
      if (p1 && p2) {
        ctx.beginPath();
        ctx.moveTo(p1[0], p1[1]);
        ctx.lineTo(p2[0], p2[1]);
        ctx.stroke();
      }
      const q1 = project3D(-2, g, 0);
      const q2 = project3D(2, g, 0);
      if (q1 && q2) {
        ctx.beginPath();
        ctx.moveTo(q1[0], q1[1]);
        ctx.lineTo(q2[0], q2[1]);
        ctx.stroke();
      }
    }

    // Draw coordinate axes at origin
    const origin = project3D(0, 0, 0);
    const xAxis = project3D(0.5, 0, 0);
    const yAxis = project3D(0, 0.5, 0);
    const zAxis = project3D(0, 0, 0.5);
    if (origin) {
      const drawAxis = (end, color) => {
        if (!end) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(origin[0], origin[1]);
        ctx.lineTo(end[0], end[1]);
        ctx.stroke();
      };
      drawAxis(xAxis, "#ef4444");
      drawAxis(yAxis, "#22c55e");
      drawAxis(zAxis, "#3b82f6");
    }

    // Mount transforms for multi-camera visualization (front, left, right, rear_down)
    let totalPts = 0;
    pcTopics.forEach((tname, idx) => {
      const pc = pcs[tname];
      if (!pc || !pc.points) return;
      const pts = pc.points;
      totalPts += pts.length;

      let camYaw = 0;
      if (tname.includes("/left/")) camYaw = Math.PI / 2;
      else if (tname.includes("/right/")) camYaw = -Math.PI / 2;
      else if (tname.includes("/rear")) camYaw = Math.PI;
      const cY = Math.cos(camYaw), sY = Math.sin(camYaw);

      const baseColor = PALETTE[idx % PALETTE.length];

      for (let i = 0; i < pts.length; i++) {
        const [ox, oy, oz] = pts[i];
        // Optical frame (z forward, x right, y down) -> Robot frame (x forward, y left, z up)
        const rx = oz;
        const ry = -ox;
        const rz = -oy * 0.5;
        const wx = rx * cY - ry * sY;
        const wy = rx * sY + ry * cY;
        const wz = rz;

        const proj = project3D(wx, wy, wz);
        if (!proj) continue;
        const [sx, sy] = proj;
        if (sx < 0 || sx > w || sy < 0 || sy > h) continue;

        if (state.pcColorMode === "camera") {
          ctx.fillStyle = baseColor;
        } else if (state.pcColorMode === "depth") {
          const d = Math.hypot(wx, wy, wz);
          const hue = Math.max(0, Math.min(240, (1 - Math.min(d / 3.0, 1)) * 240));
          ctx.fillStyle = `hsl(${hue}, 90%, 60%)`;
        } else {
          const hue = Math.max(0, Math.min(260, ((wz + 0.8) / 1.6) * 240));
          ctx.fillStyle = `hsl(${hue}, 90%, 60%)`;
        }
        ctx.fillRect(sx - 1.2, sy - 1.2, 2.5, 2.5);
      }
    });

    el.spatialHud.innerHTML = `
      <div><strong>3D PointCloud2 Viewer</strong> (Drag to orbit, Scroll to zoom)</div>
      <div><strong>Active Clouds:</strong> ${pcTopics.length} | <strong>Points Rendered:</strong> ${totalPts.toLocaleString()}</div>
    `;
  }

  function drawLaserScan2D(ctx, w, h) {
    const scans = (state.syncSnapshot && state.syncSnapshot.laserscans) || {};
    const scanTopics = Object.keys(scans);
    if (scanTopics.length === 0) {
      ctx.fillStyle = "#64748b";
      ctx.font = "12px sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("No LaserScan (/scan) topic in this bag", w / 2, h / 2);
      el.spatialHud.textContent = "LaserScan: N/A";
      return;
    }

    const tname = scanTopics[0];
    const scan = scans[tname];
    const ranges = scan.ranges || [];
    const amin = scan.angle_min || 0;
    const ainc = scan.angle_increment || 0.025;

    const cx = w / 2 + state.orbit.offset2DX;
    const cy = h / 2 + state.orbit.offset2DY;
    const maxR = 4.0;
    const ppm = (Math.min(w, h) * 0.44 / maxR) * state.orbit.zoom2D;

    // Polar range rings
    ctx.strokeStyle = "#172236";
    ctx.fillStyle = "#475569";
    ctx.font = "10px monospace";
    for (let r = 1; r <= 4; r++) {
      ctx.beginPath();
      ctx.arc(cx, cy, r * ppm, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillText(`${r}m`, cx + 4, cy - r * ppm + 11);
    }

    // Crosshairs
    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(w, cy);
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, h);
    ctx.stroke();

    // Draw rays & hit points
    let validHits = 0;
    let minHit = Infinity;
    ctx.strokeStyle = "rgba(14, 165, 233, 0.12)";
    ctx.lineWidth = 1;

    const hitPoints = [];
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (r === null || r <= 0.02 || r > 20) continue;
      validHits++;
      if (r < minHit) minHit = r;
      const ang = amin + i * ainc;
      // Forward (+X) is UP (-Y on canvas), Left (+Y) is LEFT (-X on canvas)
      const px = cx - Math.sin(ang) * r * ppm;
      const py = cy - Math.cos(ang) * r * ppm;
      hitPoints.push([px, py, r]);

      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(px, py);
      ctx.stroke();
    }

    for (let i = 0; i < hitPoints.length; i++) {
      const [px, py, r] = hitPoints[i];
      const hue = Math.max(0, Math.min(200, (Math.min(r, 3.5) / 3.5) * 190));
      ctx.fillStyle = `hsl(${hue}, 95%, 58%)`;
      ctx.fillRect(px - 2, py - 2, 4, 4);
    }

    // Robot center
    ctx.fillStyle = "#f59e0b";
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx.fill();

    el.spatialHud.innerHTML = `
      <div><strong>Topic:</strong> ${tname} (Δt ${scan.delta_ms}ms)</div>
      <div><strong>Beams:</strong> ${validHits} / ${ranges.length} valid | <strong>Nearest Obstacle:</strong> ${isFinite(minHit) ? minHit.toFixed(2) + "m" : "N/A"}</div>
    `;
  }

  // Telemetry Charts Rendering
  function buildChartSeries(slotIndex) {
    if (!state.bag || !state.bag.timeseries) return [];
    const ts = state.bag.timeseries;
    const mode = state.chartModes[slotIndex];

    if (mode === "odom_vel") {
      const odom = Object.values(ts.odometry || {})[0];
      if (!odom) return [];
      return [
        { label: "vx (m/s)", t: odom.t, y: odom.vx, color: "#38bdf8" },
        { label: "vy (m/s)", t: odom.t, y: odom.vy, color: "#10b981" },
        { label: "wz (rad/s)", t: odom.t, y: odom.wz, color: "#f59e0b" },
      ];
    }
    if (mode === "odom_pos") {
      const odom = Object.values(ts.odometry || {})[0];
      if (!odom) return [];
      return [
        { label: "x (m)", t: odom.t, y: odom.x, color: "#38bdf8" },
        { label: "y (m)", t: odom.t, y: odom.y, color: "#10b981" },
        { label: "z (m)", t: odom.t, y: odom.z, color: "#a855f7" },
      ];
    }
    if (mode === "cmd_vel") {
      const out = [];
      Object.entries(ts.cmd_vel || {}).forEach(([tname, s], idx) => {
        const short = tname.split("/").pop();
        out.push({ label: `${short}.lx`, t: s.t, y: s.lx, color: PALETTE[(idx * 2) % PALETTE.length] });
        out.push({ label: `${short}.az`, t: s.t, y: s.az, color: PALETTE[(idx * 2 + 1) % PALETTE.length] });
      });
      return out;
    }
    if (mode === "imu_euler") {
      const imu = Object.values(ts.imu || {})[0];
      if (!imu) return [];
      return [
        { label: "Roll (°)", t: imu.t, y: imu.roll, color: "#f43f5e" },
        { label: "Pitch (°)", t: imu.t, y: imu.pitch, color: "#10b981" },
        { label: "Yaw (°)", t: imu.t, y: imu.yaw, color: "#38bdf8" },
      ];
    }
    if (mode === "imu_acc") {
      const imu = Object.values(ts.imu || {})[0];
      if (!imu) return [];
      return [
        { label: "ax (m/s²)", t: imu.t, y: imu.ax, color: "#f43f5e" },
        { label: "ay (m/s²)", t: imu.t, y: imu.ay, color: "#10b981" },
        { label: "az (m/s²)", t: imu.t, y: imu.az, color: "#38bdf8" },
      ];
    }
    if (mode === "imu_gyro") {
      const imu = Object.values(ts.imu || {})[0];
      if (!imu) return [];
      return [
        { label: "gx (rad/s)", t: imu.t, y: imu.gx, color: "#f43f5e" },
        { label: "gy (rad/s)", t: imu.t, y: imu.gy, color: "#10b981" },
        { label: "gz (rad/s)", t: imu.t, y: imu.gz, color: "#38bdf8" },
      ];
    }
    if (mode === "joint_pos" || mode === "joint_vel" || mode === "joint_eff") {
      const jTopic = Object.values(ts.joints || {})[0];
      if (!jTopic) return [];
      const field = mode === "joint_pos" ? "position" : mode === "joint_vel" ? "velocity" : "effort";
      const map = jTopic[field] || {};
      return Object.entries(map)
        .slice(0, 12)
        .map(([jname, vals], idx) => ({
          label: jname.replace("_joint", ""),
          t: jTopic.t,
          y: vals,
          color: PALETTE[idx % PALETTE.length],
        }));
    }
    if (mode === "foot_force") {
      const hs = Object.values(ts.high_state || {})[0];
      if (!hs) return [];
      return [
        { label: "FR Foot", t: hs.t, y: hs.foot_FR, color: "#38bdf8" },
        { label: "FL Foot", t: hs.t, y: hs.foot_FL, color: "#10b981" },
        { label: "RR Foot", t: hs.t, y: hs.foot_RR, color: "#f59e0b" },
        { label: "RL Foot", t: hs.t, y: hs.foot_RL, color: "#f43f5e" },
      ];
    }
    if (mode === "encoders") {
      const ss = Object.values(ts.sensor_state || {})[0];
      if (!ss) return [];
      return [
        { label: "Left Encoder", t: ss.t, y: ss.left_encoder, color: "#38bdf8" },
        { label: "Right Encoder", t: ss.t, y: ss.right_encoder, color: "#10b981" },
      ];
    }
    if (mode === "battery") {
      const bat = Object.values(ts.battery || {})[0];
      if (!bat) return [];
      return [
        { label: "Voltage (V)", t: bat.t, y: bat.voltage, color: "#10b981" },
        { label: "Current (A)", t: bat.t, y: bat.current, color: "#f43f5e" },
        { label: "Charge (%)", t: bat.t, y: bat.percentage, color: "#38bdf8" },
      ];
    }
    if (mode === "temp_joints" || mode === "temp_ctrl") {
      const temps = ts.temperature || {};
      const entries = Object.entries(temps).filter(([tname]) =>
        mode === "temp_joints" ? tname.includes("/robot/") : tname.includes("/controllers/")
      );
      return entries.slice(0, 12).map(([, s], idx) => ({
        label: s.label.replace("robot/", "").replace("controllers/", "c"),
        t: s.t,
        y: s.temp,
        color: PALETTE[idx % PALETTE.length],
      }));
    }
    if (mode === "magnetic") {
      const mag = Object.values(ts.magnetic || {})[0];
      if (!mag) return [];
      return [
        { label: "mx", t: mag.t, y: mag.x, color: "#f43f5e" },
        { label: "my", t: mag.t, y: mag.y, color: "#10b981" },
        { label: "mz", t: mag.t, y: mag.z, color: "#38bdf8" },
      ];
    }
    return [];
  }

  function drawAllCharts() {
    for (let slot = 1; slot <= 4; slot++) {
      drawSingleChart(slot);
    }
  }

  function drawSingleChart(slotIndex) {
    const canvas = el.chartCanvases[slotIndex];
    const legendEl = el.chartLegends[slotIndex];
    if (!canvas) return;

    const { width, height, dpr } = resizeCanvasToDisplaySize(canvas);
    const ctx = canvas.getContext("2d");
    ctx.save();
    ctx.scale(dpr, dpr);
    const w = width / dpr;
    const h = height / dpr;

    ctx.fillStyle = "#0f1624";
    ctx.fillRect(0, 0, w, h);

    const seriesList = buildChartSeries(slotIndex);
    if (seriesList.length === 0 || state.duration <= 0) {
      ctx.fillStyle = "#64748b";
      ctx.font = "11px sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("No data for selected telemetry channel in this bag", w / 2, h / 2);
      legendEl.innerHTML = `<span class="legend-item">No matching topic in bag</span>`;
      ctx.restore();
      return;
    }

    const padL = 44, padR = 10, padT = 10, padB = 18;
    const plotW = Math.max(10, w - padL - padR);
    const plotH = Math.max(10, h - padT - padB);

    // Find global min/max Y across series
    let minY = Infinity, maxY = -Infinity;
    seriesList.forEach((s) => {
      for (let i = 0; i < s.y.length; i++) {
        const v = s.y[i];
        if (v !== null && isFinite(v)) {
          if (v < minY) minY = v;
          if (v > maxY) maxY = v;
        }
      }
    });
    if (!isFinite(minY) || !isFinite(maxY)) {
      minY = -1;
      maxY = 1;
    }
    if (Math.abs(maxY - minY) < 1e-4) {
      minY -= 1;
      maxY += 1;
    } else {
      const margin = (maxY - minY) * 0.08;
      minY -= margin;
      maxY += margin;
    }

    const xOf = (t) => padL + (t / state.duration) * plotW;
    const yOf = (v) => padT + (1 - (v - minY) / (maxY - minY)) * plotH;

    // Horizontal grid lines & Y labels
    ctx.strokeStyle = "#1a2438";
    ctx.fillStyle = "#64748b";
    ctx.font = "9.5px monospace";
    ctx.textAlign = "right";
    for (let g = 0; g <= 3; g++) {
      const frac = g / 3;
      const gy = padT + frac * plotH;
      const val = maxY - frac * (maxY - minY);
      ctx.beginPath();
      ctx.moveTo(padL, gy);
      ctx.lineTo(padL + plotW, gy);
      ctx.stroke();
      ctx.fillText(val.toFixed(2), padL - 4, gy + 3);
    }

    // Time axis labels
    ctx.textAlign = "center";
    for (let g = 0; g <= 4; g++) {
      const tVal = (g / 4) * state.duration;
      const gx = xOf(tVal);
      ctx.fillText(`${tVal.toFixed(1)}s`, gx, h - 4);
    }

    // Draw series lines
    const isProgressive = state.chartStreamMode === "progressive";
    const legendParts = [];

    seriesList.forEach((s) => {
      if (!s.t || s.t.length === 0) return;
      const curIdx = bisectRightLe(s.t, state.currentTime);
      const curVal = s.y[curIdx];

      legendParts.push(`
        <span class="legend-item">
          <span class="legend-swatch" style="background:${s.color}"></span>
          <span>${s.label}:</span>
          <span class="legend-val">${curVal !== undefined && curVal !== null ? Number(curVal).toFixed(3) : "--"}</span>
        </span>
      `);

      // If progressive mode, draw faint full trace and bright trace up to curIdx
      if (isProgressive) {
        ctx.strokeStyle = "rgba(148, 163, 184, 0.12)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let i = 0; i < s.t.length; i++) {
          const px = xOf(s.t[i]);
          const py = yOf(s.y[i]);
          if (i === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        }
        ctx.stroke();
      }

      const endIdx = isProgressive ? curIdx : s.t.length - 1;
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      for (let i = 0; i <= endIdx; i++) {
        const px = xOf(s.t[i]);
        const py = yOf(s.y[i]);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();

      // Current value dot on playhead
      if (curIdx >= 0 && curVal !== null && curVal !== undefined) {
        const cx = xOf(s.t[curIdx]);
        const cy = yOf(curVal);
        ctx.fillStyle = s.color;
        ctx.beginPath();
        ctx.arc(cx, cy, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    });

    // Synchronized playhead vertical line
    const playX = xOf(state.currentTime);
    ctx.strokeStyle = "#f8fafc";
    ctx.lineWidth = 1.2;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(playX, padT);
    ctx.lineTo(playX, padT + plotH);
    ctx.stroke();
    ctx.setLineDash([]);

    legendEl.innerHTML = legendParts.join("");
    ctx.restore();
  }

  // All-Topics Synchronized Live Inspector
  function summarizeTopicData(cat, data) {
    if (!data) return "";
    if (cat === "odometry" && data.position) {
      const p = data.position;
      const v = data.linear || {};
      return `pos=(${p.x?.toFixed(2)}, ${p.y?.toFixed(2)}, ${p.z?.toFixed(2)}) yaw=${data.euler_deg?.yaw?.toFixed(1)}° vx=${v.x?.toFixed(2)}m/s`;
    }
    if (cat === "imu" && data.euler_deg) {
      const e = data.euler_deg;
      const a = data.linear_acceleration || {};
      return `RPY=(${e.roll?.toFixed(1)}°, ${e.pitch?.toFixed(1)}°, ${e.yaw?.toFixed(1)}°) acc=(${a.x?.toFixed(2)}, ${a.y?.toFixed(2)}, ${a.z?.toFixed(2)})`;
    }
    if (cat === "cmd_vel" && data.linear) {
      return `linear.x=${data.linear.x?.toFixed(3)} m/s, angular.z=${data.angular?.z?.toFixed(3)} rad/s`;
    }
    if (cat === "temperature") {
      return `temperature = ${data.temperature?.toFixed(2)} °C`;
    }
    if (cat === "battery") {
      return `voltage=${data.voltage?.toFixed(2)}V, current=${data.current?.toFixed(2)}A, charge=${data.percentage?.toFixed(1)}%`;
    }
    if (cat === "joints" && data.name) {
      const pairs = data.name.slice(0, 4).map((n, i) => `${n.replace("_joint", "")}:${data.position?.[i]?.toFixed(2)}`);
      return `${data.name.length} joints [${pairs.join(", ")}${data.name.length > 4 ? ", ..." : ""}]`;
    }
    if (cat === "laserscan") {
      return `${data.valid_count}/${data.count} valid rays, min=${data.min_dist?.toFixed(2) ?? "--"}m, max=${data.max_dist?.toFixed(2) ?? "--"}m`;
    }
    if (cat === "pointcloud") {
      return `${data.width}x${data.height} (${data.num_points} pts), frame=${data.header?.frame_id}`;
    }
    if (cat === "sensor_state") {
      return `L_enc=${data.left_encoder}, R_enc=${data.right_encoder}, batt=${data.battery?.toFixed(2)}V, torque=${data.torque}`;
    }
    if (cat === "camera_rgb" || cat === "camera_depth") {
      return `Frame #${(data.frame_index ?? 0) + 1} (msg_id=${data.msg_id})`;
    }
    if (data.json) {
      return JSON.stringify(data.json).slice(0, 140);
    }
    return JSON.stringify(data).slice(0, 140);
  }

  function matchesCategoryFilter(cat, filterVal) {
    if (filterVal === "all") return true;
    if (filterVal === "camera") return cat === "camera_rgb" || cat === "camera_depth" || cat === "camera_info";
    if (filterVal === "spatial") return cat === "pointcloud" || cat === "laserscan" || cat === "odometry";
    if (filterVal === "control") return cat === "imu" || cat === "cmd_vel" || cat === "joints" || cat === "sensor_state";
    if (filterVal === "system") return cat === "battery" || cat === "temperature" || cat === "state_json" || cat === "tf" || cat === "diagnostics" || cat === "magnetic" || cat === "string";
    return true;
  }

  function renderTopicsInspector() {
    if (!state.bag || !state.bag.topics) {
      el.topicsInspectorList.innerHTML = `<div class="empty-state">No topics loaded</div>`;
      return;
    }

    const snapTopics = (state.syncSnapshot && state.syncSnapshot.topics) || {};
    const q = state.topicSearch.trim().toLowerCase();
    const catFilter = state.topicCategory;

    const filtered = state.bag.topics.filter((t) => {
      if (!matchesCategoryFilter(t.category, catFilter)) return false;
      if (q && !t.name.toLowerCase().includes(q) && !t.type.toLowerCase().includes(q)) return false;
      return true;
    });

    el.inspectorTopicsCount.textContent = `${filtered.length} / ${state.bag.topics.length} topics`;

    const fragment = document.createDocumentFragment();
    filtered.forEach((t) => {
      const live = snapTopics[t.name];
      const isExpanded = state.expandedTopics.has(t.name);

      const row = document.createElement("div");
      row.className = "topic-row";

      const seqStr = live ? `${live.seq_idx + 1}/${t.message_count}` : `0/${t.message_count}`;
      const deltaStr = live ? `Δt ${live.delta_ms >= 0 ? "+" : ""}${live.delta_ms.toFixed(0)}ms` : "--";
      const summaryText = live ? summarizeTopicData(t.category, live.data) : "No message at current timestamp";

      row.innerHTML = `
        <div class="topic-row-header">
          <span class="topic-name">${t.name}</span>
          <div class="topic-badges">
            <span class="topic-type-badge">${t.type}</span>
            <span class="topic-hz-badge">${t.hz} Hz</span>
            <span>${seqStr}</span>
            <span class="sync-delta-badge">${deltaStr}</span>
          </div>
        </div>
        <div class="topic-summary-preview">${summaryText}</div>
        ${isExpanded && live ? `<pre class="topic-json-detail">${JSON.stringify(live.data, null, 2)}</pre>` : ""}
      `;

      row.querySelector(".topic-row-header").addEventListener("click", () => {
        if (state.expandedTopics.has(t.name)) state.expandedTopics.delete(t.name);
        else state.expandedTopics.add(t.name);
        renderTopicsInspector();
      });

      fragment.appendChild(row);
    });

    el.topicsInspectorList.innerHTML = "";
    el.topicsInspectorList.appendChild(fragment);
  }

  // Synchronized Snapshot Fetching (/api/sync)
  function requestSyncSnapshot(relTime, force = false) {
    if (!state.bag) return;
    const nowMs = performance.now();
    if (!force) {
      if (state.syncFetchInFlight) return;
      if (Math.abs(relTime - state.lastSyncFetchTime) < 0.025 && nowMs - state.lastSyncWallMs < 90) {
        return;
      }
    }

    state.syncFetchInFlight = true;
    state.lastSyncFetchTime = relTime;
    state.lastSyncWallMs = nowMs;

    fetch(`/api/sync?t=${relTime.toFixed(4)}`)
      .then((r) => (r.ok ? r.json() : null))
      .then((data) => {
        state.syncFetchInFlight = false;
        if (!data) return;
        state.syncSnapshot = data;
        drawSpatialCanvas();
        renderTopicsInspector();
      })
      .catch(() => {
        state.syncFetchInFlight = false;
      });
  }

  // Master Clock & Transport Control
  function seekToTime(newTime, forceSync = false) {
    if (!state.bag) return;
    const clamped = Math.max(0.0, Math.min(state.duration, newTime));
    state.currentTime = clamped;

    el.timelineSlider.value = String(clamped);
    el.currentTimeDisplay.textContent = formatSec(clamped);
    el.epochTimeDisplay.textContent = formatEpochTime(state.bag.start_ns, clamped);

    updateCameraFrames(clamped);
    drawAllCharts();
    drawSpatialCanvas();
    requestSyncSnapshot(clamped, forceSync);
  }

  function setPlaying(playing) {
    state.playing = playing;
    el.playPauseBtn.classList.toggle("playing", playing);
    el.playIcon.classList.toggle("hidden", playing);
    el.pauseIcon.classList.toggle("hidden", !playing);
    el.playBtnLabel.textContent = playing ? "Pause" : "Play";
    if (playing) {
      if (state.currentTime >= state.duration - 0.01) {
        state.currentTime = 0.0;
      }
      state.lastRafTs = performance.now();
    } else {
      requestSyncSnapshot(state.currentTime, true);
    }
  }

  function animationLoop(now) {
    if (state.playing && state.bag) {
      const dtSec = ((now - state.lastRafTs) / 1000.0) * state.playSpeed;
      state.lastRafTs = now;
      let nextT = state.currentTime + dtSec;
      if (nextT >= state.duration) {
        if (state.loop) {
          nextT = nextT % state.duration;
        } else {
          nextT = state.duration;
          setPlaying(false);
        }
      }
      seekToTime(nextT, false);
    } else {
      state.lastRafTs = now;
    }
    requestAnimationFrame(animationLoop);
  }

  // Upload & Bag Selection Modal Logic
  function renderDiscoveredBagsList() {
    el.discoveredBagsList.innerHTML = "";
    const curPath = state.bag ? state.bag.source_path : null;

    state.discoveredBags.forEach((b) => {
      const isCurrent = curPath === b.path;
      const card = document.createElement("div");
      card.className = `bag-card ${isCurrent ? "active" : ""}`;
      card.innerHTML = `
        <div class="bag-card-top">
          <span class="bag-card-name">${b.name}</span>
          <span class="badge badge-robot">${b.robot_hint}</span>
        </div>
        <div class="bag-card-path">${b.path}</div>
        <div class="bag-card-footer">
          <span>${b.size_mb} MB · ${b.source}</span>
          <button class="btn btn-xs ${isCurrent ? "btn-toggle active" : "btn-primary"}">
            ${isCurrent ? "Loaded" : "Open Bag"}
          </button>
        </div>
      `;
      card.addEventListener("click", () => loadBagByPath(b.path));
      el.discoveredBagsList.appendChild(card);
    });
  }

  function loadBagByPath(path) {
    if (!path) return;
    fetch("/api/load", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path }),
    })
      .then((r) => r.json())
      .then((res) => {
        if (res.error) {
          alert(res.error);
          return;
        }
        if (res.loaded && res.bag) {
          applyLoadedBag(res.bag, res.bags);
          el.bagModalBackdrop.classList.add("hidden");
        }
      })
      .catch((err) => alert("Failed to load bag: " + err));
  }

  async function handleFilesUpload(fileList) {
    if (!fileList || fileList.length === 0) return;
    const files = Array.from(fileList).filter(
      (f) =>
        f.name.endsWith(".db3") ||
        f.name === "metadata.yaml" ||
        f.name.endsWith(".json")
    );
    const hasDb3 = files.some((f) => f.name.endsWith(".db3"));
    if (!hasDb3) {
      alert("Please select a ROS 2 .db3 bag file or a folder containing a .db3 file.");
      return;
    }

    // Derive bag folder name from webkitRelativePath or .db3 filename
    let bagName = "uploaded_bag";
    const firstFile = files.find((f) => f.name.endsWith(".db3")) || files[0];
    if (firstFile.webkitRelativePath && firstFile.webkitRelativePath.includes("/")) {
      bagName = firstFile.webkitRelativePath.split("/")[0];
    } else {
      bagName = firstFile.name.replace(/(_0)?\.db3$/i, "");
    }

    el.uploadProgressWrap.classList.remove("hidden");
    const totalBytes = files.reduce((acc, f) => acc + f.size, 0);
    let uploadedBytesBefore = 0;

    for (let i = 0; i < files.length; i++) {
      const file = files[i];
      const isLast = i === files.length - 1;
      await new Promise((resolve, reject) => {
        const xhr = new XMLHttpRequest();
        const q = new URLSearchParams({
          bag_name: bagName,
          file_name: file.name,
          activate: isLast ? "1" : "0",
        });
        xhr.open("POST", `/api/upload_file?${q.toString()}`, true);

        xhr.upload.onprogress = (e) => {
          if (e.lengthComputable) {
            const currentTotal = uploadedBytesBefore + e.loaded;
            const pct = Math.min(100, (currentTotal / Math.max(1, totalBytes)) * 100);
            el.uploadProgressFill.style.width = `${pct.toFixed(1)}%`;
            const mbDone = (currentTotal / (1024 * 1024)).toFixed(1);
            const mbTotal = (totalBytes / (1024 * 1024)).toFixed(1);
            el.uploadProgressText.textContent = `Uploading ${file.name} (${mbDone} / ${mbTotal} MB — ${pct.toFixed(0)}%)...`;
          }
        };

        xhr.onload = () => {
          uploadedBytesBefore += file.size;
          if (xhr.status >= 200 && xhr.status < 300) {
            const resp = JSON.parse(xhr.responseText);
            if (isLast && resp.loaded && resp.bag) {
              applyLoadedBag(resp.bag, resp.bags);
              el.uploadProgressWrap.classList.add("hidden");
              el.bagModalBackdrop.classList.add("hidden");
            }
            resolve(resp);
          } else {
            reject(new Error(xhr.responseText || `HTTP ${xhr.status}`));
          }
        };
        xhr.onerror = () => reject(new Error("Network error during upload"));
        xhr.send(file);
      }).catch((err) => {
        alert("Upload error: " + err.message);
        el.uploadProgressWrap.classList.add("hidden");
      });
    }
  }

  // Event Listeners Setup
  function setupEventListeners() {
    // Workspace layout tabs
    document.querySelectorAll(".layout-tab").forEach((tab) => {
      tab.addEventListener("click", () => {
        document.querySelectorAll(".layout-tab").forEach((t) => t.classList.remove("active"));
        tab.classList.add("active");
        const layout = tab.dataset.layout;
        el.workspaceGrid.className = `workspace layout-${layout}`;
        setTimeout(() => {
          drawSpatialCanvas();
          drawAllCharts();
        }, 30);
      });
    });

    // Camera view mode (single vs multi)
    el.camModeControl.querySelectorAll(".seg-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        el.camModeControl.querySelectorAll(".seg-btn").forEach((b) => b.classList.remove("active"));
        btn.classList.add("active");
        state.camViewMode = btn.dataset.mode;
        state.soloCamId = null;
        if (state.camViewMode === "single" && state.selectedCamIds.size > 1) {
          const first = state.selectedCamIds.values().next().value;
          state.selectedCamIds.clear();
          state.selectedCamIds.add(first);
        }
        renderCameraSelectorPills();
        rebuildCameraGrid();
        updateCameraFrames(state.currentTime);
      });
    });

    el.camGridColsSelect.addEventListener("change", () => {
      state.camGridCols = el.camGridColsSelect.value;
      rebuildCameraGrid();
      updateCameraFrames(state.currentTime);
    });

    el.depthColormapSelect.addEventListener("change", () => {
      state.depthColormap = el.depthColormapSelect.value;
      updateCameraFrames(state.currentTime);
    });

    el.selectAllRgbBtn.addEventListener("click", () => {
      if (!state.bag) return;
      state.soloCamId = null;
      state.selectedCamIds.clear();
      state.bag.cameras.filter((c) => !c.is_depth).forEach((c) => state.selectedCamIds.add(c.id));
      if (state.selectedCamIds.size === 0) {
        state.bag.cameras.forEach((c) => state.selectedCamIds.add(c.id));
      }
      renderCameraSelectorPills();
      rebuildCameraGrid();
      updateCameraFrames(state.currentTime);
    });

    el.selectAllDepthBtn.addEventListener("click", () => {
      if (!state.bag) return;
      const depthCams = state.bag.cameras.filter((c) => c.is_depth);
      if (depthCams.length === 0) return;
      state.soloCamId = null;
      state.selectedCamIds.clear();
      depthCams.forEach((c) => state.selectedCamIds.add(c.id));
      renderCameraSelectorPills();
      rebuildCameraGrid();
      updateCameraFrames(state.currentTime);
    });

    el.selectAllCamsBtn.addEventListener("click", () => {
      if (!state.bag) return;
      state.soloCamId = null;
      state.camViewMode = "multi";
      el.camModeControl.querySelectorAll(".seg-btn").forEach((b) => {
        b.classList.toggle("active", b.dataset.mode === "multi");
      });
      state.selectedCamIds.clear();
      state.bag.cameras.forEach((c) => state.selectedCamIds.add(c.id));
      renderCameraSelectorPills();
      rebuildCameraGrid();
      updateCameraFrames(state.currentTime);
    });

    // Spatial view tabs & interaction
    el.spatialTabs.querySelectorAll(".spatial-tab").forEach((btn) => {
      btn.addEventListener("click", () => {
        if (btn.classList.contains("disabled")) return;
        setSpatialMode(btn.dataset.view);
      });
    });

    el.pcColorModeSelect.addEventListener("change", () => {
      state.pcColorMode = el.pcColorModeSelect.value;
      drawSpatialCanvas();
    });

    el.resetSpatialViewBtn.addEventListener("click", () => {
      state.orbit.yaw = -0.65;
      state.orbit.pitch = 0.45;
      state.orbit.distance = 4.2;
      state.orbit.panX = 0;
      state.orbit.panY = 0;
      state.orbit.zoom2D = 1.0;
      state.orbit.offset2DX = 0;
      state.orbit.offset2DY = 0;
      drawSpatialCanvas();
    });

    // Spatial canvas mouse orbit/pan/zoom
    el.spatialCanvas.addEventListener("mousedown", (e) => {
      state.orbit.dragging = true;
      state.orbit.dragButton = e.button;
      state.orbit.lastX = e.clientX;
      state.orbit.lastY = e.clientY;
    });
    window.addEventListener("mouseup", () => {
      state.orbit.dragging = false;
    });
    window.addEventListener("mousemove", (e) => {
      if (!state.orbit.dragging) return;
      const dx = e.clientX - state.orbit.lastX;
      const dy = e.clientY - state.orbit.lastY;
      state.orbit.lastX = e.clientX;
      state.orbit.lastY = e.clientY;

      if (state.spatialMode === "pointcloud") {
        if (e.shiftKey || state.orbit.dragButton === 2) {
          state.orbit.panX += dx;
          state.orbit.panY += dy;
        } else {
          state.orbit.yaw += dx * 0.008;
          state.orbit.pitch = Math.max(-1.4, Math.min(1.4, state.orbit.pitch + dy * 0.008));
        }
      } else {
        state.orbit.offset2DX += dx;
        state.orbit.offset2DY += dy;
      }
      drawSpatialCanvas();
    });
    el.spatialCanvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      const factor = e.deltaY > 0 ? 0.9 : 1.1;
      if (state.spatialMode === "pointcloud") {
        state.orbit.distance = Math.max(0.6, Math.min(20.0, state.orbit.distance / factor));
      } else {
        state.orbit.zoom2D = Math.max(0.25, Math.min(10.0, state.orbit.zoom2D * factor));
      }
      drawSpatialCanvas();
    });

    // Telemetry chart controls & click-to-scrub
    el.chartStreamModeControl.querySelectorAll(".seg-btn").forEach((btn) => {
      btn.addEventListener("click", () => {
        el.chartStreamModeControl.querySelectorAll(".seg-btn").forEach((b) => b.classList.remove("active"));
        btn.classList.add("active");
        state.chartStreamMode = btn.dataset.mode;
        drawAllCharts();
      });
    });

    [1, 2, 3, 4].forEach((slot) => {
      el.chartSelects[slot].addEventListener("change", () => {
        state.chartModes[slot] = el.chartSelects[slot].value;
        drawSingleChart(slot);
      });

      const canvas = el.chartCanvases[slot];
      let scrubbingChart = false;
      const scrubFromChartEvent = (e) => {
        if (!state.bag || state.duration <= 0) return;
        const rect = canvas.getBoundingClientRect();
        const padL = 44, padR = 10;
        const plotW = Math.max(10, rect.width - padL - padR);
        const relX = e.clientX - rect.left - padL;
        const frac = Math.max(0, Math.min(1, relX / plotW));
        seekToTime(frac * state.duration, true);
      };
      canvas.addEventListener("mousedown", (e) => {
        scrubbingChart = true;
        scrubFromChartEvent(e);
      });
      window.addEventListener("mousemove", (e) => {
        if (scrubbingChart) scrubFromChartEvent(e);
      });
      window.addEventListener("mouseup", () => {
        scrubbingChart = false;
      });
    });

    // Topics Inspector filtering
    el.topicSearchInput.addEventListener("input", () => {
      state.topicSearch = el.topicSearchInput.value;
      renderTopicsInspector();
    });
    el.topicCategoryFilter.addEventListener("change", () => {
      state.topicCategory = el.topicCategoryFilter.value;
      renderTopicsInspector();
    });

    // Transport Controls
    el.playPauseBtn.addEventListener("click", () => setPlaying(!state.playing));
    el.jumpStartBtn.addEventListener("click", () => {
      setPlaying(false);
      seekToTime(0.0, true);
    });
    el.stepBackBtn.addEventListener("click", () => {
      setPlaying(false);
      seekToTime(state.currentTime - 0.05, true);
    });
    el.stepFwdBtn.addEventListener("click", () => {
      setPlaying(false);
      seekToTime(state.currentTime + 0.05, true);
    });
    el.timelineSlider.addEventListener("input", () => {
      const t = parseFloat(el.timelineSlider.value) || 0;
      seekToTime(t, false);
    });
    el.timelineSlider.addEventListener("change", () => {
      const t = parseFloat(el.timelineSlider.value) || 0;
      seekToTime(t, true);
    });
    el.playbackSpeedSelect.addEventListener("change", () => {
      state.playSpeed = parseFloat(el.playbackSpeedSelect.value) || 1.0;
    });
    el.loopToggleBtn.addEventListener("click", () => {
      state.loop = !state.loop;
      el.loopToggleBtn.classList.toggle("active", state.loop);
      el.loopToggleBtn.textContent = `Loop: ${state.loop ? "ON" : "OFF"}`;
    });

    // Keyboard shortcuts
    window.addEventListener("keydown", (e) => {
      if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
      if (e.code === "Space") {
        e.preventDefault();
        setPlaying(!state.playing);
      } else if (e.code === "ArrowLeft") {
        e.preventDefault();
        setPlaying(false);
        seekToTime(state.currentTime - 0.05, true);
      } else if (e.code === "ArrowRight") {
        e.preventDefault();
        setPlaying(false);
        seekToTime(state.currentTime + 0.05, true);
      }
    });

    // Modal controls
    el.openBagModalBtn.addEventListener("click", () => {
      fetch("/api/bags")
        .then((r) => r.json())
        .then((data) => {
          if (data.bags) {
            state.discoveredBags = data.bags;
            renderDiscoveredBagsList();
          }
        });
      el.bagModalBackdrop.classList.remove("hidden");
    });
    el.closeBagModalBtn.addEventListener("click", () => {
      el.bagModalBackdrop.classList.add("hidden");
    });
    el.bagModalBackdrop.addEventListener("click", (e) => {
      if (e.target === el.bagModalBackdrop) el.bagModalBackdrop.classList.add("hidden");
    });

    el.loadCustomPathBtn.addEventListener("click", () => {
      loadBagByPath(el.customBagPathInput.value.trim());
    });

    el.uploadDb3Input.addEventListener("change", (e) => handleFilesUpload(e.target.files));
    el.uploadFolderInput.addEventListener("change", (e) => handleFilesUpload(e.target.files));

    // Drag & Drop upload
    el.uploadDropZone.addEventListener("dragover", (e) => {
      e.preventDefault();
      el.uploadDropZone.classList.add("dragover");
    });
    el.uploadDropZone.addEventListener("dragleave", () => {
      el.uploadDropZone.classList.remove("dragover");
    });
    el.uploadDropZone.addEventListener("drop", (e) => {
      e.preventDefault();
      el.uploadDropZone.classList.remove("dragover");
      if (e.dataTransfer && e.dataTransfer.files) {
        handleFilesUpload(e.dataTransfer.files);
      }
    });

    window.addEventListener("resize", () => {
      drawSpatialCanvas();
      drawAllCharts();
    });
  }

  // Bootstrap Application
  function init() {
    setupEventListeners();
    fetch("/api/state")
      .then((r) => r.json())
      .then((data) => {
        if (data.bags) state.discoveredBags = data.bags;
        if (data.loaded && data.bag) {
          applyLoadedBag(data.bag, data.bags);
        } else {
          renderDiscoveredBagsList();
          el.bagModalBackdrop.classList.remove("hidden");
        }
      })
      .catch((err) => console.error("Failed to fetch initial state:", err));

    requestAnimationFrame(animationLoop);
  }

  init();
})();
