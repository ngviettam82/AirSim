/**
 * Main Application Orchestrator for Web Companion GCS.
 * Manages WebSocket telemetry stream, user interface interactions,
 * flight command dispatches, survey calculations, and click-to-track.
 */

let pfd = null;
let radar = null;
let mapManager = null;
let ws = null;
let currentTelemetry = {};
let latestSurveyPlan = null;

document.addEventListener('DOMContentLoaded', () => {
  // 1. Initialize Subsystems
  pfd = new PrimaryFlightDisplay('pfd-canvas');
  radar = new ObstacleRadar();
  mapManager = new MapManager('map-container');

  // 2. Setup WebSocket Telemetry Connection
  connectWebSocket();

  // 3. Setup Button Event Handlers
  setupCommandButtons();
  setupSurveyDrawer();
  setupCameraClickToTrack();
});

// =============================================================================
// WEBSOCKET TELEMETRY CLIENT
// =============================================================================
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

  const connBadge = document.getElementById('conn-badge');

  try {
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      if (connBadge) {
        connBadge.textContent = 'CONNECTED';
        connBadge.className = 'badge badge-connected';
      }
      showToast('Connected to AirSim Autonomy Gateway');
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        currentTelemetry = data;
        updateUI(data);
      } catch (err) {
        console.error('Error parsing telemetry JSON:', err);
      }
    };

    ws.onclose = () => {
      if (connBadge) {
        connBadge.textContent = 'DISCONNECTED';
        connBadge.className = 'badge badge-disconnected';
      }
      // Reconnect after 2 seconds
      setTimeout(connectWebSocket, 2000);
    };

    ws.onerror = (err) => {
      console.warn('WebSocket error:', err);
      ws.close();
    };
  } catch (e) {
    setTimeout(connectWebSocket, 3000);
  }
}

function updateUI(tel) {
  // 1. Top Bar Telemetry Updates
  setText('tel-alt', (tel.alt_agl || 0.0).toFixed(1));
  setText('tel-spd', (tel.groundspeed || 0.0).toFixed(1));
  setText('tel-hdg', Math.round((tel.heading_deg || 0) + 360) % 360 + '°');
  setText('tel-climb', (tel.vertical_speed || 0.0).toFixed(1));

  // Battery
  setText('tel-bat-pct', Math.round(tel.battery_pct || 0) + '%');
  setText('tel-bat-volt', (tel.battery_voltage || 0.0).toFixed(1) + 'V');
  setText('tel-rth-margin', (tel.smart_rth_margin_pct >= 0 ? '+' : '') + Math.round(tel.smart_rth_margin_pct || 0) + '% RTH');

  // GPS
  setText('tel-sats', tel.satellites || 0);
  setText('tel-gps-fix', tel.gps_fix_type || '3D Fix');

  // Arm status badge
  const armBadge = document.getElementById('arm-badge');
  if (armBadge) {
    if (tel.armed) {
      armBadge.textContent = 'ARMED';
      armBadge.className = 'badge badge-armed';
      const armBtn = document.getElementById('btn-arm');
      if (armBtn) armBtn.textContent = 'DISARM VEHICLE';
    } else {
      armBadge.textContent = 'DISARMED';
      armBadge.className = 'badge badge-disarmed';
      const armBtn = document.getElementById('btn-arm');
      if (armBtn) armBtn.textContent = 'ARM VEHICLE';
    }
  }

  // Flight mode badge
  setText('mode-badge', tel.flight_mode || 'OFFBOARD');

  // Navigation Tier Badge
  const navBadge = document.getElementById('nav-tier-badge');
  if (navBadge && tel.navigation_tier) {
    const tier = tel.navigation_tier;
    navBadge.textContent = tier.replace('_', ' ').toUpperCase();
    navBadge.className = 'badge ' + (tier.includes('Tier0') ? 'badge-tier0' : (tier.includes('Tier4') ? 'badge-disconnected' : 'badge-armed'));
  }

  // Sync Algorithm Dropdown if changed externally
  const algoSelect = document.getElementById('select-algorithm');
  if (algoSelect && tel.active_algorithm !== undefined && document.activeElement !== algoSelect) {
    algoSelect.value = tel.active_algorithm;
  }

  // 2. Update Subsystems
  if (pfd) pfd.update(tel);
  if (radar) radar.update(tel);
  if (mapManager && tel.lat && tel.lon) {
    mapManager.updateVehicle(tel.lat, tel.lon, tel.heading_deg || 0);
  }
}

function setText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}

// =============================================================================
// COMMAND BAR DISPATCHERS
// =============================================================================
function setupCommandButtons() {
  document.getElementById('btn-arm').addEventListener('click', () => {
    const isArmed = currentTelemetry.armed || false;
    const action = isArmed ? 'disarm' : 'arm';
    if (!isArmed || confirm('Confirm DISARMING vehicle in flight?')) {
      postApi(`/api/flight/${action}`, {}, `${action.toUpperCase()} command sent`);
    }
  });

  document.getElementById('btn-takeoff').addEventListener('click', () => {
    const alt = prompt('Enter Takeoff Altitude AGL (meters):', '10');
    if (alt && !isNaN(alt)) {
      postApi('/api/flight/takeoff', { altitude: parseFloat(alt) }, `Takeoff commanded to ${alt}m`);
    }
  });

  document.getElementById('btn-land').addEventListener('click', () => {
    if (confirm('Confirm LANDING vehicle in place?')) {
      postApi('/api/flight/land', {}, 'Landing in-place commanded');
    }
  });

  document.getElementById('btn-rth').addEventListener('click', () => {
    if (confirm('Confirm RETURN TO HOME (RTH)?')) {
      postApi('/api/flight/rth', {}, 'Return to Home commanded');
    }
  });

  document.getElementById('btn-hold').addEventListener('click', () => {
    postApi('/api/flight/hold', {}, 'EMERGENCY HOLD: Holding station');
  });

  document.getElementById('select-algorithm').addEventListener('change', (e) => {
    const algo = e.target.value;
    postApi('/api/autonomy/set_algorithm', { algorithm: algo }, `Autonomy Mission set to: '${algo || "Standby"}'`);
  });
}

// =============================================================================
// SURVEY MISSION DRAWER & OPTIMAL SWEEP
// =============================================================================
function setupSurveyDrawer() {
  const altInput = document.getElementById('input-survey-alt');
  const spdInput = document.getElementById('input-survey-spd');

  altInput.addEventListener('input', (e) => {
    setText('val-survey-alt', e.target.value + ' m');
  });

  spdInput.addEventListener('input', (e) => {
    setText('val-survey-spd', parseFloat(e.target.value).toFixed(1) + ' m/s');
  });

  document.getElementById('btn-draw-polygon').addEventListener('click', () => {
    if (mapManager) mapManager.startDrawingPolygon();
  });

  document.getElementById('btn-clear-survey').addEventListener('click', () => {
    if (mapManager) mapManager.clearSurveyPlan();
  });

  document.getElementById('btn-calc-survey').addEventListener('click', async () => {
    if (!mapManager || mapManager.polygonPoints.length < 3) {
      showToast('Please draw a polygon with at least 3 points on the map first.');
      return;
    }

    const payload = {
      polygon: mapManager.polygonPoints.map(p => ({ lat: p[0], lon: p[1] })),
      altitude: parseFloat(altInput.value),
      speed: parseFloat(spdInput.value),
      forward_overlap: parseFloat(document.getElementById('input-fwd-overlap').value) / 100.0,
      side_overlap: parseFloat(document.getElementById('input-side-overlap').value) / 100.0
    };

    showToast('Calculating optimal Boustrophedon sweep angle...');

    try {
      const resp = await fetch('/api/mission/plan_survey', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      const plan = await resp.json();

      if (plan.success) {
        latestSurveyPlan = plan;
        mapManager.renderSurveyPlan(plan);

        // Populate Summary Card
        setText('stat-gsd', plan.optical.gsd_cm + ' cm/px');
        setText('stat-strips', plan.mission.strip_count);
        setText('stat-angle', plan.mission.optimal_angle_deg + '°');
        setText('stat-dist', plan.mission.total_distance_m + ' m');
        
        const m = Math.floor(plan.mission.estimated_time_s / 60);
        const s = Math.round(plan.mission.estimated_time_s % 60);
        setText('stat-time', `${m}m ${s}s`);
        setText('stat-photos', plan.mission.photo_count);

        document.getElementById('survey-summary-card').classList.remove('hidden');
        showToast(`Optimal survey generated: ${plan.mission.strip_count} strips at ${plan.mission.optimal_angle_deg}°`);
      } else {
        showToast('Survey calculation failed: ' + (plan.error || 'Unknown error'));
      }
    } catch (err) {
      showToast('Network error calculating survey: ' + err);
    }
  });

  document.getElementById('btn-execute-survey').addEventListener('click', () => {
    if (!latestSurveyPlan) return;
    postApi('/api/autonomy/set_algorithm', { algorithm: 'photogrammetry_survey' }, '🚀 Survey mission uploaded and executed!');
    toggleSurveyDrawer();
  });
}

// =============================================================================
// CAMERA LIVE STREAM & CLICK-TO-TRACK VIDEO
// =============================================================================
function setupCameraClickToTrack() {
  const streamImg = document.getElementById('mjpeg-stream');
  const crosshair = document.getElementById('tracking-crosshair');
  const trackStatus = document.getElementById('track-status');

  streamImg.addEventListener('click', (e) => {
    const rect = streamImg.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    // Center crosshair on click
    crosshair.style.left = `${clickX}px`;
    crosshair.style.top = `${clickY}px`;
    crosshair.classList.remove('hidden');

    // Normalized coordinates (-1.0 to 1.0)
    const normX = (clickX / rect.width) * 2.0 - 1.0;
    const normY = (clickY / rect.height) * 2.0 - 1.0;

    // Approximate body FLU target coordinates (Forward: 15m standoff, Left/Right: normX, Up/Down: -normY)
    const targetForward = 15.0;
    const targetLeft = -normX * 8.0;
    const targetUp = -normY * 6.0;

    trackStatus.textContent = `TARGET LOCKED: [Fwd: 15.0m, Left: ${targetLeft.toFixed(1)}m]`;

    postApi('/api/autonomy/target', {
      x: targetForward,
      y: targetLeft,
      z: targetUp,
      frame_id: 'body'
    }, 'Tracking target locked!');
  });

  document.getElementById('btn-clear-target').addEventListener('click', () => {
    crosshair.classList.add('hidden');
    trackStatus.textContent = 'Status: Click video target to track';
    showToast('Target tracking cleared');
  });
}

function onCameraError() {
  const status = document.getElementById('track-status');
  if (status) status.textContent = 'AirSim camera offline (Port 8000)';
}

function onCameraLoad() {
  const status = document.getElementById('track-status');
  if (status && status.textContent.includes('offline')) {
    status.textContent = 'Status: Live stream connected';
  }
}

// =============================================================================
// UTILITIES
// =============================================================================
function togglePanel(id) {
  const el = document.getElementById(id);
  if (!el) return;
  const body = el.querySelector('.panel-body');
  if (body) {
    body.classList.toggle('hidden');
  }
}

function toggleSurveyDrawer() {
  const drawer = document.getElementById('survey-drawer');
  if (drawer) {
    drawer.classList.toggle('hidden');
  }
}

async function postApi(url, data, successToast) {
  try {
    const resp = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data)
    });
    const res = await resp.json();
    if (res.success && successToast) {
      showToast(successToast);
    }
    return res;
  } catch (err) {
    showToast('Command failed: ' + err);
  }
}

let toastTimer = null;
function showToast(message) {
  const toast = document.getElementById('toast');
  if (!toast) return;

  toast.textContent = message;
  toast.classList.remove('hidden');

  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    toast.classList.add('hidden');
  }, 3500);
}

