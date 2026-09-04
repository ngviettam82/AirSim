/**
 * 3D Obstacle Clearance Radar Widget Controller.
 * Updates the 3x3 spatial clearance sector grid and dynamic stopping bubble alert.
 */

class ObstacleRadar {
  constructor() {
    this.cellIds = [
      ['sec-u-l', 'sec-u-c', 'sec-u-r'],
      ['sec-m-l', 'sec-m-c', 'sec-m-r'],
      ['sec-d-l', 'sec-d-c', 'sec-d-r']
    ];

    this.minDistEl = document.getElementById('radar-min-dist');
    this.bubbleIndicatorEl = document.getElementById('bubble-breach-indicator');
  }

  update(telemetry) {
    const sectors = telemetry.obstacle_sectors || [
      [25.0, 25.0, 25.0],
      [25.0, 25.0, 25.0],
      [25.0, 25.0, 25.0]
    ];

    let overallMin = 100.0;

    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        const cellEl = document.getElementById(this.cellIds[r][c]);
        if (!cellEl) continue;

        const val = sectors[r][c] || 100.0;
        if (val < overallMin) overallMin = val;

        const valEl = cellEl.querySelector('.sec-val');
        if (valEl) {
          valEl.textContent = val >= 50.0 ? '>50m' : val.toFixed(1) + 'm';
        }

        // Apply Proximity Colors
        cellEl.classList.remove('sec-danger', 'sec-warning', 'sec-safe');
        if (val < 2.0) {
          cellEl.classList.add('sec-danger');
          cellEl.style.backgroundColor = 'rgba(255, 23, 68, 0.4)';
          cellEl.style.borderColor = '#ff1744';
          if (valEl) valEl.style.color = '#ff1744';
        } else if (val < 5.0) {
          cellEl.classList.add('sec-warning');
          cellEl.style.backgroundColor = 'rgba(255, 171, 0, 0.25)';
          cellEl.style.borderColor = '#ffab00';
          if (valEl) valEl.style.color = '#ffab00';
        } else {
          cellEl.classList.add('sec-safe');
          cellEl.style.backgroundColor = '#1a2332';
          cellEl.style.borderColor = (r === 1 && c === 1) ? '#00e5ff' : '#2d3b52';
          if (valEl) valEl.style.color = '#00e676';
        }
      }
    }

    const minClearance = telemetry.min_depth_m || overallMin;
    if (this.minDistEl) {
      this.minDistEl.textContent = minClearance.toFixed(1) + ' m';
      this.minDistEl.style.color = minClearance < 2.0 ? '#ff1744' : (minClearance < 5.0 ? '#ffab00' : '#00e676');
    }

    if (this.bubbleIndicatorEl) {
      if (minClearance < 2.5) {
        this.bubbleIndicatorEl.textContent = 'BUBBLE: BREACH';
        this.bubbleIndicatorEl.className = 'bubble-alert';
      } else {
        this.bubbleIndicatorEl.textContent = 'BUBBLE: CLEAR';
        this.bubbleIndicatorEl.className = 'bubble-safe';
      }
    }
  }
}

