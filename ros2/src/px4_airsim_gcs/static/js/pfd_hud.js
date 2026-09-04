/**
 * Aviation Primary Flight Display (PFD) HUD Canvas Renderer.
 * 
 * Draws artificial horizon (sky/ground), pitch ladder, roll dial,
 * airspeed tape, altitude tape, and heading indicator matching
 * military and commercial glass cockpit standards.
 */

class PrimaryFlightDisplay {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    if (!this.canvas) return;
    this.ctx = this.canvas.getContext('2d');
    
    this.pitch = 0.0;    // degrees (+ pitch up)
    this.roll = 0.0;     // degrees (+ roll right)
    this.yaw = 0.0;      // degrees (0 - 360)
    this.speed = 0.0;    // m/s
    this.altitude = 0.0; // meters AGL
    this.climb = 0.0;    // m/s
  }

  update(telemetry) {
    if (!this.ctx) return;
    this.pitch = telemetry.pitch_deg || 0.0;
    this.roll = telemetry.roll_deg || 0.0;
    this.yaw = telemetry.heading_deg || telemetry.yaw_deg || 0.0;
    this.speed = telemetry.groundspeed || 0.0;
    this.altitude = telemetry.alt_agl || 0.0;
    this.climb = telemetry.vertical_speed || 0.0;
    this.render();
  }

  render() {
    const w = this.canvas.width;
    const h = this.canvas.height;
    const ctx = this.ctx;
    const cx = w / 2;
    const cy = h / 2;

    ctx.clearRect(0, 0, w, h);

    // 1. Artificial Horizon (Sky / Ground clipped circle)
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, w, h);
    ctx.clip();

    // Rotate and translate for roll and pitch
    ctx.translate(cx, cy);
    ctx.rotate((-this.roll * Math.PI) / 180);

    const pitchOffset = this.pitch * 3.0; // 3 pixels per degree of pitch

    // Sky (Blue)
    ctx.fillStyle = '#005599';
    ctx.fillRect(-w, -h * 2 + pitchOffset, w * 2, h * 2);

    // Ground (Brown / Dark Olive)
    ctx.fillStyle = '#5c4033';
    ctx.fillRect(-w, pitchOffset, w * 2, h * 2);

    // Horizon line (White)
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-w, pitchOffset);
    ctx.lineTo(w, pitchOffset);
    ctx.stroke();

    // Pitch Ladder Lines (±10°, ±20°, ±30°)
    ctx.font = '10px monospace';
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';

    for (let deg = -40; deg <= 40; deg += 10) {
      if (deg === 0) continue;
      const y = pitchOffset - deg * 3.0;
      const lineWidth = deg % 20 === 0 ? 40 : 25;

      ctx.beginPath();
      ctx.moveTo(-lineWidth, y);
      ctx.lineTo(lineWidth, y);
      ctx.stroke();

      ctx.fillText(Math.abs(deg).toString(), -lineWidth - 12, y + 3);
      ctx.fillText(Math.abs(deg).toString(), lineWidth + 12, y + 3);
    }

    ctx.restore();

    // 2. Fixed Aircraft Boresight / Symbol (Yellow crosshair in center)
    ctx.strokeStyle = '#ffeb3b';
    ctx.lineWidth = 3;
    ctx.beginPath();
    // Center dot
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fillStyle = '#ffeb3b';
    ctx.fill();

    // Left wing
    ctx.moveTo(cx - 35, cy);
    ctx.lineTo(cx - 12, cy);
    ctx.lineTo(cx - 12, cy + 6);

    // Right wing
    ctx.moveTo(cx + 35, cy);
    ctx.lineTo(cx + 12, cy);
    ctx.lineTo(cx + 12, cy + 6);
    ctx.stroke();

    // 3. Roll Indicator Arc at top
    ctx.save();
    ctx.translate(cx, cy);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(0, 0, 90, (-140 * Math.PI) / 180, (-40 * Math.PI) / 180);
    ctx.stroke();

    // Roll ticks at 0, ±15, ±30, ±45, ±60
    [-60, -45, -30, -15, 0, 15, 30, 45, 60].forEach(deg => {
      const rad = ((deg - 90) * Math.PI) / 180;
      const r1 = 90;
      const r2 = deg % 30 === 0 ? 100 : 96;
      ctx.beginPath();
      ctx.moveTo(r1 * Math.cos(rad), r1 * Math.sin(rad));
      ctx.lineTo(r2 * Math.cos(rad), r2 * Math.sin(rad));
      ctx.stroke();
    });

    // Roll pointer (Yellow triangle rotated with current roll)
    ctx.rotate((-this.roll * Math.PI) / 180);
    ctx.fillStyle = '#ffeb3b';
    ctx.beginPath();
    ctx.moveTo(0, -90);
    ctx.lineTo(-5, -82);
    ctx.lineTo(5, -82);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // 4. Speed Tape on Left (m/s)
    ctx.fillStyle = 'rgba(10, 15, 25, 0.85)';
    ctx.fillRect(4, 25, 45, h - 50);
    ctx.strokeStyle = '#2a3649';
    ctx.lineWidth = 1;
    ctx.strokeRect(4, 25, 45, h - 50);

    ctx.font = 'bold 12px monospace';
    ctx.fillStyle = '#00e5ff';
    ctx.textAlign = 'center';
    ctx.fillText(this.speed.toFixed(1), 26, cy);
    ctx.font = '9px sans-serif';
    ctx.fillStyle = '#94a3b8';
    ctx.fillText('m/s', 26, cy + 12);

    // 5. Altitude Tape on Right (m AGL)
    ctx.fillStyle = 'rgba(10, 15, 25, 0.85)';
    ctx.fillRect(w - 49, 25, 45, h - 50);
    ctx.strokeStyle = '#2a3649';
    ctx.strokeRect(w - 49, 25, 45, h - 50);

    ctx.font = 'bold 12px monospace';
    ctx.fillStyle = '#00e676';
    ctx.textAlign = 'center';
    ctx.fillText(this.altitude.toFixed(1), w - 27, cy);
    ctx.font = '9px sans-serif';
    ctx.fillStyle = '#94a3b8';
    ctx.fillText('ALT', w - 27, cy + 12);

    // 6. Heading Ribbon at top
    ctx.fillStyle = 'rgba(10, 15, 25, 0.9)';
    ctx.fillRect(cx - 35, 4, 70, 18);
    ctx.strokeStyle = '#2a3649';
    ctx.strokeRect(cx - 35, 4, 70, 18);

    ctx.font = 'bold 11px monospace';
    ctx.fillStyle = '#ffab00';
    ctx.textAlign = 'center';
    ctx.fillText(Math.round((this.yaw + 360) % 360).toString().padStart(3, '0') + '°', cx, 17);
  }
}

