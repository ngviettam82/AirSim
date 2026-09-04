/**
 * Map Manager: Leaflet Map Integration with Google Satellite Basemap,
 * Vehicle Orientation Tracking, Breadcrumb Trail, and Interactive
 * Boustrophedon Survey Polygon Drawing.
 */

class MapManager {
  constructor(mapContainerId) {
    this.containerId = mapContainerId;
    this.map = null;
    this.droneMarker = null;
    this.homeMarker = null;
    this.targetMarker = null;
    this.flightTrail = null;
    this.geofencePolygon = null;

    this.trailCoords = [];
    this.drawnPolygon = null;
    this.surveyPathLayer = null;
    this.photoTriggerMarkers = [];

    this.isDrawing = false;
    this.polygonPoints = [];
    this.drawMarkers = [];
    this.tempPolygon = null;

    this.initMap();
  }

  initMap() {
    // Default AirSim Blocks coordinates (Redmond / Seattle area)
    const initialLat = 47.641468;
    const initialLon = -122.140165;

    this.map = L.map(this.containerId, {
      center: [initialLat, initialLon],
      zoom: 18,
      maxZoom: 22,
      zoomControl: false
    });

    L.control.zoom({ position: 'bottomright' }).addTo(this.map);

    // 1. Google Maps & Basemap Layers
    const googleHybrid = L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', {
      maxZoom: 22,
      attribution: 'Google Hybrid'
    });

    const googleSatellite = L.tileLayer('https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
      maxZoom: 22,
      attribution: 'Google Satellite'
    });

    const googleStreets = L.tileLayer('https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}', {
      maxZoom: 22,
      attribution: 'Google Streets'
    });

    const esriWorld = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxZoom: 20,
      attribution: 'Esri World Imagery'
    });

    const osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: 'OpenStreetMap'
    });

    // Default to Google Hybrid (Satellite + Roads/Labels)
    googleHybrid.addTo(this.map);

    const baseMaps = {
      "Google Hybrid": googleHybrid,
      "Google Satellite": googleSatellite,
      "Google Streets": googleStreets,
      "ESRI Satellite": esriWorld,
      "OpenStreetMap": osm
    };

    L.control.layers(baseMaps, null, { position: 'topright' }).addTo(this.map);

    // 2. Custom Drone SVG Icon (Rotatable)
    const droneSvg = `
      <div id="drone-icon-wrapper" style="transform: rotate(0deg); transform-origin: 50% 50%; transition: transform 0.1s linear;">
        <svg width="36" height="36" viewBox="0 0 48 48" fill="none" xmlns="http://www.w3.org/2000/svg">
          <circle cx="24" cy="24" r="22" fill="rgba(0, 229, 255, 0.2)" stroke="#00e5ff" stroke-width="2"/>
          <path d="M24 8 L32 36 L24 30 L16 36 Z" fill="#00e5ff" stroke="#ffffff" stroke-width="1.5"/>
          <circle cx="24" cy="24" r="4" fill="#ff1744"/>
        </svg>
      </div>
    `;

    const droneIcon = L.divIcon({
      html: droneSvg,
      className: 'drone-map-icon',
      iconSize: [36, 36],
      iconAnchor: [18, 18]
    });

    this.droneMarker = L.marker([initialLat, initialLon], { icon: droneIcon }).addTo(this.map);

    // 3. Flight Trail Polyline
    this.flightTrail = L.polyline([], {
      color: '#00e676',
      weight: 2.5,
      opacity: 0.8,
      dashArray: '4, 4'
    }).addTo(this.map);

    // 4. Click map handler (Guided Mode / Polygon Drawing)
    this.map.on('click', (e) => this.onMapClick(e));
  }

  updateVehicle(lat, lon, headingDeg) {
    if (!this.map || !this.droneMarker) return;

    const latLng = [lat, lon];
    this.droneMarker.setLatLng(latLng);

    // Rotate custom SVG element
    const iconEl = document.getElementById('drone-icon-wrapper');
    if (iconEl) {
      iconEl.style.transform = `rotate(${headingDeg}deg)`;
    }

    // Add to flight trail (sample every ~10 updates or >1m delta)
    if (this.trailCoords.length === 0) {
      this.trailCoords.push(latLng);
    } else {
      const last = this.trailCoords[this.trailCoords.length - 1];
      const dist = Math.hypot(lat - last[0], lon - last[1]);
      if (dist > 0.00002) { // roughly >2 meters
        this.trailCoords.push(latLng);
        if (this.trailCoords.length > 500) this.trailCoords.shift();
        this.flightTrail.setLatLngs(this.trailCoords);
      }
    }
  }

  startDrawingPolygon() {
    this.isDrawing = true;
    this.clearSurveyPlan();
    this.polygonPoints = [];
    showToast("Click on map to place polygon vertices. Click first point to finish.");
  }

  onMapClick(e) {
    if (!this.isDrawing) {
      // If Shift key is held, send Click-to-Go Guided Waypoint
      if (e.originalEvent.shiftKey) {
        this.sendGuidedTarget(e.latlng.lat, e.latlng.lng);
      }
      return;
    }

    const pt = [e.latlng.lat, e.latlng.lng];

    // Check if clicked near first point to close polygon
    if (this.polygonPoints.length >= 3) {
      const first = this.polygonPoints[0];
      const dist = Math.hypot(pt[0] - first[0], pt[1] - first[1]);
      if (dist < 0.00015) { // close enough to first point
        this.finishDrawingPolygon();
        return;
      }
    }

    this.polygonPoints.push(pt);

    // Draw vertex marker
    const marker = L.circleMarker(pt, {
      radius: 6,
      fillColor: '#00e5ff',
      color: '#ffffff',
      weight: 2,
      fillOpacity: 0.9
    }).addTo(this.map);
    this.drawMarkers.push(marker);

    // Update temporary polygon
    if (this.tempPolygon) {
      this.map.removeLayer(this.tempPolygon);
    }
    this.tempPolygon = L.polygon(this.polygonPoints, {
      color: '#00e5ff',
      weight: 2,
      fillColor: 'rgba(0, 229, 255, 0.25)',
      fillOpacity: 0.25
    }).addTo(this.map);
  }

  finishDrawingPolygon() {
    this.isDrawing = false;
    showToast("Polygon completed. Calculating optimal Boustrophedon sweep...");
    
    // Automatically trigger calculation
    document.getElementById('btn-calc-survey').click();
  }

  clearSurveyPlan() {
    this.polygonPoints = [];
    if (this.tempPolygon) {
      this.map.removeLayer(this.tempPolygon);
      this.tempPolygon = null;
    }
    this.drawMarkers.forEach(m => this.map.removeLayer(m));
    this.drawMarkers = [];

    if (this.surveyPathLayer) {
      this.map.removeLayer(this.surveyPathLayer);
      this.surveyPathLayer = null;
    }
    this.photoTriggerMarkers.forEach(m => this.map.removeLayer(m));
    this.photoTriggerMarkers = [];

    const summaryCard = document.getElementById('survey-summary-card');
    if (summaryCard) summaryCard.classList.add('hidden');
  }

  renderSurveyPlan(plan) {
    if (!plan || !plan.waypoints || plan.waypoints.length === 0) return;

    if (this.surveyPathLayer) {
      this.map.removeLayer(this.surveyPathLayer);
    }
    this.photoTriggerMarkers.forEach(m => this.map.removeLayer(m));
    this.photoTriggerMarkers = [];

    // 1. Draw Sweep Tracks (Cyan polyline)
    const latlngs = plan.waypoints.map(wp => [wp.lat, wp.lon]);
    this.surveyPathLayer = L.polyline(latlngs, {
      color: '#00e5ff',
      weight: 3,
      opacity: 0.9
    }).addTo(this.map);

    // 2. Draw Camera Photo Triggers (Yellow dots)
    if (plan.photo_triggers) {
      plan.photo_triggers.forEach(pt => {
        const trigMarker = L.circleMarker([pt.lat, pt.lon], {
          radius: 3,
          fillColor: '#ffeb3b',
          color: '#ff9800',
          weight: 1,
          fillOpacity: 0.9
        }).addTo(this.map);
        this.photoTriggerMarkers.push(trigMarker);
      });
    }

    // Fit map bounds to show complete survey
    this.map.fitBounds(this.surveyPathLayer.getBounds(), { padding: [40, 40] });
  }

  sendGuidedTarget(lat, lon) {
    showToast(`Guided Waypoint commanded: ${lat.toFixed(5)}, ${lon.toFixed(5)}`);
    if (this.targetMarker) {
      this.targetMarker.setLatLng([lat, lon]);
    } else {
      this.targetMarker = L.marker([lat, lon], {
        icon: L.divIcon({
          html: '🎯',
          className: 'target-pin',
          iconSize: [24, 24],
          iconAnchor: [12, 12]
        })
      }).addTo(this.map);
    }

    // Send to backend
    fetch('/api/autonomy/target', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x: 10.0, y: 0.0, z: 0.0, frame_id: 'map' })
    });
  }
}

