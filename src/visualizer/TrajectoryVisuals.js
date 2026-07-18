import * as THREE from 'three';

export class TrajectoryVisuals {
  constructor(scene) {
    this.scene = scene;
    this.lines = [];
    this.waypointPointsMesh = null;
    
    // Create a glowing circular texture for waypoint particles
    this.waypointTexture = this.createCircleTexture();
  }

  createCircleTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 16;
    canvas.height = 16;
    const ctx = canvas.getContext('2d');
    const grad = ctx.createRadialGradient(8, 8, 0, 8, 8, 8);
    grad.addColorStop(0, 'rgba(0, 242, 254, 1)');
    grad.addColorStop(0.3, 'rgba(0, 242, 254, 0.8)');
    grad.addColorStop(1, 'rgba(0, 0, 0, 0)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 16, 16);
    return new THREE.CanvasTexture(canvas);
  }

  update(pathsData, dronesData) {
    // 1. Clear old line trajectories
    this.clearLines();

    // 2. Render continuous self-planned paths
    if (pathsData && pathsData.length > 0) {
      pathsData.forEach(pathCoords => {
        const points = pathCoords.map(coord => new THREE.Vector3(coord[0], coord[2], -coord[1]));
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({
          color: 0x00f2fe,
          transparent: true,
          opacity: 0.18,
          linewidth: 1
        });
        const line = new THREE.Line(geometry, material);
        this.scene.add(line);
        this.lines.push({ line, geometry });
      });
    }

    // 3. Render sparse mission waypoints as particles
    this.clearWaypoints();
    
    if (dronesData && dronesData.length > 0) {
      const waypointCoords = [];
      
      // For massive swarms (100+), limit waypoint dots to prevent visual clutter
      // Let's only render waypoints for first 15 drones if there are 100 drones, or all if 10 or less
      const limit = dronesData.length > 10 ? 15 : dronesData.length;
      
      for (let i = 0; i < limit; i++) {
        const drone = dronesData[i];
        if (drone.waypoints && drone.waypoints.length > 0) {
          drone.waypoints.forEach(wp => {
            waypointCoords.push(new THREE.Vector3(wp[0], wp[2], -wp[1]));
          });
        }
      }

      if (waypointCoords.length > 0) {
        const geometry = new THREE.BufferGeometry().setFromPoints(waypointCoords);
        const material = new THREE.PointsMaterial({
          color: 0x00f2fe,
          size: 0.45,
          map: this.waypointTexture,
          transparent: true,
          opacity: 0.7,
          depthWrite: false
        });
        
        this.waypointPointsMesh = new THREE.Points(geometry, material);
        this.scene.add(this.waypointPointsMesh);
      }
    }
  }

  clearLines() {
    this.lines.forEach(item => {
      this.scene.remove(item.line);
      item.geometry.dispose();
    });
    this.lines = [];
  }

  clearWaypoints() {
    if (this.waypointPointsMesh) {
      this.scene.remove(this.waypointPointsMesh);
      this.waypointPointsMesh.geometry.dispose();
      this.waypointPointsMesh = null;
    }
  }

  clear() {
    this.clearLines();
    this.clearWaypoints();
  }
}
