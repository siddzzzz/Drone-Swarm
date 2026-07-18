import * as THREE from 'three';

export class DroneVisuals {
  constructor(scene) {
    this.scene = scene;
    this.dronesMap = new Map(); // id -> { group, ledMesh, trailGeometry, trailLine, trailPoints }
    
    // Shared geometries/materials for performance
    this.droneBodyGeometry = new THREE.CylinderGeometry(0.12, 0.12, 0.05, 8);
    this.armGeometry = new THREE.BoxGeometry(0.8, 0.02, 0.02);
    this.propGeometry = new THREE.CylinderGeometry(0.18, 0.18, 0.005, 8);
    
    this.bodyMaterial = new THREE.MeshStandardMaterial({ color: 0x222530, roughness: 0.5, metalness: 0.8 });
    this.propMaterial = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.25,
      roughness: 0.9
    });
  }

  createDroneMesh(colorHex) {
    const droneGroup = new THREE.Group();
    droneGroup.castShadow = true;
    
    // 1. Central Hub
    const hub = new THREE.Mesh(this.droneBodyGeometry, this.bodyMaterial);
    hub.rotation.x = Math.PI / 2;
    hub.castShadow = true;
    droneGroup.add(hub);
    
    // 2. Arms (X shape)
    const arm1 = new THREE.Mesh(this.armGeometry, this.bodyMaterial);
    arm1.rotation.y = Math.PI / 4;
    arm1.castShadow = true;
    droneGroup.add(arm1);
    
    const arm2 = new THREE.Mesh(this.armGeometry, this.bodyMaterial);
    arm2.rotation.y = -Math.PI / 4;
    arm2.castShadow = true;
    droneGroup.add(arm2);
    
    // 3. Propellers (4 rotators)
    const offsets = [
      [0.28, 0.04, 0.28],
      [0.28, 0.04, -0.28],
      [-0.28, 0.04, 0.28],
      [-0.28, 0.04, -0.28]
    ];
    
    const props = [];
    offsets.forEach(([dx, dy, dz]) => {
      const prop = new THREE.Mesh(this.propGeometry, this.propMaterial);
      prop.position.set(dx, dy, dz);
      droneGroup.add(prop);
      props.push(prop);
    });

    // 4. Glowing LED Sphere (at center bottom)
    const ledGeo = new THREE.SphereGeometry(0.14, 16, 16);
    const ledMat = new THREE.MeshBasicMaterial({
      color: new THREE.Color(colorHex),
      transparent: false
    });
    const led = new THREE.Mesh(ledGeo, ledMat);
    led.position.set(0, -0.06, 0);
    droneGroup.add(led);

    this.scene.add(droneGroup);

    return {
      group: droneGroup,
      ledMesh: led,
      props: props
    };
  }

  createTrailMesh(colorHex, numDrones) {
    // Adapt trail length based on swarm scale to prevent GPU memory/rendering bottlenecks
    const maxTrailPoints = numDrones > 60 ? 15 : (numDrones > 20 ? 35 : 70);
    const trailPoints = [];
    // Populate trail coordinates at origin initially
    for (let i = 0; i < maxTrailPoints; i++) {
      trailPoints.push(new THREE.Vector3(0, 0, 0));
    }
    
    const geometry = new THREE.BufferGeometry().setFromPoints(trailPoints);
    
    // Custom transparent glowing material for trails
    const material = new THREE.LineBasicMaterial({
      color: new THREE.Color(colorHex),
      transparent: true,
      opacity: 0.45,
      linewidth: 1
    });
    
    const line = new THREE.Line(geometry, material);
    this.scene.add(line);
    
    return {
      geometry: geometry,
      line: line,
      points: trailPoints
    };
  }

  update(telemetryDrones) {
    const activeIds = new Set();
    
    const numDrones = telemetryDrones.length;
    
    telemetryDrones.forEach(droneData => {
      const id = droneData.id;
      activeIds.add(id);
      
      const pos = new THREE.Vector3(droneData.x, droneData.z, -droneData.y); // Coordinate mapping: Python Z is height -> Three.js Y, Python Y is depth -> Three.js -Z
      const colorHex = droneData.color;
      
      // If drone visual doesn't exist, create it
      if (!this.dronesMap.has(id)) {
        const meshes = this.createDroneMesh(colorHex);
        const trail = this.createTrailMesh(colorHex, numDrones);
        
        // Initialize position instantly
        meshes.group.position.copy(pos);
        for (let i = 0; i < trail.points.length; i++) {
          trail.points[i].copy(pos);
        }
        trail.geometry.setFromPoints(trail.points);
        
        this.dronesMap.set(id, {
          ...meshes,
          ...trail,
          color: colorHex
        });
      }
      
      const droneVisual = this.dronesMap.get(id);
      
      // Update position
      droneVisual.group.position.copy(pos);
      
      // Update orientation (tilt: roll/pitch)
      // Note: Three.js pitch/roll/yaw rotations
      droneVisual.group.rotation.set(droneData.pitch, 0, droneData.roll);
      
      // Spin propellers visually
      droneVisual.props.forEach((prop, i) => {
        prop.rotation.y += (i % 2 === 0 ? 0.4 : -0.4);
      });
      
      // Update LED Color if it changed
      if (droneVisual.color !== colorHex) {
        const newColor = new THREE.Color(colorHex);
        droneVisual.ledMesh.material.color.copy(newColor);
        droneVisual.line.material.color.copy(newColor);
        
        // Hide trail completely and reset history when entering standby
        if (colorHex === "#151720") {
          droneVisual.line.material.opacity = 0.0;
          for (let i = 0; i < droneVisual.points.length; i++) {
            droneVisual.points[i].copy(pos);
          }
        } else {
          droneVisual.line.material.opacity = 0.45;
        }
        
        droneVisual.color = colorHex;
      }
      
      // Update Trail
      // Slide historical coordinates back
      const pts = droneVisual.points;
      pts.shift();
      pts.push(pos.clone());
      
      // Update buffer geometry coordinates
      droneVisual.geometry.setFromPoints(pts);
      droneVisual.geometry.attributes.position.needsUpdate = true;
    });
    
    // Clean up inactive drones (if drone count scaled down)
    for (const [id, droneVisual] of this.dronesMap.entries()) {
      if (!activeIds.has(id)) {
        this.scene.remove(droneVisual.group);
        this.scene.remove(droneVisual.line);
        droneVisual.geometry.dispose();
        // ledMesh and props materials don't need manual disposal since they are shared,
        // but pointLight and individual objects should be cleaned up.
        this.dronesMap.delete(id);
      }
    }
  }

  clearAll() {
    for (const [id, droneVisual] of this.dronesMap.entries()) {
      this.scene.remove(droneVisual.group);
      this.scene.remove(droneVisual.line);
      droneVisual.geometry.dispose();
    }
    this.dronesMap.clear();
  }
}
