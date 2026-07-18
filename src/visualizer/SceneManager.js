import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export class SceneManager {
  constructor(containerId) {
    this.container = document.getElementById(containerId);
    if (!this.container) {
      throw new Error(`Container #${containerId} not found.`);
    }

    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;
    this.gridHelper = null;

    this.init();
  }

  init() {
    // 1. Create Scene
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x07080d);
    // Add space-like fog
    this.scene.fog = new THREE.FogExp2(0x07080d, 0.015);

    // 2. Create Camera
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    this.camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
    this.camera.position.set(15, 15, 20);

    // 3. Create WebGL Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    // 4. Orbit Controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.05;
    this.controls.maxPolarAngle = Math.PI / 2 - 0.01; // Don't go below ground
    this.controls.minDistance = 2;
    this.controls.maxDistance = 100;
    this.controls.target.set(0, 5, 0); // Focus on height of 5 units

    // 5. Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.15);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(20, 40, 20);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.5;
    dirLight.shadow.camera.far = 100;
    const d = 25;
    dirLight.shadow.camera.left = -d;
    dirLight.shadow.camera.right = d;
    dirLight.shadow.camera.top = d;
    dirLight.shadow.camera.bottom = -d;
    dirLight.shadow.bias = -0.0005;
    this.scene.add(dirLight);

    // Add a secondary subtle cyan/blue fill light from below/side
    const fillLight = new THREE.DirectionalLight(0x00f2fe, 0.3);
    fillLight.position.set(-20, 5, -20);
    this.scene.add(fillLight);

    // 6. Ground & Coordinate System
    this.createGround();

    // 7. Window Resize Listener
    window.addEventListener('resize', this.onWindowResize.bind(this));
  }

  createGround() {
    // Holographic grid
    const gridSize = 60;
    const gridDivisions = 60;
    this.gridHelper = new THREE.GridHelper(gridSize, gridDivisions, 0x00f2fe, 0x1f293d);
    // Position grid at y = 0
    this.gridHelper.position.y = 0;
    this.gridHelper.material.opacity = 0.35;
    this.gridHelper.material.transparent = true;
    this.scene.add(this.gridHelper);

    // Subtle dark ground plane to receive shadows
    const groundGeo = new THREE.PlaneGeometry(100, 100);
    const groundMat = new THREE.MeshStandardMaterial({
      color: 0x08090f,
      roughness: 0.8,
      metalness: 0.2,
      transparent: true,
      opacity: 0.95
    });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.01;
    ground.receiveShadow = true;
    this.scene.add(ground);

    // Compass ring (decorative circle at center)
    const ringGeo = new THREE.RingGeometry(8, 8.1, 64);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x00f2fe,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.15
    });
    const compassRing = new THREE.Mesh(ringGeo, ringMat);
    compassRing.rotation.x = -Math.PI / 2;
    compassRing.position.y = 0.01;
    this.scene.add(compassRing);
  }

  onWindowResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  setCameraPosition(x, y, z, tx = 0, ty = 5, tz = 0) {
    this.camera.position.set(x, y, z);
    this.controls.target.set(tx, ty, tz);
    this.controls.update();
  }

  update() {
    this.controls.update();
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }
}
