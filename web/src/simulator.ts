import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

import type { DemoDefinition } from "./demos";
import { getMujoco, prepareAssets, WORKING_ROOT } from "./mujocoAssets";
import type { MainModule, MjData, MjModel } from "@mujoco/mujoco";

type Side = "left" | "right";
type NumericView = {
  readonly length: number;
  [index: number]: number;
  subarray?: (start: number, end?: number) => NumericView;
};

interface ArmControlSpec {
  key: string;
  label: string;
  neg: string;
  pos: string;
  speed: number;
}

interface ActuatorTarget {
  aid: number;
  side: Side;
  spec: ArmControlSpec;
  name: string;
  min: number;
  max: number;
  value: number;
}

const ARM_CONTROL_SPECS: ArmControlSpec[] = [
  { key: "joint1", label: "J1", neg: "q", pos: "a", speed: 1.0 },
  { key: "joint2", label: "J2", neg: "w", pos: "s", speed: 1.0 },
  { key: "joint3", label: "J3", neg: "e", pos: "d", speed: 1.0 },
  { key: "joint4", label: "J4", neg: "r", pos: "f", speed: 1.0 },
  { key: "joint5", label: "J5", neg: "t", pos: "g", speed: 1.15 },
  { key: "joint6", label: "J6", neg: "y", pos: "h", speed: 1.15 },
  { key: "joint7", label: "J7", neg: "u", pos: "j", speed: 1.15 },
  { key: "finger1", label: "Grip", neg: "[", pos: "]", speed: 0.8 },
];

const SIDE_LABEL: Record<Side, string> = {
  left: "Left arm",
  right: "Right arm",
};

const clamp = (value: number, min: number, max: number): number =>
  Math.min(Math.max(value, min), max);

const radToDeg = (value: number): number => (value * 180) / Math.PI;

export class ViewerApp {
  private readonly root: HTMLElement;
  private readonly demo: DemoDefinition;
  private readonly onExit: () => void;
  private readonly stage: HTMLDivElement;
  private readonly panel: HTMLElement;
  private readonly hud: HTMLDivElement;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene: THREE.Scene;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly resizeObserver: ResizeObserver;
  private readonly pressedKeys = new Set<string>();
  private readonly bodyGroups = new Map<number, THREE.Group>();
  private readonly targetByAid = new Map<number, ActuatorTarget>();
  private readonly targetsBySide: Record<Side, ActuatorTarget[]> = {
    left: [],
    right: [],
  };
  private readonly neutralTargets = new Map<number, number>();

  private mujoco!: MainModule;
  private model!: MjModel;
  private data!: MjData;
  private mujocoRoot?: THREE.Group;
  private showroomRoot?: THREE.Group;
  private activeSide: Side = "left";
  private paused = false;
  private lastTimeMs = 0;
  private accumulator = 0;
  private animationActive = false;

  static async create(
    root: HTMLElement,
    demo: DemoDefinition,
    onExit: () => void,
  ): Promise<ViewerApp> {
    const app = new ViewerApp(root, demo, onExit);
    await app.init();
    return app;
  }

  private constructor(root: HTMLElement, demo: DemoDefinition, onExit: () => void) {
    this.root = root;
    this.demo = demo;
    this.onExit = onExit;

    this.root.innerHTML = "";
    const viewer = document.createElement("div");
    viewer.className = "viewer";

    this.stage = document.createElement("div");
    this.stage.className = "viewer__stage";

    this.hud = document.createElement("div");
    this.hud.className = "hud";
    this.stage.append(this.hud);

    this.panel = document.createElement("aside");
    this.panel.className = "panel";

    viewer.append(this.stage, this.panel);
    this.root.append(viewer);

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x172522);
    this.scene.fog = new THREE.Fog(0x172522, 9, 24);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.001, 100);
    this.scene.add(this.camera);

    const ambient = new THREE.AmbientLight(0xe7fff7, 0.34);
    this.scene.add(ambient);
    const hemisphere = new THREE.HemisphereLight(0xd8fff4, 0x2f3f3a, 0.58);
    this.scene.add(hemisphere);
    const key = new THREE.DirectionalLight(0xffffff, 1.9);
    key.position.set(3.4, 5.2, 3.1);
    key.castShadow = true;
    key.shadow.mapSize.width = 2048;
    key.shadow.mapSize.height = 2048;
    key.shadow.camera.near = 0.1;
    key.shadow.camera.far = 18;
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xa9d7d0, 0.68);
    fill.position.set(-3.6, 2.4, -2.8);
    this.scene.add(fill);
    const rim = new THREE.DirectionalLight(0x9ce0c9, 0.82);
    rim.position.set(-1.8, 3.3, 4.2);
    this.scene.add(rim);
    this.buildShowroomScene();

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: false,
      preserveDrawingBuffer: true,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.stage.append(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.panSpeed = 1.3;
    this.controls.zoomSpeed = 0.85;

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(this.stage);
    this.resize();
    this.resetCamera();
  }

  private async init(): Promise<void> {
    this.mujoco = await getMujoco();
    await prepareAssets(this.mujoco);

    this.model = this.mujoco.MjModel.from_xml_path(
      `${WORKING_ROOT}/${this.demo.xmlPath}`,
    );
    this.data = new this.mujoco.MjData(this.model);
    this.resetMujocoState();
    this.discoverActuators();
    this.syncTargetsFromData();
    for (const target of this.targetByAid.values()) {
      this.neutralTargets.set(target.aid, target.value);
    }
    this.buildModelScene();
    this.setupPanel();
    this.updateHud();
    this.updateBodyTransforms();

    window.addEventListener("keydown", this.handleKeyDown);
    window.addEventListener("keyup", this.handleKeyUp);
    this.animationActive = true;
    this.renderer.setAnimationLoop(this.render);
  }

  dispose(): void {
    this.animationActive = false;
    this.renderer.setAnimationLoop(null);
    window.removeEventListener("keydown", this.handleKeyDown);
    window.removeEventListener("keyup", this.handleKeyUp);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.disposeSceneObjects();
    this.disposeShowroomScene();
    this.renderer.dispose();
    this.data?.delete();
    this.model?.delete();
    this.root.innerHTML = "";
  }

  private readonly render = (timeMs: number): void => {
    if (!this.animationActive) {
      return;
    }
    const dt = this.lastTimeMs
      ? clamp((timeMs - this.lastTimeMs) / 1000, 0, 0.05)
      : 0;
    this.lastTimeMs = timeMs;
    this.controls.update();

    if (!this.paused) {
      this.applyKeyboardJog(dt);
      this.applyScriptedController();
      this.applyTargetsToData();

      const timestep = this.model.opt.timestep || 0.002;
      this.accumulator += dt;
      let steps = 0;
      while (this.accumulator >= timestep && steps < 20) {
        this.applyTargetsToData();
        this.mujoco.mj_step(this.model, this.data);
        this.accumulator -= timestep;
        steps += 1;
      }
    } else {
      this.mujoco.mj_forward(this.model, this.data);
    }

    this.updateBodyTransforms();
    this.syncSliderValues();
    this.updateHud();
    this.renderer.render(this.scene, this.camera);
  };

  private readonly handleKeyDown = (event: KeyboardEvent): void => {
    if (this.isTypingTarget(event.target)) {
      return;
    }

    if (event.key === "1") {
      this.setActiveSide("left");
      event.preventDefault();
      return;
    }
    if (event.key === "2") {
      this.setActiveSide("right");
      event.preventDefault();
      return;
    }
    if (event.code === "Space") {
      this.paused = !this.paused;
      event.preventDefault();
      this.updatePanelButtons();
      return;
    }
    if (event.code === "Backspace") {
      this.resetSimulation();
      event.preventDefault();
      return;
    }
    if (event.key.toLowerCase() === "c") {
      this.resetCamera();
      event.preventDefault();
      return;
    }
    if (event.key === "Escape") {
      event.preventDefault();
      this.onExit();
      return;
    }

    const key = event.key.toLowerCase();
    if (this.isJogKey(key)) {
      this.pressedKeys.add(key);
      event.preventDefault();
    }
  };

  private readonly handleKeyUp = (event: KeyboardEvent): void => {
    this.pressedKeys.delete(event.key.toLowerCase());
  };

  private isTypingTarget(target: EventTarget | null): boolean {
    if (!(target instanceof HTMLElement)) {
      return false;
    }
    return target.tagName === "INPUT" && target.getAttribute("type") !== "range";
  }

  private isJogKey(key: string): boolean {
    return ARM_CONTROL_SPECS.some((spec) => spec.neg === key || spec.pos === key);
  }

  private setupPanel(): void {
    this.panel.innerHTML = `
      <div class="panel__top">
        <div>
          <h1>${this.demo.title}</h1>
          <p class="panel__sub">${this.demo.eyebrow}</p>
        </div>
        <button class="icon-button" data-action="exit" aria-label="Back to demos" title="Back to demos">x</button>
      </div>
      <div class="segment" aria-label="Arm selection">
        <button data-side="left" aria-pressed="true">Left</button>
        <button data-side="right" aria-pressed="false">Right</button>
      </div>
      <div class="command-row">
        <button class="command-button" data-action="pause">Pause</button>
        <button class="command-button" data-action="reset">Reset</button>
        <button class="command-button" data-action="camera">Camera</button>
      </div>
      <div class="slider-group" data-slider-group></div>
      <div class="key-grid" aria-label="Keyboard teleop map">
        ${ARM_CONTROL_SPECS.map(
          (spec) => `<div class="keycap">${spec.neg.toUpperCase()} / ${spec.pos.toUpperCase()}</div>`,
        ).join("")}
      </div>
      <div class="status" data-status></div>
    `;

    this.panel
      .querySelector<HTMLButtonElement>('[data-action="exit"]')
      ?.addEventListener("click", this.onExit);
    this.panel
      .querySelector<HTMLButtonElement>('[data-action="pause"]')
      ?.addEventListener("click", () => {
        this.paused = !this.paused;
        this.updatePanelButtons();
      });
    this.panel
      .querySelector<HTMLButtonElement>('[data-action="reset"]')
      ?.addEventListener("click", () => this.resetSimulation());
    this.panel
      .querySelector<HTMLButtonElement>('[data-action="camera"]')
      ?.addEventListener("click", () => this.resetCamera());
    this.panel.querySelectorAll<HTMLButtonElement>("[data-side]").forEach((button) => {
      button.addEventListener("click", () => {
        const side = button.dataset.side === "right" ? "right" : "left";
        this.setActiveSide(side);
      });
    });

    this.renderSliders();
    this.updatePanelButtons();
  }

  private renderSliders(): void {
    const group = this.panel.querySelector<HTMLDivElement>("[data-slider-group]");
    if (!group) {
      return;
    }
    const targets = this.targetsBySide[this.activeSide];
    group.innerHTML = targets
      .map(
        (target) => `
        <div class="slider-row">
          <label for="act-${target.aid}">${target.spec.label}</label>
          <input
            id="act-${target.aid}"
            type="range"
            min="${target.min}"
            max="${target.max}"
            step="0.001"
            value="${target.value}"
            data-aid="${target.aid}"
            aria-label="${SIDE_LABEL[target.side]} ${target.spec.label}"
          />
          <output data-output="${target.aid}">${radToDeg(target.value).toFixed(0)} deg</output>
        </div>`,
      )
      .join("");

    group.querySelectorAll<HTMLInputElement>("input[data-aid]").forEach((input) => {
      input.addEventListener("input", () => {
        const aid = Number(input.dataset.aid);
        const target = this.targetByAid.get(aid);
        if (!target) {
          return;
        }
        target.value = clamp(Number(input.value), target.min, target.max);
        this.applyTargetsToData();
        this.syncSliderValues();
      });
    });
    this.syncSliderValues();
  }

  private setActiveSide(side: Side): void {
    this.activeSide = side;
    this.panel.querySelectorAll<HTMLButtonElement>("[data-side]").forEach((button) => {
      button.setAttribute(
        "aria-pressed",
        button.dataset.side === this.activeSide ? "true" : "false",
      );
    });
    this.renderSliders();
  }

  private updatePanelButtons(): void {
    const pause = this.panel.querySelector<HTMLButtonElement>('[data-action="pause"]');
    if (pause) {
      pause.textContent = this.paused ? "Resume" : "Pause";
    }
  }

  private updateHud(): void {
    this.hud.innerHTML = `
      <div class="hud__pill">${this.demo.title}</div>
      <div class="hud__pill">${SIDE_LABEL[this.activeSide]}</div>
      <div class="hud__pill">${this.paused ? "Paused" : `${this.data?.time.toFixed(2) ?? "0.00"}s`}</div>
    `;
    const status = this.panel.querySelector<HTMLDivElement>("[data-status]");
    if (status) {
      status.textContent = `${SIDE_LABEL[this.activeSide]} selected. ${this.demo.script === "wristSweep" ? "Wrist sweep is driving J6/J7." : "Keyboard and sliders drive joint-space targets."}`;
    }
  }

  private syncSliderValues(): void {
    this.panel.querySelectorAll<HTMLInputElement>("input[data-aid]").forEach((input) => {
      const aid = Number(input.dataset.aid);
      const target = this.targetByAid.get(aid);
      if (!target) {
        return;
      }
      input.value = String(target.value);
      const output = this.panel.querySelector<HTMLOutputElement>(
        `[data-output="${aid}"]`,
      );
      if (output) {
        output.value = `${radToDeg(target.value).toFixed(0)} deg`;
        output.textContent = output.value;
      }
    });
  }

  private applyKeyboardJog(dt: number): void {
    if (!dt) {
      return;
    }
    const targets = this.targetsBySide[this.activeSide];
    for (const target of targets) {
      const neg = this.pressedKeys.has(target.spec.neg);
      const pos = this.pressedKeys.has(target.spec.pos);
      if (neg === pos) {
        continue;
      }
      const direction = pos ? 1 : -1;
      target.value = clamp(
        target.value + direction * target.spec.speed * dt,
        target.min,
        target.max,
      );
    }
  }

  private applyScriptedController(): void {
    if (this.demo.script !== "wristSweep") {
      return;
    }
    const phaseSeconds = 6.0;
    const blendSeconds = 1.0;
    const settleSeconds = 2.0;
    const t = this.data.time - settleSeconds;

    for (const side of ["left", "right"] as const) {
      this.setTarget(side, "joint4", Math.PI / 2);
    }

    if (t < 0) {
      return;
    }

    const phase = Math.floor(t / phaseSeconds) % 3;
    const s = (t % phaseSeconds) / phaseSeconds;
    const tIn = t % phaseSeconds;
    const blend =
      smoothstep(tIn / blendSeconds) *
      smoothstep((phaseSeconds - tIn) / blendSeconds);

    for (const side of ["left", "right"] as const) {
      if (phase === 0 || phase === 2) {
        this.setSweptTarget(side, "joint6", Math.sin(2 * Math.PI * s), blend);
      }
      if (phase === 1 || phase === 2) {
        const signal = phase === 2 ? Math.cos(2 * Math.PI * s) : Math.sin(2 * Math.PI * s);
        this.setSweptTarget(side, "joint7", signal, blend);
      }
    }
  }

  private setSweptTarget(
    side: Side,
    key: string,
    signal: number,
    blend: number,
  ): void {
    const target = this.findTarget(side, key);
    if (!target) {
      return;
    }
    const neutral = this.neutralTargets.get(target.aid) ?? target.value;
    const mid = (target.min + target.max) / 2;
    const amp = (target.max - target.min) / 2;
    target.value = clamp(neutral + blend * (mid + amp * signal - neutral), target.min, target.max);
  }

  private setTarget(side: Side, key: string, value: number): void {
    const target = this.findTarget(side, key);
    if (target) {
      target.value = clamp(value, target.min, target.max);
    }
  }

  private findTarget(side: Side, key: string): ActuatorTarget | undefined {
    return this.targetsBySide[side].find((target) => target.spec.key === key);
  }

  private applyTargetsToData(): void {
    for (const target of this.targetByAid.values()) {
      this.data.ctrl[target.aid] = target.value;
    }
  }

  private resetSimulation(): void {
    this.pressedKeys.clear();
    this.resetMujocoState();
    for (const target of this.targetByAid.values()) {
      target.value = this.neutralTargets.get(target.aid) ?? this.data.ctrl[target.aid];
    }
    this.applyTargetsToData();
    this.accumulator = 0;
    this.renderSliders();
  }

  private resetMujocoState(): void {
    const keyId = this.mujoco.mj_name2id(
      this.model,
      this.mujoco.mjtObj.mjOBJ_KEY.value,
      "home",
    );
    if (keyId >= 0) {
      this.mujoco.mj_resetDataKeyframe(this.model, this.data, keyId);
      for (let aid = 0; aid < this.model.nu; aid += 1) {
        this.data.ctrl[aid] = this.model.key_ctrl[keyId * this.model.nu + aid];
      }
    } else {
      this.mujoco.mj_resetData(this.model, this.data);
    }
    this.mujoco.mj_forward(this.model, this.data);
  }

  private discoverActuators(): void {
    this.targetByAid.clear();
    this.targetsBySide.left = [];
    this.targetsBySide.right = [];

    for (const side of ["left", "right"] as const) {
      for (const spec of ARM_CONTROL_SPECS) {
        const name = `${side}_${spec.key}_ctrl`;
        const aid = this.mujoco.mj_name2id(
          this.model,
          this.mujoco.mjtObj.mjOBJ_ACTUATOR.value,
          name,
        );
        if (aid < 0) {
          continue;
        }
        const target: ActuatorTarget = {
          aid,
          side,
          spec,
          name,
          min: this.model.actuator_ctrlrange[aid * 2],
          max: this.model.actuator_ctrlrange[aid * 2 + 1],
          value: this.data.ctrl[aid],
        };
        this.targetByAid.set(aid, target);
        this.targetsBySide[side].push(target);
      }
    }
  }

  private syncTargetsFromData(): void {
    for (const target of this.targetByAid.values()) {
      target.value = clamp(this.data.ctrl[target.aid], target.min, target.max);
    }
  }

  private resetCamera(): void {
    this.camera.position.set(...this.demo.camera.position);
    this.controls.target.set(...this.demo.camera.target);
    this.controls.update();
  }

  private buildShowroomScene(): void {
    this.showroomRoot = new THREE.Group();
    this.showroomRoot.name = "Showroom Root";
    this.scene.add(this.showroomRoot);

    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(18, 18),
      new THREE.MeshStandardMaterial({
        color: 0xb9c5c1,
        roughness: 0.82,
        metalness: 0.02,
      }),
    );
    floor.name = "Showroom floor";
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -0.002;
    floor.receiveShadow = true;
    this.showroomRoot.add(floor);

    const grid = new THREE.GridHelper(12, 36, 0x5ab4a0, 0x78908a);
    grid.name = "Showroom floor grid";
    grid.position.y = 0.004;
    const gridMaterial = grid.material as THREE.Material | THREE.Material[];
    for (const material of Array.isArray(gridMaterial) ? gridMaterial : [gridMaterial]) {
      material.transparent = true;
      material.opacity = 0.2;
      material.depthWrite = false;
    }
    this.showroomRoot.add(grid);

    const backPanel = new THREE.Mesh(
      new THREE.PlaneGeometry(14, 7),
      new THREE.MeshBasicMaterial({
        color: 0x21332f,
        transparent: true,
        opacity: 0.36,
        side: THREE.DoubleSide,
      }),
    );
    backPanel.name = "Showroom rear plane";
    backPanel.position.set(0, 3.2, -5.2);
    this.showroomRoot.add(backPanel);

    const referenceLines = new THREE.Group();
    referenceLines.name = "Showroom reference lines";
    const lineMaterial = new THREE.LineBasicMaterial({
      color: 0x9ce0c9,
      transparent: true,
      opacity: 0.18,
    });
    for (const y of [1.2, 2.4, 3.6, 4.8]) {
      const points = [new THREE.Vector3(-6, y, -5.15), new THREE.Vector3(6, y, -5.15)];
      referenceLines.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), lineMaterial));
    }
    for (const x of [-4, -2, 0, 2, 4]) {
      const points = [new THREE.Vector3(x, 0.35, -5.14), new THREE.Vector3(x, 5.4, -5.14)];
      referenceLines.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), lineMaterial));
    }
    this.showroomRoot.add(referenceLines);
  }

  private resize(): void {
    const width = Math.max(this.stage.clientWidth, 1);
    const height = Math.max(this.stage.clientHeight, 1);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  private buildModelScene(): void {
    this.disposeSceneObjects();
    this.mujocoRoot = new THREE.Group();
    this.mujocoRoot.name = "MuJoCo Root";
    this.scene.add(this.mujocoRoot);
    this.bodyGroups.clear();

    const meshCache = new Map<number, THREE.BufferGeometry>();
    for (let geomId = 0; geomId < this.model.ngeom; geomId += 1) {
      if (this.model.geom_group[geomId] >= 3) {
        continue;
      }
      const bodyId = this.model.geom_bodyid[geomId];
      const body = this.ensureBodyGroup(bodyId);
      const object = this.createGeomObject(geomId, meshCache);
      if (!object) {
        continue;
      }
      getPosition(this.model.geom_pos, geomId, object.position);
      getQuaternion(this.model.geom_quat, geomId, object.quaternion);
      if (this.model.geom_type[geomId] === this.mujoco.mjtGeom.mjGEOM_PLANE.value) {
        object.rotateX(-Math.PI / 2);
      }
      body.add(object);
    }
  }

  private ensureBodyGroup(bodyId: number): THREE.Group {
    const existing = this.bodyGroups.get(bodyId);
    if (existing) {
      return existing;
    }
    const group = new THREE.Group();
    group.name = `body-${bodyId}`;
    this.bodyGroups.set(bodyId, group);
    this.mujocoRoot?.add(group);
    return group;
  }

  private createGeomObject(
    geomId: number,
    meshCache: Map<number, THREE.BufferGeometry>,
  ): THREE.Object3D | undefined {
    const type = this.model.geom_type[geomId];
    const size = this.model.geom_size;
    const sx = size[geomId * 3];
    const sy = size[geomId * 3 + 1];
    const sz = size[geomId * 3 + 2];
    let geometry: THREE.BufferGeometry | undefined;

    if (type === this.mujoco.mjtGeom.mjGEOM_PLANE.value) {
      geometry = new THREE.PlaneGeometry(60, 60);
    } else if (type === this.mujoco.mjtGeom.mjGEOM_SPHERE.value) {
      geometry = new THREE.SphereGeometry(sx, 32, 18);
    } else if (type === this.mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
      geometry = new THREE.CapsuleGeometry(sx, sy * 2, 12, 24);
    } else if (type === this.mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
      geometry = new THREE.SphereGeometry(1, 32, 18);
    } else if (type === this.mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
      geometry = new THREE.CylinderGeometry(sx, sx, sy * 2, 32);
    } else if (type === this.mujoco.mjtGeom.mjGEOM_BOX.value) {
      geometry = new THREE.BoxGeometry(sx * 2, sz * 2, sy * 2);
    } else if (type === this.mujoco.mjtGeom.mjGEOM_MESH.value) {
      const meshId = this.model.geom_dataid[geomId];
      geometry = meshCache.get(meshId);
      if (!geometry) {
        geometry = this.createMeshGeometry(meshId);
        meshCache.set(meshId, geometry);
      }
    }

    if (!geometry) {
      return undefined;
    }

    const material = this.createMaterial(geomId);
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = type !== this.mujoco.mjtGeom.mjGEOM_PLANE.value;
    mesh.receiveShadow = true;
    if (type === this.mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
      mesh.scale.set(sx, sz, sy);
    }
    return mesh;
  }

  private createMeshGeometry(meshId: number): THREE.BufferGeometry {
    const geometry = new THREE.BufferGeometry();
    const vertStart = this.model.mesh_vertadr[meshId] * 3;
    const vertCount = this.model.mesh_vertnum[meshId];
    const positions = new Float32Array(vertCount * 3);
    for (let i = 0; i < vertCount; i += 1) {
      const src = vertStart + i * 3;
      positions[i * 3] = this.model.mesh_vert[src];
      positions[i * 3 + 1] = this.model.mesh_vert[src + 2];
      positions[i * 3 + 2] = -this.model.mesh_vert[src + 1];
    }
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    const normalStart = this.model.mesh_normaladr[meshId] * 3;
    const normalCount = this.model.mesh_normalnum[meshId] || vertCount;
    if (normalStart >= 0 && normalCount >= vertCount) {
      const normals = new Float32Array(vertCount * 3);
      for (let i = 0; i < vertCount; i += 1) {
        const src = normalStart + i * 3;
        normals[i * 3] = this.model.mesh_normal[src];
        normals[i * 3 + 1] = this.model.mesh_normal[src + 2];
        normals[i * 3 + 2] = -this.model.mesh_normal[src + 1];
      }
      geometry.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    }

    const texStart = this.model.mesh_texcoordadr[meshId] * 2;
    const texCount = this.model.mesh_texcoordnum[meshId] || 0;
    if (texStart >= 0 && texCount >= vertCount) {
      const uvs = new Float32Array(vertCount * 2);
      for (let i = 0; i < vertCount * 2; i += 1) {
        uvs[i] = this.model.mesh_texcoord[texStart + i];
      }
      geometry.setAttribute("uv", new THREE.BufferAttribute(uvs, 2));
    }

    const faceStart = this.model.mesh_faceadr[meshId] * 3;
    const faceCount = this.model.mesh_facenum[meshId] * 3;
    const indices = new Uint32Array(faceCount);
    for (let i = 0; i < faceCount; i += 1) {
      indices[i] = this.model.mesh_face[faceStart + i];
    }
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    if (!geometry.getAttribute("normal")) {
      geometry.computeVertexNormals();
    }
    geometry.computeBoundingSphere();
    return geometry;
  }

  private createMaterial(geomId: number): THREE.Material {
    if (this.model.geom_type[geomId] === this.mujoco.mjtGeom.mjGEOM_PLANE.value) {
      return new THREE.MeshStandardMaterial({
        color: 0xbac4c1,
        roughness: 0.86,
        metalness: 0.02,
        side: THREE.DoubleSide,
      });
    }
    const rgba = this.resolveGeomRgba(geomId);
    return new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      roughness: 0.68,
      metalness: 0.1,
      transparent: rgba[3] < 1,
      opacity: rgba[3],
      side:
        this.model.geom_type[geomId] === this.mujoco.mjtGeom.mjGEOM_PLANE.value
          ? THREE.DoubleSide
          : THREE.FrontSide,
    });
  }

  private resolveGeomRgba(geomId: number): [number, number, number, number] {
    const matId = this.model.geom_matid[geomId];
    if (matId >= 0) {
      return [
        this.model.mat_rgba[matId * 4],
        this.model.mat_rgba[matId * 4 + 1],
        this.model.mat_rgba[matId * 4 + 2],
        this.model.mat_rgba[matId * 4 + 3],
      ];
    }
    return [
      this.model.geom_rgba[geomId * 4],
      this.model.geom_rgba[geomId * 4 + 1],
      this.model.geom_rgba[geomId * 4 + 2],
      this.model.geom_rgba[geomId * 4 + 3],
    ];
  }

  private updateBodyTransforms(): void {
    for (const [bodyId, group] of this.bodyGroups) {
      getPosition(this.data.xpos, bodyId, group.position);
      getQuaternion(this.data.xquat, bodyId, group.quaternion);
      group.updateMatrixWorld();
    }
  }

  private disposeSceneObjects(): void {
    if (!this.mujocoRoot) {
      return;
    }
    this.mujocoRoot.traverse((object) => {
      if (object instanceof THREE.Mesh) {
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        materials.forEach((material) => material.dispose());
      }
    });
    this.scene.remove(this.mujocoRoot);
    this.mujocoRoot = undefined;
  }

  private disposeShowroomScene(): void {
    if (!this.showroomRoot) {
      return;
    }
    this.showroomRoot.traverse((object) => {
      if (object instanceof THREE.Mesh || object instanceof THREE.Line) {
        object.geometry.dispose();
        const materials = Array.isArray(object.material)
          ? object.material
          : [object.material];
        materials.forEach((material) => material.dispose());
      }
    });
    this.scene.remove(this.showroomRoot);
    this.showroomRoot = undefined;
  }
}

function getPosition(
  buffer: NumericView,
  index: number,
  target: THREE.Vector3,
): THREE.Vector3 {
  return target.set(
    buffer[index * 3],
    buffer[index * 3 + 2],
    -buffer[index * 3 + 1],
  );
}

function getQuaternion(
  buffer: NumericView,
  index: number,
  target: THREE.Quaternion,
): THREE.Quaternion {
  return target.set(
    -buffer[index * 4 + 1],
    -buffer[index * 4 + 3],
    buffer[index * 4 + 2],
    -buffer[index * 4],
  );
}

function smoothstep(value: number): number {
  const s = clamp(value, 0, 1);
  return s * s * (3 - 2 * s);
}
