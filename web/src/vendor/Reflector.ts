/**
 * Planar reflector that blends the mirror image 50/50 with a lit, textured
 * floor material — the look used by mujoco_wasm-style hosted viewers
 * (e.g. thomasonzhou.github.io/mujoco_anywhere).
 *
 * Adapted from zalo/mujoco_wasm's examples/utils/Reflector.js (itself derived
 * from three.js examples' Reflector, MIT license), updated for modern three:
 * the removed `LinearEncoding`/`outputEncoding` API is no longer used —
 * render targets are not output-encoded in three >= r152.
 */

import {
  HalfFloatType,
  Matrix4,
  Mesh,
  MeshPhysicalMaterial,
  NoToneMapping,
  PerspectiveCamera,
  Plane,
  Texture,
  Vector3,
  Vector4,
  WebGLRenderTarget,
  type BufferGeometry,
  type WebGLRenderer,
  type Scene,
  type Camera,
} from "three";

export interface ReflectorOptions {
  texture?: Texture;
  textureWidth?: number;
  textureHeight?: number;
  clipBias?: number;
  multisample?: number;
  /** 0 = all mirror, 1 = all floor material. Reference viewers use 0.5. */
  materialMix?: number;
}

export class Reflector extends Mesh {
  readonly isReflector = true;
  declare material: MeshPhysicalMaterial;
  camera = new PerspectiveCamera();
  getRenderTarget: () => WebGLRenderTarget;
  disposeReflector: () => void;

  constructor(geometry: BufferGeometry, options: ReflectorOptions = {}) {
    super(geometry);

    const scope = this;

    const textureWidth = options.textureWidth ?? 512;
    const textureHeight = options.textureHeight ?? 512;
    const clipBias = options.clipBias ?? 0;
    const multisample = options.multisample ?? 4;
    const blendTexture = options.texture;
    const materialMix = options.materialMix ?? 0.5;

    const reflectorPlane = new Plane();
    const normal = new Vector3();
    const reflectorWorldPosition = new Vector3();
    const cameraWorldPosition = new Vector3();
    const rotationMatrix = new Matrix4();
    const lookAtPosition = new Vector3(0, 0, -1);
    const clipPlane = new Vector4();

    const view = new Vector3();
    const target = new Vector3();
    const q = new Vector4();

    const textureMatrix = new Matrix4();
    const virtualCamera = this.camera;

    const renderTarget = new WebGLRenderTarget(textureWidth, textureHeight, {
      samples: multisample,
      type: HalfFloatType,
    });

    this.material = new MeshPhysicalMaterial({ map: blendTexture ?? null });
    this.material.polygonOffset = true;
    this.material.polygonOffsetFactor = 1;
    this.material.onBeforeCompile = (shader) => {
      // vertex: project positions through the mirror camera for tDiffuse lookup
      let bodyStart = shader.vertexShader.indexOf("void main() {");
      shader.vertexShader =
        shader.vertexShader.slice(0, bodyStart) +
        "\nuniform mat4 textureMatrix;\nvarying vec4 vUv3;\n" +
        shader.vertexShader.slice(bodyStart, -1) +
        "\tvUv3 = textureMatrix * vec4( position, 1.0 );\n}";

      // fragment: blend the mirror render with the lit floor material
      bodyStart = shader.fragmentShader.indexOf("void main() {");
      shader.fragmentShader =
        "\nuniform sampler2D tDiffuse;\nvarying vec4 vUv3;\n" +
        shader.fragmentShader.slice(0, bodyStart) +
        shader.fragmentShader.slice(bodyStart, -1) +
        `\tgl_FragColor = vec4( mix( texture2DProj( tDiffuse, vUv3 ).rgb, gl_FragColor.rgb, ${materialMix.toFixed(3)} ), 1.0 );\n}`;

      shader.uniforms.tDiffuse = { value: renderTarget.texture };
      shader.uniforms.textureMatrix = { value: textureMatrix };
      this.material.userData.shader = shader;
    };
    this.receiveShadow = true;

    this.onBeforeRender = function (
      renderer: WebGLRenderer,
      scene: Scene,
      camera: Camera,
    ) {
      reflectorWorldPosition.setFromMatrixPosition(scope.matrixWorld);
      cameraWorldPosition.setFromMatrixPosition(camera.matrixWorld);

      rotationMatrix.extractRotation(scope.matrixWorld);

      normal.set(0, 0, 1);
      normal.applyMatrix4(rotationMatrix);

      view.subVectors(reflectorWorldPosition, cameraWorldPosition);

      // avoid rendering when the reflector faces away
      if (view.dot(normal) > 0) return;

      view.reflect(normal).negate();
      view.add(reflectorWorldPosition);

      rotationMatrix.extractRotation(camera.matrixWorld);

      lookAtPosition.set(0, 0, -1);
      lookAtPosition.applyMatrix4(rotationMatrix);
      lookAtPosition.add(cameraWorldPosition);

      target.subVectors(reflectorWorldPosition, lookAtPosition);
      target.reflect(normal).negate();
      target.add(reflectorWorldPosition);

      virtualCamera.position.copy(view);
      virtualCamera.up.set(0, 1, 0);
      virtualCamera.up.applyMatrix4(rotationMatrix);
      virtualCamera.up.reflect(normal);
      virtualCamera.lookAt(target);

      virtualCamera.far = (camera as PerspectiveCamera).far;

      virtualCamera.updateMatrixWorld();
      virtualCamera.projectionMatrix.copy(
        (camera as PerspectiveCamera).projectionMatrix,
      );

      textureMatrix.set(
        0.5, 0.0, 0.0, 0.5,
        0.0, 0.5, 0.0, 0.5,
        0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, 0.0, 1.0,
      );
      textureMatrix.multiply(virtualCamera.projectionMatrix);
      textureMatrix.multiply(virtualCamera.matrixWorldInverse);
      textureMatrix.multiply(scope.matrixWorld);

      // oblique near-plane clipping (Lengyel): clip at the mirror plane
      reflectorPlane.setFromNormalAndCoplanarPoint(
        normal,
        reflectorWorldPosition,
      );
      reflectorPlane.applyMatrix4(virtualCamera.matrixWorldInverse);

      clipPlane.set(
        reflectorPlane.normal.x,
        reflectorPlane.normal.y,
        reflectorPlane.normal.z,
        reflectorPlane.constant,
      );

      const projectionMatrix = virtualCamera.projectionMatrix;

      q.x =
        (Math.sign(clipPlane.x) + projectionMatrix.elements[8]) /
        projectionMatrix.elements[0];
      q.y =
        (Math.sign(clipPlane.y) + projectionMatrix.elements[9]) /
        projectionMatrix.elements[5];
      q.z = -1.0;
      q.w =
        (1.0 + projectionMatrix.elements[10]) / projectionMatrix.elements[14];

      clipPlane.multiplyScalar(2.0 / clipPlane.dot(q));

      projectionMatrix.elements[2] = clipPlane.x;
      projectionMatrix.elements[6] = clipPlane.y;
      projectionMatrix.elements[10] = clipPlane.z + 1.0 - clipBias;
      projectionMatrix.elements[14] = clipPlane.w;

      scope.visible = false;

      const currentRenderTarget = renderer.getRenderTarget();
      const currentXrEnabled = renderer.xr.enabled;
      const currentShadowAutoUpdate = renderer.shadowMap.autoUpdate;
      const currentToneMapping = renderer.toneMapping;

      renderer.xr.enabled = false;
      renderer.shadowMap.autoUpdate = false;
      renderer.toneMapping = NoToneMapping;

      renderer.setRenderTarget(renderTarget);
      renderer.state.buffers.depth.setMask(true);
      if (renderer.autoClear === false) renderer.clear();
      renderer.render(scene, virtualCamera);

      renderer.xr.enabled = currentXrEnabled;
      renderer.shadowMap.autoUpdate = currentShadowAutoUpdate;
      renderer.toneMapping = currentToneMapping;

      renderer.setRenderTarget(currentRenderTarget);

      const viewport = (camera as PerspectiveCamera & { viewport?: Vector4 })
        .viewport;
      if (viewport !== undefined) {
        renderer.state.viewport(viewport);
      }

      scope.visible = true;
    };

    this.getRenderTarget = () => renderTarget;
    this.disposeReflector = () => {
      renderTarget.dispose();
      scope.material.dispose();
    };
  }
}
