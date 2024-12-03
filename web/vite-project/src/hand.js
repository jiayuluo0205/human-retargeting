// main.js
import { WebXRButton } from '../js/util/webxr-button.js';
import { Scene } from '../js/render/scenes/scene.js';
import { Node } from '../js/render/core/node.js';
import { Gltf2Node } from '../js/render/nodes/gltf2.js';
import { SkyboxNode } from '../js/render/nodes/skybox.js';
import { Renderer, createWebGLContext } from '../js/render/core/renderer.js';
import { BoxBuilder } from '../js/render/geometry/box-builder.js';
import { PbrMaterial } from '../js/render/materials/pbr.js';
import { QueryArgs } from '../js/util/query-args.js';
import { mat4 } from '../js/render/math/gl-matrix.js';
import { vec3 } from '../js/render/math/gl-matrix.js';
import { Ray } from '../js/render/math/ray.js';
import { fetchProfile } from 'https://cdn.jsdelivr.net/npm/@webxr-input-profiles/motion-controllers@1.0/dist/motion-controllers.module.js';

const DEFAULT_PROFILES_PATH = 'https://cdn.jsdelivr.net/npm/@webxr-input-profiles/assets@1.0/dist/profiles';

import WebXRPolyfill from '../js/third-party/webxr-polyfill/build/webxr-polyfill.module.js';
// import { init } from './init.js';

// 初始化全局变量
let xrButton = null;
let xrRefSpace = null;
let isAR = true;
let radii = new Float32Array(25);
let positions = new Float32Array(16 * 25);
let boxes_left = [];
let boxes_right = [];
let boxes_none = [];
let tracked_boxes_left = [];
let tracked_boxes_right = [];
let tracked_boxes_none = [];
let boxes = { input_left: boxes_left, input_right: boxes_right, input_none: boxes_none, tracked_left: tracked_boxes_left, tracked_right: tracked_boxes_right, tracked_none: tracked_boxes_none };
let indexFingerBoxes = { input_left: null, input_right: null, tracked_left: null, tracked_right: null };
const defaultBoxColor = { r: 0.5, g: 0.5, b: 0.5 };
const leftBoxColor = { r: 1, g: 0, b: 1 };
const rightBoxColor = { r: 0, g: 1, b: 1 };

let interactionBox = null;
let leftInteractionBox = null;
let rightInteractionBox = null;

let gl = null;
let renderer = null;
let scene = new Scene();
let usePolyfill = true;

let solarSystem = new Gltf2Node({url: 'media/gltf/space/space.gltf'});
solarSystem.scale = [0.1, 0.1, 0.1];
scene.addNode(solarSystem);
let skybox = new SkyboxNode({url: 'media/textures/milky-way-4k.png'});
scene.addNode(skybox);

if (QueryArgs.getBool('usePolyfill', true)) {
  new WebXRPolyfill({ debug: false });
}

scene.enableStats(false); // 禁用 FPS 计数器

// init();

function createBoxPrimitive(r, g, b) {
  let boxBuilder = new BoxBuilder();
  boxBuilder.pushCube([0, 0, 0], 1);
  let boxPrimitive = boxBuilder.finishPrimitive(renderer);
  let boxMaterial = new PbrMaterial();
  boxMaterial.baseColorFactor.value = [r, g, b, 1];
  return renderer.createRenderPrimitive(boxPrimitive, boxMaterial);
}

function addBox(x, y, z, r, g, b, offset) {
  let boxRenderPrimitive = createBoxPrimitive(r, g, b);
  let boxNode = new Node();
  boxNode.addRenderPrimitive(boxRenderPrimitive);
  boxNode.selectable = true;
  return boxNode;
}

function initHands() {
  for (const box of boxes_left) scene.removeNode(box);
  for (const box of boxes_right) scene.removeNode(box);
  boxes_left = [];
  boxes_right = [];
  boxes = { input_left: boxes_left, input_right: boxes_right, input_none: boxes_none, tracked_left: tracked_boxes_left, tracked_right: tracked_boxes_right, tracked_none: tracked_boxes_none };

  if (typeof XRHand !== 'undefined') {
    for (let i = 0; i <= 24; i++) {
      const r = .6 + Math.random() * .4;
      const g = .6 + Math.random() * .4;
      const b = .6 + Math.random() * .4;
      boxes_left.push(addBox(0, 0, 0, r, g, b));
      boxes_right.push(addBox(0, 0, 0, r, g, b));
      tracked_boxes_left.push(addBox(0, 0, 0, r, g, b));
      tracked_boxes_right.push(addBox(0, 0, 0, r, g, b));
    }
  }

  if (indexFingerBoxes.input_left) scene.removeNode(indexFingerBoxes.left);
  if (indexFingerBoxes.input_right) scene.removeNode(indexFingerBoxes.input_right);
  if (indexFingerBoxes.tracked_left) scene.removeNode(indexFingerBoxes.tracked_left);
  if (indexFingerBoxes.tracked_right) scene.removeNode(indexFingerBoxes.tracked_right);

  indexFingerBoxes.input_left = addBox(0, 0, 0, leftBoxColor.r, leftBoxColor.g, leftBoxColor.b);
  indexFingerBoxes.input_right = addBox(0, 0, 0, rightBoxColor.r, rightBoxColor.g, rightBoxColor.b);
  indexFingerBoxes.tracked_left = addBox(0, 0, 0, leftBoxColor.r, leftBoxColor.g, leftBoxColor.b);
  indexFingerBoxes.tracked_right = addBox(0, 0, 0, rightBoxColor.r, rightBoxColor.g, rightBoxColor.b);
}

function initXR() {
  xrButton = new WebXRButton({
    onRequestSession: onRequestSession,
    onEndSession: onEndSession,
    textEnterXRTitle: "START AR",
    textXRNotFoundTitle: "AR NOT FOUND",
    textExitXRTitle: "EXIT AR",
  });
  document.querySelector('header').appendChild(xrButton.domElement);

  if (navigator.xr) {
    navigator.xr.isSessionSupported('immersive-ar').then((supported) => {
      if (supported) xrButton.enabled = supported;
      else xrButton.enabled = false;
    });
    // navigator.xr.requestSession('inline').then(onSessionStarted);
  }
}

function onRequestSession() {
  return navigator.xr.requestSession('immersive-ar', { optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking'] }).then(onSessionStarted);
}

function initGL() {
  if (gl)
    return;

  gl = createWebGLContext({
    xrCompatible: true
  });
  document.body.appendChild(gl.canvas);

  function onResize() {
    gl.canvas.width = gl.canvas.clientWidth * window.devicePixelRatio;
    gl.canvas.height = gl.canvas.clientHeight * window.devicePixelRatio;
  }
  window.addEventListener('resize', onResize);
  onResize();

  renderer = new Renderer(gl);

  scene.setRenderer(renderer);
}

function onSessionStarted(session) {
  xrButton.setSession(session);
  session.addEventListener('end', onSessionEnded);
  session.addEventListener('inputsourceschange', onInputSourcesChange);

  if (session.isImmersive) {
    skybox.visible = false;
  }
  initGL();

  initHands();

  scene.setRenderer(renderer);

  session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });

  session.requestReferenceSpace('local').then((refSpace) => {
    xrRefSpace = refSpace.getOffsetReferenceSpace(new XRRigidTransform({ x: 0, y: 0, z: 0 }));
    session.requestAnimationFrame(onXRFrame);
  });
}

function onEndSession(session) {
  session.end();
}

function onSessionEnded(event) {
  xrButton.setSession(null);
  renderer = null;
}

function onInputSourcesChange(event) {
  onSourcesChange(event, "input_");
}

function onSourcesChange(event, type) {
  for (let inputSource of event.added) {
    if (inputSource.targetRayMode == 'tracked-pointer') {
      fetchProfile(inputSource, DEFAULT_PROFILES_PATH).then(({ profile, assetPath }) => {
        scene.inputRenderer.setControllerMesh(new Gltf2Node({ url: assetPath }), inputSource.handedness, inputSource.profiles[0]);
      });
    }
  }
}

function updateInputSources(session, frame, refSpace) {
  updateSources(session, frame, refSpace, session.inputSources, "input_");
}

function updateSources(session, frame, refSpace, sources, type) {
  if (session.visibilityState === 'visible-blurred') return;
  for (let inputSource of sources) {
    let hand_type = type + inputSource.handedness;
    if (type == "input_") {
      let targetRayPose = frame.getPose(inputSource.targetRaySpace, refSpace);
      if (targetRayPose) {
        let targetRay = new Ray(targetRayPose.transform);
        let cursorDistance = 2.0;
        let cursorPos = vec3.fromValues(
          targetRay.origin.x,
          targetRay.origin.y,
          targetRay.origin.z
        );
        vec3.add(cursorPos, cursorPos, [
          targetRay.direction.x * cursorDistance,
          targetRay.direction.y * cursorDistance,
          targetRay.direction.z * cursorDistance,
        ]);
        scene.inputRenderer.addCursor(cursorPos);
      }
    }

    if (!inputSource.hand && inputSource.gripSpace) {
      let gripPose = frame.getPose(inputSource.gripSpace, refSpace);
      if (gripPose) {
        scene.inputRenderer.addController(gripPose.transform.matrix, inputSource.handedness, inputSource.profiles[0]);
      }
    }

    let offset = 0;
    if (!inputSource.hand) {
      for (const box of boxes[hand_type]) scene.removeNode(box);
      scene.removeNode(indexFingerBoxes[hand_type]);
      continue;
    } else {
      let pose = frame.getPose(inputSource.targetRaySpace, refSpace);
      if (pose === undefined) continue;

      if (!frame.fillJointRadii(inputSource.hand.values(), radii)) continue;
      if (!frame.fillPoses(inputSource.hand.values(), refSpace, positions)) continue;

      for (const box of boxes[hand_type]) {
        scene.addNode(box);
        let matrix = positions.slice(offset * 16, (offset + 1) * 16);
        let jointRadius = radii[offset];
        offset++;
        mat4.getTranslation(box.translation, matrix);
        mat4.getRotation(box.rotation, matrix);
        box.scale = [jointRadius, jointRadius, jointRadius];
      }

      const indexFingerBox = indexFingerBoxes[hand_type];
      scene.addNode(indexFingerBox);
      let joint = inputSource.hand.get('index-finger-tip');
      let jointPose = frame.getJointPose(joint, xrRefSpace);
      if (jointPose) {
        let matrix = jointPose.transform.matrix;
        mat4.getTranslation(indexFingerBox.translation, matrix);
        mat4.getRotation(indexFingerBox.rotation, matrix);
        indexFingerBox.scale = [0.007, 0.007, 0.007];
      }
    }
  }
}

function onXRFrame(t, frame) {
  let session = frame.session;

  scene.startFrame();
  session.requestAnimationFrame(onXRFrame);

  updateInputSources(session, frame, xrRefSpace);

  let pose = frame.getViewerPose(xrRefSpace);
  if (pose) {
    let glLayer = session.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, glLayer.framebuffer);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    for (let view of pose.views) {
      let viewport = glLayer.getViewport(view);
      gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
      scene.draw(view.projectionMatrix, view.transform);
    }
  }

  scene.endFrame();
}

initXR();