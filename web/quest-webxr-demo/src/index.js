/**
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { Text } from 'troika-three-text';
import { XR_BUTTONS } from 'gamepad-wrapper';
import { gsap } from 'gsap';
import { init } from './init.js';

// Bullet setup
const bulletGeometry = new THREE.SphereGeometry(0.02);
const bulletMaterial = new THREE.MeshStandardMaterial({ color: 'gray' });
const bulletPrototype = new THREE.Mesh(bulletGeometry, bulletMaterial);

const forwardVector = new THREE.Vector3(0, 0, -1);
const bulletSpeed = 10;
const bulletTimeToLive = 1;
const bullets = {};

// Setup AR camera background
let cameraTexture = null;
let cameraMaterial = null;
let videoElement = null;

function setupScene({ scene, camera, renderer, player, controllers }) {
	// Set up AR environment (WebXR)
	const xr = renderer.xr;

	// Create video element for background
	videoElement = document.createElement('video');
	videoElement.width = 640;
	videoElement.height = 480;
	videoElement.autoplay = true;
	videoElement.playsInline = true;
	videoElement.loop = true;

	// Access the front camera (user's camera)
	navigator.mediaDevices
		.getUserMedia({ video: { facingMode: 'user' } })
		.then((stream) => {
			videoElement.srcObject = stream;
		})
		.catch((err) => {
			console.error('Error accessing front camera: ', err);
			// alert('Failed to access front camera. Please check your permissions or device settings.');
		});

	// Create texture from video element for AR background
	cameraTexture = new THREE.VideoTexture(videoElement);
	cameraTexture.minFilter = THREE.LinearFilter;
	cameraTexture.magFilter = THREE.LinearFilter;
	cameraTexture.format = THREE.RGBFormat;

	// Apply the video texture to the background
	cameraMaterial = new THREE.MeshBasicMaterial({
		map: cameraTexture,
		transparent: true,
	});

	// Create a large plane to serve as the background
	const planeGeometry = new THREE.PlaneGeometry(6, 6); // 6x6m plane
	const plane = new THREE.Mesh(planeGeometry, cameraMaterial);
	scene.add(plane);

	// Setup objects (cube, sphere, etc.)
	const floorGeometry = new THREE.PlaneGeometry(6, 6); // 6x6m floor
	const floorMaterial = new THREE.MeshStandardMaterial({ color: 'white' }); // white floor
	const floor = new THREE.Mesh(floorGeometry, floorMaterial);
	floor.rotateX(-Math.PI / 2);
	scene.add(floor);

	const coneGeometry = new THREE.ConeGeometry(0.6, 1.5);
	const coneMaterial = new THREE.MeshStandardMaterial({ color: 'purple' });
	const cone = new THREE.Mesh(coneGeometry, coneMaterial);
	scene.add(cone);
	cone.position.set(0.4, 0.75, -1.5);

	const cubeGeometry = new THREE.BoxGeometry(1, 1, 1);
	const cubeMaterial = new THREE.MeshStandardMaterial({ color: 'orange' });
	const cube = new THREE.Mesh(cubeGeometry, cubeMaterial);
	scene.add(cube);
	cube.position.set(-0.8, 0.5, -1.5);
	cube.rotateY(Math.PI / 4);

	const sphereGeometry = new THREE.SphereGeometry(0.4);
	const sphereMaterial = new THREE.MeshStandardMaterial({ color: 'red' });
	const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
	scene.add(sphere);
	sphere.position.set(0.6, 0.4, -0.5);
	sphere.scale.set(1.2, 1.2, 1.2);
}

function onFrame(delta, time, { scene, camera, renderer, player, controllers }) {
	if (controllers.right) {
		const { gamepad, raySpace } = controllers.right;
		if (gamepad.getButtonClick(XR_BUTTONS.TRIGGER)) {
			const bullet = bulletPrototype.clone();
			scene.add(bullet);
			raySpace.getWorldPosition(bullet.position);
			raySpace.getWorldQuaternion(bullet.quaternion);

			const directionVector = forwardVector
				.clone()
				.applyQuaternion(bullet.quaternion);
			bullet.userData = {
				velocity: directionVector.multiplyScalar(bulletSpeed),
				timeToLive: bulletTimeToLive,
			};
			bullets[bullet.uuid] = bullet;
		}
	}

	Object.values(bullets).forEach((bullet) => {
		if (bullet.userData.timeToLive < 0) {
			delete bullets[bullet.uuid];
			scene.remove(bullet);
			return;
		}
		const deltaVec = bullet.userData.velocity.clone().multiplyScalar(delta);
		bullet.position.add(deltaVec);
		bullet.userData.timeToLive -= delta;
	});
}

init(setupScene, onFrame);