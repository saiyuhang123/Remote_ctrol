'use strict';
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');

const source = fs.readFileSync(path.join(__dirname, '..', '..', 'web', 'map-loader.js'), 'utf8');
const context = { window: {}, console };
vm.createContext(context);
vm.runInContext(source, context);
const Projection = context.window.SiteMapProjection;
assert(Projection, 'SiteMapProjection must be exported');

const deg = value => value * Math.PI / 180;
function almost(actual, expected, tolerance = 1e-8) {
  assert(Math.abs(actual - expected) <= tolerance,
         `expected ${expected}, got ${actual}`);
}
for (const yawDeg of [0, 30, 90]) {
  const meta = {
    width: 100, height: 80, resolution: 0.05,
    origin: [-3.25, 4.5, deg(yawDeg)], image: 'active.png',
  };
  const projection = new Projection(meta);
  // Image corners to world and back.
  for (const [u, v] of [[0, 0], [99, 0], [0, 79], [99, 79], [37.5, 22.25]]) {
    const [x, y] = projection.imageToWorld(u, v);
    const [u2, v2] = projection.worldToImage(x, y);
    almost(u2, u, 1e-7); almost(v2, v, 1e-7);
  }
  // A world point expressed in the rotated local frame.
  const localX = 1.75, localY = -0.5;
  const cos = Math.cos(deg(yawDeg)), sin = Math.sin(deg(yawDeg));
  const worldX = meta.origin[0] + cos * localX - sin * localY;
  const worldY = meta.origin[1] + sin * localX + cos * localY;
  const [u, v] = projection.worldToImage(worldX, worldY);
  almost(u, localX / meta.resolution, 1e-7);
  almost(v, meta.height - localY / meta.resolution, 1e-7);
  almost(projection.worldYawToScreen(deg(yawDeg + 15)), deg(-15), 1e-8);
}
console.log('map transform tests passed (origin yaw 0, 30, 90 degrees)');
