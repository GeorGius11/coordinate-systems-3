function calcDistance(point1, point2) {
  return Math.sqrt((point2.x - point1.x) ** 2 + (point2.y - point1.y) ** 2);
}

function trilateration(x1, y1, r1, x2, y2, r2, x3, y3, r3) {
  var A = 2 * x2 - 2 * x1;
  var B = 2 * y2 - 2 * y1;
  var C = r1 ** 2 - r2 ** 2 - x1 ** 2 - y1 ** 2 + x2 ** 2 + y2 ** 2;
  var D = 2 * x3 - 2 * x2;
  var E = 2 * y3 - 2 * y2;
  var F = r2 ** 2 - r3 ** 2 - x2 ** 2 - y2 ** 2 + x3 ** 2 + y3 ** 2;
  var x = (C * E - F * B) / (E * A - B * D);
  var y = (C * D - A * F) / (B * D - A * E);
  return { x: x, y: y };
}

function triangulate(x1, y1, theta1, x2, y2, theta2) {
  let tanTheta1 = Math.tan(theta1);
  let tanTheta2 = Math.tan(theta2);
  let x = (tanTheta1 * x1 - tanTheta2 * x2 + y2 - y1) / (tanTheta1 - tanTheta2);
  let y =
    (y1 * tanTheta2 - y2 * tanTheta1 + (x2 - x1) * tanTheta1 * tanTheta2) /
    (tanTheta2 - tanTheta1);
  return { x: x, y: y };
}

function euclideanDistance(base1, y1, base2, y2) {
  return Math.sqrt((base2 - base1) ** 2 + (y2 - y1) ** 2);
}

function lossFunction(x, y, anchors, distances) {
  let loss = 0;
  for (let i = 0; i < anchors.length; i++) {
    const measuredDistance = distances[i];
    const calculatedDistance = euclideanDistance(
      x,
      y,
      anchors[i][0],
      anchors[i][1]
    );
    loss += (calculatedDistance - measuredDistance) ** 2;
  }
  return loss;
}

function gradient(x, y, anchors, distances) {
  let gradX = 0,
    gradY = 0;
  for (let i = 0; i < anchors.length; i++) {
    const dx = x - anchors[i][0];
    const dy = y - anchors[i][1];
    const measuredDistance = distances[i];
    const calculatedDistance = euclideanDistance(
      x,
      y,
      anchors[i][0],
      anchors[i][1]
    );
    gradX +=
      (calculatedDistance - measuredDistance) * (dx / calculatedDistance);
    gradY +=
      (calculatedDistance - measuredDistance) * (dy / calculatedDistance);
  }
  return [gradX, gradY];
}

function gradientDescent(
  anchors,
  distances,
  initialGuess,
  learningRate,
  iterations
) {
  let [x, y] = initialGuess;
  for (let i = 0; i < iterations; i++) {
    const [gradX, gradY] = gradient(x, y, anchors, distances);
    x -= learningRate * gradX;
    y -= learningRate * gradY;
  }
  return [x, y];
}

function addNoise(x, y, noise) {
  const noiseX = (Math.random() - 0.5) * noise;
  const noiseY = (Math.random() - 0.5) * noise;

  return { x: x + noiseX, y: y + noiseY };
}

const base1 = { x: -22, y: 19 };
const base2 = { x: 5, y: 63 };
const base3 = { x: 0, y: -18 };
const objectCoord = { x: 27, y: -9 };

const distToBase1 = calcDistance(objectCoord, base1);
const distToBase2 = calcDistance(objectCoord, base2);
const distToBase3 = calcDistance(objectCoord, base3);

console.log("\nResults without noise:");
let trilaterationResult = trilateration(
  base1.x,
  base1.y,
  distToBase1,
  base2.x,
  base2.y,
  distToBase2,
  base3.x,
  base3.y,
  distToBase3
);
console.log(
  `Trilateration: x = ${trilaterationResult.x.toFixed(
    2
  )}, y = ${trilaterationResult.y.toFixed(2)};`
);

let triangulationResult = triangulate(
  base1.x,
  base1.y,
  Math.atan2(objectCoord.y - base1.y, objectCoord.x - base1.x),
  base2.x,
  base2.y,
  Math.atan2(objectCoord.y - base2.y, objectCoord.x - base2.x)
);
console.log(
  `Triangulation: x = ${triangulationResult.x.toFixed(
    2
  )}, y = ${triangulationResult.y.toFixed(2)};`
);

let estimatedPosition = gradientDescent(
  [
    [base1.x, base1.y],
    [base2.x, base2.y],
    [base3.x, base3.y],
  ],
  [distToBase1, distToBase2, distToBase3],
  [objectCoord.x, objectCoord.y],
  0.01,
  1000
);
console.log(
  `Gradient Descent: x = ${estimatedPosition[0].toFixed(
    2
  )}, y = ${estimatedPosition[1].toFixed(2)};`
);

const noiseLevel = 3;

const noisyBase1 = addNoise(base1.x, base1.y, noiseLevel);
const noisyBase2 = addNoise(base2.x, base2.y, noiseLevel);
const noisyBase3 = addNoise(base3.x, base3.y, noiseLevel);
const noisyObject = addNoise(objectCoord.x, objectCoord.y, noiseLevel);

console.log("\nResults with noise:");
console.log("Noise: " + noiseLevel);
trilaterationResult = trilateration(
  noisyBase1.x,
  noisyBase1.y,
  distToBase1,
  noisyBase2.x,
  noisyBase2.y,
  distToBase2,
  noisyBase3.x,
  noisyBase3.y,
  distToBase3
);
console.log(
  `Trilateration: x = ${trilaterationResult.x.toFixed(
    2
  )}, y = ${trilaterationResult.y.toFixed(2)};`
);

triangulationResult = triangulate(
  noisyBase1.x,
  noisyBase1.y,
  Math.atan2(noisyObject.y - base1.y, noisyObject.x - base1.x),
  noisyBase2.x,
  noisyBase2.y,
  Math.atan2(noisyObject.y - base2.y, noisyObject.x - base2.x)
);

console.log(
  `Triangulation: x = ${triangulationResult.x.toFixed(
    2
  )}, y = ${triangulationResult.y.toFixed(2)};`
);

estimatedPosition = gradientDescent(
  [
    [noisyBase1.x, noisyBase1.y],
    [noisyBase2.x, noisyBase2.y],
    [noisyBase3.x, noisyBase3.y],
  ],
  [distToBase1, distToBase2, distToBase3],
  [noisyObject.x, noisyObject.y],
  0.01,
  1000
);
console.log(
  `Gradient Descent: x = ${estimatedPosition[0].toFixed(
    2
  )}, y = ${estimatedPosition[1].toFixed(2)};`
);
