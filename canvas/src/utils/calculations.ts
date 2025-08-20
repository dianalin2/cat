type Position = {
  x: number;
  y: number;
}

// Assuming only 2 joints and that angle is positive
export function calculateJointPos(goalPos: Position, lengths: number[]): Position {
  const x = goalPos.x;
  const y = goalPos.y;
  const l1 = lengths[0];
  const l2 = lengths[1];
  const theta2 = Math.acos((x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2));
  const theta1 = Math.atan(x / y) - Math.atan((l2 * Math.sin(theta2)) / (l1 + l2 * Math.cos(theta2)));
  console.log("Inner joint: ", rad2Deg(theta2), "\nCenter joint: ", rad2Deg(theta1));
  const jointPos: Position = {
    x: l1 * Math.sin(theta1),
    y: l1 * Math.cos(theta1),
  }
  return jointPos;
}

export function vectorDiff(v1: Position, v2: Position): Position {
  const translatedPos: Position = {
    x: v1.x - v2.x,
    y: v1.y - v2.y,
  }
  return translatedPos;
}

export function vectorAdd(v1: Position, v2: Position): Position {
  const translatedPos: Position = {
    x: v1.x + v2.x,
    y: v1.y + v2.y,
  }
  return translatedPos;
}

function rad2Deg(radians: number): number {
  return radians * 180 / Math.PI;
}
