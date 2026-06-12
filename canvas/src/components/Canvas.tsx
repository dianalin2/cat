import { useState, useEffect, useContext } from 'react';
import { Stage, Layer, Rect, Circle, Line } from 'react-konva';
import { calculateJointPos, vectorDiff, vectorAdd } from '../utils/calculations';
import { RosContext } from '../RosProvider';
import { Topic } from 'roslib';

type Position = {
  x: number;
  y: number;
}

function Canvas() {
  const { ros, isConnected } = useContext(RosContext);

  const [servoAngles, setServoAngles] = useState<{ base: number; elbow: number }>({ base: 0, elbow: 0 });

  const widthPortion = 0.8;
  const heightPortion = 0.8;
  const [dimensions, setDimensions] = useState<Position>({
    x: window.innerWidth * widthPortion,
    y: window.innerHeight * heightPortion,
  });
  const [origin, setOrigin] = useState<Position>({
    x: dimensions.x / 2,
    y: 0
  })

  const [jointPositions, setJointPositions] = useState<Record<string, Position>>({
    goal: {
      x: dimensions.x / 2,
      y: dimensions.y / 2,
    },
    joint: {
      x: dimensions.x / 2 + 50,
      y: dimensions.y / 2 - 100,
    }
  })
  const lengths = [250, 400];
  const totalLength = lengths[0] + lengths[1];
  const innerRadius = Math.abs(lengths[0] - lengths[1]);

  const handleDragBoundary = (pos: Position) => {
    let bounded: Position = {
      x: Math.max(0, Math.min(pos.x, dimensions.x)),
      y: Math.max(0, Math.min(pos.y, dimensions.y)),
    };
    bounded = handleCircleBoundary(bounded, totalLength * 0.99, true);
    bounded = handleCircleBoundary(bounded, innerRadius * 1.01, false);
    return bounded;
  }

  const handleCircleBoundary = (pos: Position, length: number, isInside: boolean) => {
    const scale = length / Math.sqrt((pos.x - origin.x) ** 2 + (pos.y - origin.y) ** 2);
    if (scale > 1 && isInside || scale < 1 && !isInside) return pos
    const bounded = {
      x: Math.round((pos.x - origin.x) * scale + origin.x),
      y: Math.round((pos.y - origin.y) * scale + origin.y),
    }
    return bounded;
  }

  useEffect(() => {
    window.addEventListener('resize', () => {
      const dimensions = {
        x: window.innerWidth * widthPortion,
        y: window.innerHeight * heightPortion,
      };
      setDimensions(dimensions);
      setOrigin({
        x: dimensions.x / 2,
        y: 0
      });
    });
  }, []);

  useEffect(() => {
    const goalAboutOrigin = vectorDiff(jointPositions.goal, origin);
    const jointAboutOrigin = calculateJointPos(goalAboutOrigin, lengths);
    const calculatedJoint = vectorAdd(jointAboutOrigin, origin);

    // Directly update positions object
    setJointPositions(prev => ({ ...prev, joint: calculatedJoint }));

    // --- SERVO ANGLE CALCULATION ---
    const dx1 = calculatedJoint.x - origin.x;
    const dy1 = calculatedJoint.y - origin.y;
    const baseAngleRad = Math.atan2(dy1, dx1);

    const dx2 = jointPositions.goal.x - calculatedJoint.x;
    const dy2 = jointPositions.goal.y - calculatedJoint.y;
    const elbowAngleAbsRad = Math.atan2(dy2, dx2);

    let elbowAngleRelRad = elbowAngleAbsRad - baseAngleRad;
    elbowAngleRelRad = Math.atan2(Math.sin(elbowAngleRelRad), Math.cos(elbowAngleRelRad));

    setServoAngles({
      base: Math.round(baseAngleRad * (180 / Math.PI)),
      elbow: Math.round(elbowAngleRelRad * (180 / Math.PI))
    });
  }, [jointPositions.goal, origin])

  useEffect(() => {
    console.log('Servo Angles:', servoAngles);

    if (!ros || !isConnected) {
      console.log('ROS is not connected');
      return;
    }

    const commandTopic = new Topic({
      ros: ros,
      name: '/teleop',
      messageType: 'msgs/msg/Teleop',
    });

    commandTopic.publish(servoAngles);
  }, [servoAngles])


  return (
    <div className="border rounded-xl">
      <Stage width={dimensions.x} height={dimensions.y}>
        <Layer>
          <Circle
            x={dimensions.x / 2}
            y={0}
            radius={30}
            fill="#e5e5e5"
          />
          <Rect
            x={dimensions.x / 2 - 1}
            y={0}
            width={3}
            height={20}
            fill="black"
          />
          <Rect
            x={dimensions.x / 2 - 15}
            y={0}
            width={30}
            height={3}
            fill="black"
          />
          <Circle
            x={dimensions.x / 2}
            y={0}
            radius={totalLength}
            fill="#e5e5e5"
          />
          <Circle
            x={origin.x}
            y={origin.y}
            radius={innerRadius}
            fill="white"
          />
        </Layer>
        <Layer>
          <Line
            points={[dimensions.x / 2, 0, jointPositions.joint.x, jointPositions.joint.y]}
            stroke="#c2c2c2"
            strokeWidth={5}
          />
          <Line
            points={[jointPositions.joint.x, jointPositions.joint.y, jointPositions.goal.x, jointPositions.goal.y]}
            stroke="#c2c2c2"
            strokeWidth={5}
          />
        </Layer>
        <Layer>
          <Circle
            x={jointPositions.joint.x}
            y={jointPositions.joint.y}
            radius={10}
            fill="blue"
          />
          <Circle
            x={jointPositions.goal.x}
            y={jointPositions.goal.y}
            radius={15}
            fill="black"
            draggable
            onDragMove={(e) => {
              const { x: newX, y: newY } = e.target.position();
              setJointPositions(prev => ({
                ...prev,
                goal: {
                  x: newX,
                  y: newY,
                }
              }))
            }}
            dragBoundFunc={handleDragBoundary}
          />
        </Layer>
      </Stage>
    </div>
  )
}

export default Canvas
