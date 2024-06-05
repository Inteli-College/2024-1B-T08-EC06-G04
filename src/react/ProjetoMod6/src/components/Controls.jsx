import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const Controls = ({ ros }) => {
  const [cmdVel, setCmdVel] = useState(null);
  const [frontClear, setFrontClear] = useState(true);
  const [backClear, setBackClear] = useState(true);
  const [pressedKeys, setPressedKeys] = useState({});

  useEffect(() => {
    if (!ros) return;

    const cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
    const lidarTopic = new ROSLIB.Topic({
      ros,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan'
    });

    lidarTopic.subscribe(message => {
      lidarCallback(message.ranges);
    });

    setCmdVel(cmdVelTopic);

    const handleKeyDown = (e) => {
      setPressedKeys(prevKeys => ({ ...prevKeys, [e.key]: true }));
    };

    const handleKeyUp = (e) => {
      setPressedKeys(prevKeys => ({ ...prevKeys, [e.key]: false }));
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      lidarTopic.unsubscribe();
    };
  }, [ros]);

  useEffect(() => {
    const interval = setInterval(() => {
      updateMovement();
    }, 10);

    return () => {
      clearInterval(interval);
    };
  }, [frontClear, backClear, pressedKeys]);

  const lidarCallback = (ranges) => {
    const numRanges = ranges.length;
    const sectorSize = Math.floor(numRanges / 12);
    const safetyDistance = 0.35;

    const frontLeftIndices = Array.from({ length: sectorSize }, (_, i) => numRanges - sectorSize + i);
    const frontRightIndices = Array.from({ length: sectorSize }, (_, i) => i);
    const backIndices = Array.from({ length: sectorSize * 2 }, (_, i) => 5 * sectorSize + i);

    const frontRanges = frontLeftIndices.concat(frontRightIndices).map(index => ranges[index]).filter(range => range > 0.01 && range < 100.0);
    const backRanges = backIndices.map(index => ranges[index]).filter(range => range > 0.01 && range < 100.0);

    const frontIsClear = !frontRanges.some(range => range < safetyDistance);
    const backIsClear = !backRanges.some(range => range < safetyDistance);

    setFrontClear(frontIsClear);
    setBackClear(backIsClear);
  };

  const updateMovement = () => {
    let linear = 0.0;
    let angular = 0.0;

    if (frontClear && pressedKeys['w']) {
      linear = 0.2;
    } else if (backClear && pressedKeys['s']) {
      linear = -0.2;
    }

    if (pressedKeys['a']) {
      angular = 0.5;
    } else if (pressedKeys['d']) {
      angular = -0.5;
    }

    if (!frontClear && linear > 0) {
      linear = 0.0;
    }
    if (!backClear && linear < 0) {
      linear = 0.0;
    }

    moveRobot(linear, angular);
  };

  const moveRobot = (linear, angular) => {
    if (!cmdVel) return;

    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angular }
    });

    cmdVel.publish(twist);
  };

  return (
    <div className="controls flex flex-col items-center space-y-2 ml-10">
      <button
        onMouseDown={() => setPressedKeys({ ...pressedKeys, 'w': true })}
        onMouseUp={() => setPressedKeys({ ...pressedKeys, 'w': false })}
        className="w-[100px] h-[100px] relative bg-green-300 rounded-[20px] border-2 border-neutral-500 hover:bg-green-400"
      >
        <div className="rotate-[180deg] h-20 w-20 flex items-center justify-center">
          <img src="/images/arrow.svg" alt="UP" className='fl'/>
        </div>
      </button>
      <div className="flex space-x-2">
        <button
          onMouseDown={() => setPressedKeys({ ...pressedKeys, 'a': true })}
          onMouseUp={() => setPressedKeys({ ...pressedKeys, 'a': false })}
          className="w-[100px] h-[100px] relative bg-green-300 rounded-[20px] border-2 border-neutral-500 hover:bg-green-400"
        >
          <div className="rotate-[90deg] h-20 w-20 flex items-center justify-center">
          <img src="/images/arrow.svg" alt="UP" className='fl'/>
        </div>
        </button>
        <button
          onMouseDown={() => setPressedKeys({})}
          className="w-[100px] h-[100px] relative bg-red-500 rounded-[20px] border-2 border-neutral-500 text-white flex items-center justify-center hover:bg-red-600"
        >
          Parar
        </button>
        <button
          onMouseDown={() => setPressedKeys({ ...pressedKeys, 'd': true })}
          onMouseUp={() => setPressedKeys({ ...pressedKeys, 'd': false })}
          className="w-[100px] h-[100px] relative bg-green-300 rounded-[20px] border-2 border-neutral-500 hover:bg-green-400"
        >
          <div className="rotate-[270deg] h-20 w-20 flex items-center justify-center">
          <img src="/images/arrow.svg" alt="UP" className='fl'/>
        </div>
        </button>
      </div>
      <button
        onMouseDown={() => setPressedKeys({ ...pressedKeys, 's': true })}
        onMouseUp={() => setPressedKeys({ ...pressedKeys, 's': false })}
        className="w-[100px] h-[100px] relative bg-green-300 rounded-[20px] border-2 border-neutral-500 hover:bg-green-400"
      >
        <div className="h-20 w-20 flex items-center justify-center">
          <img src="/images/arrow.svg" alt="UP" className='fl'/>
        </div>
      </button>
    </div>
  );
};

export default Controls;