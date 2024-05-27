import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import Header from '../components/Header';
import Camera from '../components/Camera';
import Controls from '../components/Controls';

const MainPage = () => {
  const [imgSrc, setImgSrc] = useState(null);
  const [connected, setConnected] = useState(false);
  const [cmdVel, setCmdVel] = useState(null);
  const [frontClear, setFrontClear] = useState(true);
  const [backClear, setBackClear] = useState(true);
  const [pressedKeys, setPressedKeys] = useState({});

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://10.128.0.9:9090' });
    const cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
    const videoTopic = new ROSLIB.Topic({
      ros,
      name: '/chatter',
      messageType: 'std_msgs/String'
    });
    const lidarTopic = new ROSLIB.Topic({
      ros,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan'
    });

    ros.on('connection', () => {
      console.log('Connected to rosbridge websocket server.');
      setConnected(true);
    });
    ros.on('error', error => {
      console.error('Error connecting to websocket server:', error);
      setConnected(false);
    });
    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
      setConnected(false);
    });

    videoTopic.subscribe(message => {
      try {
        const base64Image = `data:image/jpeg;base64,${message.data.replace(/\s/g, '')}`;
        setImgSrc(base64Image);
      } catch (error) {
        console.error('Error processing image message:', error);
      }
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
    };
  }, []);

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

    // If front or back is not clear, stop the robot
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
    <div className="App flex flex-col justify-center min-h-screen bg-white w-full" tabIndex="0">
      <div className="grid grid-cols-8 gap-4 w-full p-4">
        <div className="col-start-2 col-span-1">
          <Camera imgSrc={imgSrc} />
        </div>
        <div className="col-start-4 col-span-1 mt-[-2rem]">
          <Header connected={connected} />
        </div>
        <div className="col-start-5 col-span-1 mt-20">
          <Controls handleButtonClick={(direction) => setPressedKeys({ [direction]: true })} />
        </div>
      </div>
    </div>
  );
};

export default MainPage;
