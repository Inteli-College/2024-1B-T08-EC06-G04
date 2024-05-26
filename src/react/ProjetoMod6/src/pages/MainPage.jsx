import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import Header from '../components/Header';
import Camera from '../components/Camera';
import Controls from '../components/Controls';

const MainPage = () => {
  const [imgSrc, setImgSrc] = useState(null);
  const [connected, setConnected] = useState(false);
  const [cmdVel, setCmdVel] = useState(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
    const cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });
    const imageTopic = new ROSLIB.Topic({
      ros,
      name: '/camera/image',
      messageType: 'sensor_msgs/Image'
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
    imageTopic.subscribe(message => {
      const base64Image = `data:image/jpeg;base64,${message.data}`;
      setImgSrc(base64Image);
    });

    setCmdVel(cmdVelTopic);

    return () => {
      imageTopic.unsubscribe();
      ros.close();
    };
  }, []);

  const handleKeyDown = (e) => {
    if (!cmdVel) return;

    const twist = new ROSLIB.Message({
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 }
    });

    switch (e.key) {
      case 'w':
        twist.linear.x = 0.1;
        break;
      case 's':
        twist.linear.x = -0.1;
        break;
      case 'a':
        twist.angular.z = 0.1;
        break;
      case 'd':
        twist.angular.z = -0.1;
        break;
      case ' ':
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        break;
      default:
        break;
    }

    cmdVel.publish(twist);
  };

  const handleButtonClick = (direction) => {
    if (!cmdVel) return;

    const twist = new ROSLIB.Message({
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 }
    });

    switch (direction) {
      case 'up':
        twist.linear.x = 0.1;
        break;
      case 'down':
        twist.linear.x = -0.1;
        break;
      case 'left':
        twist.angular.z = 0.1;
        break;
      case 'right':
        twist.angular.z = -0.1;
        break;
      case 'stop':
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        break;
      default:
        break;
    }

    cmdVel.publish(twist);
  };

  return (
    <div className="App flex flex-col justify-center min-h-screen bg-white w-full" onKeyDown={e => handleKeyDown(e, cmdVel)} tabIndex="0">
      <div className="grid grid-cols-8 gap-4 w-full p-4">
        <div className="col-start-2 col-span-1">
          <Camera imgSrc={imgSrc} />
        </div>
        <div className="col-start-4 col-span-1 mt-[-2rem]"> {/* Moves the Header up */}
          <Header connected={connected} />
        </div>
        <div className="col-start-5 col-span-1 mt-20"> {/* Moves the Controls further down */}
          <Controls handleButtonClick={direction => handleButtonClick(direction, cmdVel)} />
        </div>
      </div>
    </div>
  );
};

export default MainPage;
