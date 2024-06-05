import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import Header from '../components/Header';
import Camera from '../components/Camera';
import Controls from '../components/Controls';

const MainPage = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: 'ws://10.128.0.9:9090' });

    rosInstance.on('connection', () => {
      console.log('Connected to rosbridge websocket server.');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to websocket server:', error);
      setConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Connection to websocket server closed.');
      setConnected(false);
    });

    setRos(rosInstance);

    return () => {
      rosInstance.close();
    };
  }, []);

  return (
    <div className="App flex flex-col justify-center min-h-screen bg-white w-full" tabIndex="0">
      <Header connected={connected} />
      <div className="relative flex flex-grow">
        <Camera ros={ros} />
        <div className="absolute bottom-5 left-5">
          <Controls ros={ros} />
        </div>
      </div>
    </div>
  );
};

export default MainPage;
