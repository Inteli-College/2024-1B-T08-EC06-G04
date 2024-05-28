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
      <div className="grid grid-cols-8 gap-4 w-full p-4">
        <div className="col-start-2 col-span-1">
          <Camera ros={ros} />
        </div>
        <div className="col-start-4 col-span-1 mt-[-2rem]">
          <Header connected={connected} />
        </div>
        <div className="col-start-5 col-span-1 mt-20">
          <Controls ros={ros} />
        </div>
      </div>
    </div>
  );
};

export default MainPage;
