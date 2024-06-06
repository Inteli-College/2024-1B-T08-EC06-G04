// Em MainPage.js

import React, { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import Header from '../components/Header';
import Camera from '../components/Camera';
import Controls from '../components/Controls';
import PhotoButton from '../components/PhotoButton';

const MainPage = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const latestFrame = useRef(null);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

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

  const handleTakePhoto = () => {
    if (latestFrame.current) {
      const base64image = latestFrame.current;
      console.log('Base64 Image:', base64image);

      fetch('http://localhost:8000/process_image', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ image: base64image })
      })
      .then(response => response.json())
      .then(data => {
        history.push('/result', { processedImage: data });
      })
      .catch(error => console.error('Error processing image:', error));
      
    } else {
      console.log('No image available.');
    }
  };

  return (
    <div className="App flex flex-col justify-center min-h-screen bg-white w-full" tabIndex="0">
      <Header connected={connected} />
      <div className="relative flex flex-grow">
        <Camera ros={ros} onUpdateFrame={(frame) => (latestFrame.current = frame)} />
        <div className="absolute bottom-5 left-5">
          <Controls ros={ros} />
        </div>
        <div className="absolute bottom-10 right-10">
          <PhotoButton onClick={handleTakePhoto} />
        </div>
      </div>
    </div>
  );
};

export default MainPage;
