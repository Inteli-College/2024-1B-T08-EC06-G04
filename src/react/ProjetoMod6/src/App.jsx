// App.js
import React, { useEffect, useState } from 'react';
import './App.css';

const App = () => {
  const [imgSrc, setImgSrc] = useState(null);
  const [controlSocket, setControlSocket] = useState(null);
  const [imageSocket, setImageSocket] = useState(null);

  useEffect(() => {
    const imgSocket = new WebSocket('ws://localhost:8765');
    setImageSocket(imgSocket);

    imgSocket.onmessage = (event) => {
      const [timestamp, jpg_as_text] = event.data.split('|');
      const img = `data:image/jpeg;base64,${jpg_as_text}`;
      setImgSrc(img);
    };

    const ctrlSocket = new WebSocket('ws://localhost:8766');
    setControlSocket(ctrlSocket);

    return () => {
      imgSocket.close();
      ctrlSocket.close();
    };
  }, []);

  const handleKeyDown = (e) => {
    if (!controlSocket) return;

    switch (e.key) {
      case 'w':
        controlSocket.send(JSON.stringify({ action: 'increase_linear_speed' }));
        break;
      case 's':
        controlSocket.send(JSON.stringify({ action: 'decrease_linear_speed' }));
        break;
      case 'a':
        controlSocket.send(JSON.stringify({ action: 'increase_angular_speed' }));
        break;
      case 'd':
        controlSocket.send(JSON.stringify({ action: 'decrease_angular_speed' }));
        break;
      case ' ':
        controlSocket.send(JSON.stringify({ action: 'stop_robot' }));
        break;
      default:
        break;
    }
  };

  return (
    <div className="App" onKeyDown={handleKeyDown} tabIndex="0">
      <header className="App-header">
        <h1>Teleoperação do Robô</h1>
        {imgSrc && <img src={imgSrc} alt="Video Stream" />}
      </header>
    </div>
  );
};

export default App;
