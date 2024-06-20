import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

// Componente Camera, pede um objeto ros e uma função onUpdateFrame como parâmetros
const Camera = ({ ros, onUpdateFrame }) => {
  const [frames, setFrames] = useState([]);
  const [timestamp, setTimestamp] = useState(null);
  const [latency, setLatency] = useState(null);

  useEffect(() => {
    // Se ROS não estiver conectado não faça nada
    if (!ros) return;

    // Cria um novo tópico no /chatter
    const videoTopic = new ROSLIB.Topic({
      ros,
      name: '/chatter',
      messageType: 'std_msgs/String'
    });

    const handleMessage = (message) => {
      try {
        // Divide a mensagem entre o timestamp e a base64 (são divididas usando o caractere '|')
        const [timestamp, base64Image] = message.data.split('|');
        const imgSrc = `data:image/jpeg;base64,${base64Image}`;
        const messageTimestamp = parseFloat(timestamp) * 1000; // Converte para milisegundos
        const currentTimestamp = Date.now();
        const latency = currentTimestamp - messageTimestamp;

        setFrames((prevFrames) => {
          // Adiciona a nova imagem ao array de frames
          const newFrames = [...prevFrames, imgSrc];
          if (newFrames.length > 10) {
            newFrames.shift(); // Remove a imagem mais antiga
          }
          return newFrames;
        });
        // Atualiza o timestamp e a latência
        setTimestamp(new Date(messageTimestamp).toLocaleString());
        setLatency(latency);
        if (onUpdateFrame) {
          onUpdateFrame(base64Image); // Passe o base64Image para a função onUpdateFrame
        }
      } catch (error) {
        console.error('Error processing image message:', error);
      }
    };

    videoTopic.subscribe(handleMessage);

    return () => {
      videoTopic.unsubscribe(handleMessage);
    };
  }, [ros, onUpdateFrame]);

  return (
    <div className="camera flex-grow h-screen bg-gray-300 border-4 border-gray-500 rounded-lg flex items-center justify-center overflow-hidden">
      <img id="videoStream" src={frames[frames.length - 1]} alt="Video Stream" className="w-full h-full object-cover " />

      <div className="absolute top-3 left-3 text-white bg-black bg-opacity-50 p-1 rounded">
        {timestamp && <div className="timestamp text-sm">{`Timestamp: ${timestamp}`}</div>}
        {latency !== null && <div className="latency text-sm">{`Latency: ${latency.toFixed(2)} ms`}</div>}
      </div>
    </div>
  );
};

export default Camera;
