import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";

// Constante relacionada a câmera, se inscreve no tópico /chatter que envia dados da câmera
const Camera = ({ ros, onUpdateFrame }) => {
  const [frames, setFrames] = useState([]);
  const [timestamp, setTimestamp] = useState(null);
  const [latency, setLatency] = useState(null);

  useEffect(() => {
    if (!ros) return;

    const videoTopic = new ROSLIB.Topic({
      ros,
      name: "/chatter",
      messageType: "std_msgs/String",
    });

    // Função para processar as imagens recebidas e atualizar o src da img
    const handleMessage = (message) => {
      try {
        const [timestamp, base64Image] = message.data.split("|");
        const imgSrc = `data:image/jpeg;base64,${base64Image}`;
        const messageTimestamp = parseFloat(timestamp) * 1000; // Converte em milisegundos
        const currentTimestamp = Date.now();
        const latency = currentTimestamp - messageTimestamp;

        setFrames((prevFrames) => {
          const newFrames = [...prevFrames, imgSrc];
          if (newFrames.length > 10) {
            newFrames.shift(); // Remove o ultimo frame da imagem
          }
          return newFrames;
        });
        setTimestamp(new Date(messageTimestamp).toLocaleString());
        setLatency(latency);
        if (onUpdateFrame) {
          onUpdateFrame(base64Image); // Passe o base64Image para a função onUpdateFrame
        }
      } catch (error) {
        console.error("Error processing image message:", error);
      }
    };

    videoTopic.subscribe(handleMessage);

    return () => {
      videoTopic.unsubscribe(handleMessage);
    };
  }, [ros, onUpdateFrame]);

  return (
    <div className="camera flex-grow h-screen bg-gray-300 border-4 border-gray-500 rounded-lg flex items-center justify-center overflow-hidden">
      <img
        id="videoStream"
        src={frames[frames.length - 1]}
        alt="Video Stream"
        className="w-full h-full object-cover "
      />

      <div className="absolute top-3 left-3 text-white bg-black bg-opacity-50 p-1 rounded">
        {timestamp && (
          <div className="timestamp text-sm">{`Timestamp: ${timestamp}`}</div>
        )}
        {latency !== null && (
          <div className="latency text-sm">{`Latency: ${latency.toFixed(2)} ms`}</div>
        )}
      </div>
    </div>
  );
};

export default Camera;
