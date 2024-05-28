import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const Camera = ({ ros }) => {
  const [frames, setFrames] = useState([]);
  const [timestamp, setTimestamp] = useState(null);
  const [latency, setLatency] = useState(null);

  useEffect(() => {
    if (!ros) return;

    const videoTopic = new ROSLIB.Topic({
      ros,
      name: '/chatter',
      messageType: 'std_msgs/String'
    });

    const handleMessage = (message) => {
      try {
        const [timestamp, base64Image] = message.data.split('|');
        const imgSrc = `data:image/jpeg;base64,${base64Image}`;
        const messageTimestamp = parseFloat(timestamp) * 1000; // Convert to milliseconds
        const currentTimestamp = Date.now();
        const latency = currentTimestamp - messageTimestamp;

        setFrames((prevFrames) => {
          const newFrames = [...prevFrames, imgSrc];
          if (newFrames.length > 10) {
            newFrames.shift(); // Remove the oldest frame
          }
          return newFrames;
        });
        setTimestamp(new Date(messageTimestamp).toLocaleString());
        setLatency(latency);
      } catch (error) {
        console.error('Error processing image message:', error);
      }
    };

    videoTopic.subscribe(handleMessage);

    return () => {
      videoTopic.unsubscribe(handleMessage);
    };
  }, [ros]);

  return (
    <div className="camera w-80 h-96 bg-gray-300 border-4 border-gray-500 rounded-lg flex flex-col items-center justify-center overflow-hidden">
      <img id="videoStream" src={frames[frames.length - 1]} alt="Video Stream" className="max-w-full max-h-full object-cover" />
      {timestamp && <div className="timestamp mt-2 text-sm text-gray-700">{`Timestamp: ${timestamp}`}</div>}
      {latency !== null && <div className="latency mt-1 text-sm text-gray-700">{`Latency: ${latency.toFixed(2)} ms`}</div>}
    </div>
  );
};

export default Camera;
