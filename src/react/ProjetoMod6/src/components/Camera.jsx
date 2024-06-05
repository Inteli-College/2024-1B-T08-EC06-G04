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
