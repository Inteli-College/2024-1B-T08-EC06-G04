import React from 'react';
import ROSLIB from 'roslib';

const KillSwitch = ({ ros }) => {

  const closeConnection = () => {
    if (ros) {
      ros.close();
      console.log('ROS connection closed.');
    }
  };

  return (
    <button className="bg-green-300 hover:bg-green-500 text-white font-bold py-10 px-10 rounded-full border-2 border-neutral-500 hover:bg-green-400" onClick={closeConnection}>
      <img src="/images/emergency.svg" alt="Parar" className="w-20 h-20" />
    </button>
  );
};

export default KillSwitch;
