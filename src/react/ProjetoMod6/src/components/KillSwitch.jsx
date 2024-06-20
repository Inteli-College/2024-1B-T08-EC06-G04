import React from "react";
import ROSLIB from "roslib";

const KillSwitch = ({ ros }) => {
  const emergencyStop = () => {
    // Se houver conexão ROS
    if (ros) {
      // Cria um serviço no nome /emergency_stop
      const emergencyStopService = new ROSLIB.Service({
        ros: ros,
        name: "/emergency_stop",
        serviceType: "std_srvs/srv/Empty",
      });

      // Faz o corpo do request
      const request = new ROSLIB.ServiceRequest({});

      // Chama o serviço
      emergencyStopService.callService(request, (result) => {
        console.log("Emergency stop service called.", result);
      });
    }
  };

  // Fecha a conexão ROS 
  const closeConnection = () => {
    if (ros) {
      ros.close();
      console.log('ROS connection closed.');
    }
  };

  return (
    <button
      className="bg-green-300 hover:bg-green-500 text-white font-bold py-10 px-10 rounded-full border-2 border-neutral-500 hover:bg-green-400"
      onClick={() => {
        emergencyStop();
        closeConnection();
      }}
    >
      <img src="/images/emergency.svg" alt="Parar" className="w-20 h-20" />
    </button>
  );
};

export default KillSwitch;