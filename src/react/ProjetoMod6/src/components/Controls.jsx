import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";

// Função principal dos controles, se inscreve nos tópicos /cmd_vel e /scan, da velocidade e LiDAR respectivamente
const Controls = ({ ros, onWarning }) => {
  const [cmdVel, setCmdVel] = useState(null);
  const [frontClear, setFrontClear] = useState(true);
  const [backClear, setBackClear] = useState(true);
  const [pressedKeys, setPressedKeys] = useState({});

  useEffect(() => {
    if (!ros) return;

    // Tópico de velocidade do robô
    const cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
    // Tópico do LiDAR
    const lidarTopic = new ROSLIB.Topic({
      ros,
      name: "/scan",
      messageType: "sensor_msgs/LaserScan",
    });

    lidarTopic.subscribe((message) => {
      lidarCallback(message.ranges);
    });

    setCmdVel(cmdVelTopic);

    // Funções relativas a apertar e soltar os comandos
    const handleKeyDown = (e) => {
      setPressedKeys((prevKeys) => ({ ...prevKeys, [e.key]: true }));
    };

    const handleKeyUp = (e) => {
      setPressedKeys((prevKeys) => ({ ...prevKeys, [e.key]: false }));
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      lidarTopic.unsubscribe();
    };
  }, [ros]);

  useEffect(() => {
    const interval = setInterval(() => {
      updateMovement();
    }, 10);

    return () => {
      clearInterval(interval);
    };
  }, [frontClear, backClear, pressedKeys]);

  // Função para calcular a distância da frente e trás do objeto mais próximo ao robô
  const lidarCallback = (ranges) => {
    const numRanges = ranges.length;
    const sectorSize = Math.floor(numRanges / 12);
    // Distância para o LiDAR travar o robô
    const safetyDistance = 0.35;

    const frontLeftIndices = Array.from(
      { length: sectorSize },
      (_, i) => numRanges - sectorSize + i,
    );
    const frontRightIndices = Array.from({ length: sectorSize }, (_, i) => i);
    const backIndices = Array.from(
      { length: sectorSize * 2 },
      (_, i) => 5 * sectorSize + i,
    );

    const frontRanges = frontLeftIndices
      .concat(frontRightIndices)
      .map((index) => ranges[index])
      .filter((range) => range > 0.01 && range < 100.0);
    const backRanges = backIndices
      .map((index) => ranges[index])
      .filter((range) => range > 0.01 && range < 100.0);

    // Constantes se a frente ou trás está com objeto
    const frontIsClear = !frontRanges.some((range) => range < safetyDistance);
    const backIsClear = !backRanges.some((range) => range < safetyDistance);

    if (!frontIsClear) {
      onWarning(
        "Está muito próximo da frente. Movimento na direção bloqueado.",
      );
    } else if (!backIsClear) {
      onWarning(
        "Está muito próximo da parte de trás. Movimento na direção bloqueado.",
      );
    } else {
      onWarning("");
    }

    setFrontClear(frontIsClear);
    setBackClear(backIsClear);
  };

  // Função para atualizar o movimento do robô em relação ao botão/tecla apertado
  const updateMovement = () => {
    let linear = 0.0;
    let angular = 0.0;

    if (frontClear && pressedKeys["w"]) {
      linear = 0.2;
    } else if (backClear && pressedKeys["s"]) {
      linear = -0.2;
    }

    if (pressedKeys["a"]) {
      angular = 0.5;
    } else if (pressedKeys["d"]) {
      angular = -0.5;
    }

    if (!frontClear && linear > 0) {
      linear = 0.0;
    }
    if (!backClear && linear < 0) {
      linear = 0.0;
    }

    moveRobot(linear, angular);
  };

  // Função para enviar as informações ao tópico
  const moveRobot = (linear, angular) => {
    if (!cmdVel) return;

    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angular },
    });

    cmdVel.publish(twist);
  };

  return (
    <div className="controls flex flex-col items-center space-y-2 ml-10">
      <button
        onMouseDown={() => setPressedKeys({ ...pressedKeys, w: true })}
        onMouseUp={() => setPressedKeys({ ...pressedKeys, w: false })}
        className="w-[90px] h-[90px] relative bg-green-400 rounded-lg border-2 border-neutral-500 hover:bg-green-700 shadow-md hover:shadow-lg transition-all duration-300 ease-in-out"
      >
        <div className="rotate-180 h-full w-full flex items-center justify-center">
          <svg
            className="w-8 h-8 text-black"
            fill="currentColor"
            viewBox="0 0 20 20"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              fillRule="evenodd"
              d="M10 18a1 1 0 01-.707-.293l-7-7a1 1 0 011.414-1.414L9 14.586V3a1 1 0 112 0v11.586l5.293-5.293a1 1 0 011.414 1.414l-7 7A1 1 0 0110 18z"
              clipRule="evenodd"
            />
          </svg>
        </div>
      </button>
      <div className="flex space-x-2">
        <button
          onMouseDown={() => setPressedKeys({ ...pressedKeys, a: true })}
          onMouseUp={() => setPressedKeys({ ...pressedKeys, a: false })}
          className="w-[90px] h-[90px] relative bg-green-400 rounded-lg border-2 border-neutral-500 hover:bg-green-700 shadow-md hover:shadow-lg transition-all duration-300 ease-in-out"
        >
          <div className="rotate-90 h-full w-full flex items-center justify-center">
            <svg
              className="w-8 h-8 text-black"
              fill="currentColor"
              viewBox="0 0 20 20"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                fillRule="evenodd"
                d="M10 18a1 1 0 01-.707-.293l-7-7a1 1 0 011.414-1.414L9 14.586V3a1 1 0 112 0v11.586l5.293-5.293a1 1 0 011.414 1.414l-7 7A1 1 0 0110 18z"
                clipRule="evenodd"
              />
            </svg>
          </div>
        </button>
        <button
          onMouseDown={() => setPressedKeys({})}
          className="w-[90px] h-[90px] relative bg-red-500 rounded-lg border-2 border-neutral-700 text-white flex items-center justify-center hover:bg-red-700 shadow-md hover:shadow-lg transition-all duration-300 ease-in-out"
        >
          Parar
        </button>
        <button
          onMouseDown={() => setPressedKeys({ ...pressedKeys, d: true })}
          onMouseUp={() => setPressedKeys({ ...pressedKeys, d: false })}
          className="w-[90px] h-[90px] relative bg-green-400 rounded-lg border-2 border-neutral-500 hover:bg-green-700 shadow-md hover:shadow-lg transition-all duration-300 ease-in-out"
        >
          <div className="rotate-[270deg] h-full w-full flex items-center justify-center">
            <svg
              className="w-8 h-8 text-black"
              fill="currentColor"
              viewBox="0 0 20 20"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                fillRule="evenodd"
                d="M10 18a1 1 0 01-.707-.293l-7-7a1 1 0 011.414-1.414L9 14.586V3a1 1 0 112 0v11.586l5.293-5.293a1 1 0 011.414 1.414l-7 7A1 1 0 0110 18z"
                clipRule="evenodd"
              />
            </svg>
          </div>
        </button>
      </div>
      <button
        onMouseDown={() => setPressedKeys({ ...pressedKeys, s: true })}
        onMouseUp={() => setPressedKeys({ ...pressedKeys, s: false })}
        className="w-[90px] h-[90px] relative bg-green-400 rounded-lg border-2 border-neutral-500 hover:bg-green-700 shadow-md hover:shadow-lg transition-all duration-300 ease-in-out"
      >
        <div className="h-full w-full flex items-center justify-center">
          <svg
            className="w-8 h-8 text-black"
            fill="currentColor"
            viewBox="0 0 20 20"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              fillRule="evenodd"
              d="M10 18a1 1 0 01-.707-.293l-7-7a1 1 0 011.414-1.414L9 14.586V3a1 1 0 112 0v11.586l5.293-5.293a1 1 0 011.414 1.414l-7 7A1 1 0 0110 18z"
              clipRule="evenodd"
            />
          </svg>
        </div>
      </button>
    </div>
  );
};

export default Controls;
