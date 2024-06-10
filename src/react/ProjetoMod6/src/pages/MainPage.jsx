import React, { useEffect, useState, useRef } from "react";
import ROSLIB from "roslib";
import Header from "../components/Header";
import Camera from "../components/Camera";
import Controls from "../components/Controls";
import PhotoButton from "../components/PhotoButton";
import KillSwitch from "../components/KillSwitch";
import Popup from "../components/Popup";
import WarningPopup from "../components/WarningPopup";

// Página principal
const MainPage = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [processedImage, setProcessedImage] = useState(null);
  const [warningMessage, setWarningMessage] = useState("");
  const latestFrame = useRef(null);

  // Conecta no Websocket do robô utilizando a biblioteca ROSLIB
  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url: "ws://10.128.0.9:9090" });

    rosInstance.on("connection", () => {
      console.log("Connected to rosbridge websocket server.");
      setConnected(true);
    });

    rosInstance.on("error", (error) => {
      console.error("Error connecting to websocket server:", error);
      setConnected(false);
    });

    rosInstance.on("close", () => {
      console.log("Connection to websocket server closed.");
      setConnected(false);
    });

    setRos(rosInstance);

    return () => {
      rosInstance.close();
    };
  }, []);

  // Função pega a imagem pós clicar no botão de tirar foto e envia ao endpoint para realizar as alterações na mesma
  const handleTakePhoto = () => {
    if (latestFrame.current) {
      const base64image = latestFrame.current;

      // Endpoint do processamento da imagem
      fetch("http://127.0.0.1:8000/api/image_processing/process_image", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ image: base64image }),
      })
        .then((response) => response.json())
        .then((data) => {
          // Imagem processada
          setProcessedImage(data.processed_image);
        })
        .catch((error) => console.error("Error processing image:", error));
    } else {
      console.log("No image available.");
    }
  };

  const closePopup = () => {
    setProcessedImage(null);
  };

  const handleWarning = (message) => {
    setWarningMessage(message);
  };

  return (
    <div
      className="App flex flex-col justify-center min-h-screen bg-white w-full"
      tabIndex="0"
    >
      <Header connected={connected} />
      <div className="relative flex flex-grow">
        <Camera
          ros={ros}
          onUpdateFrame={(frame) => (latestFrame.current = frame)}
        />
        <div className="absolute bottom-5 left-5">
          <Controls ros={ros} onWarning={handleWarning} />
        </div>
        <div className="flex flex-col gap-10 absolute bottom-10 right-10">
          <KillSwitch ros={ros} />
          <PhotoButton onClick={handleTakePhoto} />
        </div>
      </div>
      {processedImage && <Popup image={processedImage} onClose={closePopup} />}
      {warningMessage && <WarningPopup message={warningMessage} />}
    </div>
  );
};

export default MainPage;
