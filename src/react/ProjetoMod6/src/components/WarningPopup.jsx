import React from "react";

// PopUp de aviso ao usuário se o robô vai bater na frente ou atrás
const WarningPopup = ({ message }) => {
  if (!message) return null;

  return (
    <div className="fixed top-5 right-5 bg-red-500 text-white p-3 rounded shadow-lg z-50">
      <p>{message}</p>
    </div>
  );
};

export default WarningPopup;
