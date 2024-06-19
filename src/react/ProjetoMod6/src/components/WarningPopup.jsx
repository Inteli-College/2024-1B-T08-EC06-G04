import React from 'react';

// Componente que exibe uma notificação popup, pede uma mensagem
const WarningPopup = ({ message }) => {
  if (!message) return null;

  return (
    <div className="fixed top-5 right-5 bg-red-500 text-white p-3 rounded shadow-lg z-50">
      <p>{message}</p>
    </div>
  );
};

export default WarningPopup;
