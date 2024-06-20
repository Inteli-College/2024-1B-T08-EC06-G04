import React from 'react';

// Componente para exibir uma imagem em um popup, pede a base64 de uma imagem e uma função onClose
const Popup = ({ image, onClose }) => {
  if (!image) return null;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-75 flex items-center justify-center z-50">
      <div className="bg-white p-5 rounded-lg shadow-lg">
        <img src={`data:image/jpeg;base64,${image}`} alt="Processed" className="max-w-full max-h-full" />
        <button className="mt-3 bg-red-500 text-white py-2 px-4 rounded" onClick={onClose}>Fechar</button>
      </div>
    </div>
  );
};

export default Popup;
