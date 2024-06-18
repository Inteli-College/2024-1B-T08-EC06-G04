import React from 'react';

const ImagePopup = ({ currentImage, closePopup }) => {
  return (
    <div className="fixed inset-0 flex items-center justify-center bg-gray-800 bg-opacity-50">
      <div className="bg-white inline-flex flex-col items-center p-4 gap-4 rounded shadow-lg max-w-sm">
        <h2 className='text-xl font-bold'>Imagem Analisada</h2>
        <img src={`data:image/png;base64,${currentImage}`} alt="Processed" className="w-[400px] h-[400px] bg-slate-500" />
        <button
          className="px-4 py-2 bg-red-500 hover:bg-red-700 text-white rounded"
          onClick={closePopup}
        >
          Fechar
        </button>
      </div>
    </div>
  );
};

export default ImagePopup;
