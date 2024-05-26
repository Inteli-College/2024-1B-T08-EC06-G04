import React from 'react';

const Camera = ({ imgSrc }) => {
  return (
    <div className="camera w-80 h-96 bg-gray-300 border-4 border-gray-500 rounded-lg flex items-center justify-center overflow-hidden">
      {imgSrc ? (
        <img src={imgSrc} alt="Video Stream" className="max-w-full max-h-full" />
      ) : (
        <div className="camera-placeholder text-gray-700">Câmera não conectada</div>
      )}
    </div>
  );
};

export default Camera;
