import React, { useRef, useEffect } from 'react';

const Camera = ({ imgSrc }) => {
  const imgRef = useRef(null);

  useEffect(() => {
    if (imgRef.current && imgSrc) {
      imgRef.current.src = imgSrc;
    }
  }, [imgSrc]);

  return (
    <div className="camera w-80 h-96 bg-gray-300 border-4 border-gray-500 rounded-lg flex items-center justify-center overflow-hidden">
      <img ref={imgRef} alt="Video Stream" className="max-w-full max-h-full object-cover" />
    </div>
  );
};

export default Camera;
