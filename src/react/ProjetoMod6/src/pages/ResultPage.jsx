import React from 'react';
import { useLocation } from 'react-router-dom';

const ResultPage = () => {
  const location = useLocation();
  const { processedImage } = location.state || {};

  if (!processedImage) {
    return <div>No image data available.</div>;
  }

  return (
    <div className="ResultPage">
      <h1>Processed Image</h1>
      <img src={`data:image/jpeg;base64,${processedImage}`} alt="Processed" />
    </div>
  );
};

export default ResultPage;
