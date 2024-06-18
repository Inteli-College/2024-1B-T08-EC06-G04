import React, { useState, useEffect } from 'react';
import ImageTable from '../components/ResultTable';
import ImagePopup from '../components/ImagePopup';
import Button from '../components/Button'

const ResultPage = () => {
  const [rows, setRows] = useState([]);
  const [showPopup, setShowPopup] = useState(false);
  const [currentImage, setCurrentImage] = useState('');

  const openPopup = (image) => {
    setCurrentImage(image);
    setShowPopup(true);
  };

  const closePopup = () => {
    setShowPopup(false);
    setCurrentImage('');
  };

  const GetInformations = async () => {
    try {
      const response = await fetch("http://127.0.0.1:8000/api/crud/read", {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
        },
      });
      const data = await response.json();
      if (data.error) {
        console.error("Error retrieving data:", data.error);
      } else {
        setRows(data);
      }
    } catch (error) {
      console.error("Error retrieving data:", error);
    }
  };

  useEffect(() => {
    GetInformations();
  }, []);

  return (
    <div className='flex flex-col items-center justify-center gap-6 m-8'>
      <div className="absolute top-5 left-5 z-10">
      <Button label="Voltar" url="http://localhost:5173/"/> 
      </div>
      <h1 className='text-xl font-bold'>Lista de Imagens analisadas</h1>
      <ImageTable rows={rows} openPopup={openPopup} />
      {showPopup && <ImagePopup currentImage={currentImage} closePopup={closePopup} />}
    </div>
  );
};

export default ResultPage;
