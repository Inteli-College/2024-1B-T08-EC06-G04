import React, { useState, useEffect } from 'react';
import { useLocation } from 'react-router-dom';

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
        console.log(data)
        setRows(data);
      }
    } catch (error) {
      console.error("Error retrieving data:", error);
    }
  };

  useEffect(() => {
    // Chamada inicial para buscar os dados da tabela
    GetInformations();
  }, []);

  return (
    <div className='flex flex-col items-center justify-center gap-6 m-8'>
      <h1 className='text-xl font-bold'>Lista de Imagens analisadas</h1>
      <table>
        <thead>
          <tr>
            <th className="px-4 py-2 border border-gray-500">Id</th>
            <th className="px-4 py-2 border border-gray-500">Version</th>
            <th className="px-4 py-2 border border-gray-500">Status</th>
            <th className="px-4 py-2 border border-gray-500">Image</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((row) => (
            <tr key={row.id}>
              <td className="px-4 py-2 border border-gray-300">{row.id}</td>
              <td className="px-4 py-2 border border-gray-300">{row.version}</td>
              <td className="px-4 py-2 border border-gray-300">{row.result}</td>
              <td className="px-4 py-2 border border-gray-300">
                <button
                  className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded"
                  onClick={() => openPopup(row.image)}
                >
                  Abrir Imagem
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {showPopup && (
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
      )}
    </div>
  );
};

export default ResultPage;
