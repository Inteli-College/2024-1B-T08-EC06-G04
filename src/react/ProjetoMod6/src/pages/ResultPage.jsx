import React, { useState } from 'react';
import { useLocation } from 'react-router-dom';

const ResultPage = () => {
  const location = useLocation();
  const { processedImage } = location.state || {};
  const rows = [
    { id: 1, version: 10, status: "Success", image: "Vampeta", processedImage: "data:image/jpeg;base64,<base64_data_1>" },
    { id: 2, version: 20, status: "Fail", image: "Ronaldinho", processedImage: "data:image/jpeg;base64,<base64_data_2>" },
    { id: 3, version: 30, status: "Fail", image: "Ronaldo", processedImage: "data:image/jpeg;base64,<base64_data_3>" },
    { id: 4, version: 40, status: "Success", image: "Rivaldo", processedImage: "data:image/jpeg;base64,<base64_data_4>" },
    { id: 5, version: 50, status: "Success", image: "Romario", processedImage: "data:image/jpeg;base64,<base64_data_5>" },
  ];

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
              <td className="px-4 py-2 border border-gray-300">{row.status}</td>
              <td className="px-4 py-2 border border-gray-300">
                <button
                  className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded"
                  onClick={() => openPopup(row.processedImage)}>

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
            <img src={currentImage} alt="Processed" className="w-[400px] h-[400px] bg-slate-500" />

            <button
              className="px-4 py-2 bg-red-500 hover:bg-red-700 text-white rounded"
              onClick={closePopup}>

              Fechar
            </button>

          </div>
        </div>
      )}
    </div>
  );
};

export default ResultPage;
