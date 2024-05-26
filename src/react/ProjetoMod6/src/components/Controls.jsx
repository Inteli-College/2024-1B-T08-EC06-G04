import React from 'react';
import { FaArrowUp, FaArrowDown, FaArrowLeft, FaArrowRight } from 'react-icons/fa';

const Controls = ({ handleButtonClick }) => {
  return (
    <div className="controls flex flex-col items-center space-y-2 ml-10">
      <button onClick={() => handleButtonClick('up')} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
        <FaArrowUp />
      </button>
      <div className="flex space-x-2">
        <button onClick={() => handleButtonClick('left')} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
          <FaArrowLeft />
        </button>
        <button onClick={() => handleButtonClick('stop')} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
          Parar
        </button>
        <button onClick={() => handleButtonClick('right')} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
          <FaArrowRight />
        </button>
      </div>
      <button onClick={() => handleButtonClick('down')} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
        <FaArrowDown />
      </button>
    </div>
  );
};

export default Controls;
