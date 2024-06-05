import React from "react";

const PhotoButton = ({ onClick }) => {
    return (
        <button
        className="bg-green-300 hover:bg-green-500 text-white font-bold py-10 px-10 rounded-full border-2 border-neutral-500 hover:bg-green-400"
        onClick={onClick}
        >
        <img src="/images/camera.svg" alt="Tirar Foto" className="w-20 h-20"/>
        </button>
    );
    }

export default PhotoButton;