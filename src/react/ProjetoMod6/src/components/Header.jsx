import React from "react";

// Header para informar ao usuário se o robô está ou não conectado
const Header = ({ connected }) => {
  return (
    <header className="App-header text-center w-full flex items-center justify-center text-white">
      <h1 className="text-4xl font-bold mb-4 fixed top-5 z-20">
        Teleoperação do Robô
      </h1>
      {!connected && (
        <div className="popup  bg-red-500 text-white px-5 py-2 rounded-tl rounded-bl shadow-lg p- fixed top-5 right-5 z-20">
          Robô não conectado
        </div>
      )}
    </header>
  );
};

export default Header;
