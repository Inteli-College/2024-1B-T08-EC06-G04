import React from 'react';

const Header = ({ connected }) => {
  return (
    <header className="App-header text-center w-full mb-8 ">
      <h1 className="text-4xl font-bold mb-4">Teleoperação do Robô</h1>
      {!connected && <div className="popup bg-red-500 text-white p-2 rounded shadow-lg fixed top-5 right-5">Robô não conectado</div>}
    </header>
  );
};

export default Header;
