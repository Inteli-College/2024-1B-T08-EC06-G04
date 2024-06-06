import React from 'react';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import MainPage from './pages/MainPage';
import ResultPage from './pages/ResultPage';

const App = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="*" element={<h1>Page not found</h1>} />
        <Route path="/" element={< MainPage />} />
        <Route path="/result" element={< ResultPage />} />
      </Routes>
    </BrowserRouter>
  );
};

export default App;