import React, { useEffect, useState } from 'react';

const PopupNotification = ({ message, onClose }) => {
  const [isVisible, setIsVisible] = useState(true);

  useEffect(() => {
    const timer = setTimeout(() => {
      setIsVisible(false);
    }, 2500); 

    const removeTimer = setTimeout(() => {
      onClose();
    }, 3000); 

    return () => {
      clearTimeout(timer);
      clearTimeout(removeTimer);
    };
  }, [onClose]);

  return (
    <div className={`fixed top-4 right-4 bg-green-500 text-white px-4 py-2 rounded shadow-lg transition-opacity duration-500 ${isVisible ? 'opacity-100 animate-fadeIn' : 'opacity-0 animate-fadeOut'}`}>
      {message}
    </div>
  );
};

export default PopupNotification;
