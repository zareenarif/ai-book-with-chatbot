import React from 'react';

const LoadingIndicator = () => {
  return (
    <div className="loading-indicator">
      <div className="loading-spinner">
        <div className="spinner-dot"></div>
        <div className="spinner-dot"></div>
        <div className="spinner-dot"></div>
      </div>
      <p>Searching book content...</p>
    </div>
  );
};

export default LoadingIndicator;
