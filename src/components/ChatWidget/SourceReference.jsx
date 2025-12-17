import React from 'react';

const SourceReference = ({ source }) => {
  return (
    <a
      href={source.url}
      className="source-reference"
      target="_blank"
      rel="noopener noreferrer"
    >
      <span className="source-chapter">{source.chapter}</span>
      <span className="source-divider">•</span>
      <span className="source-section">{source.section}</span>
      <span className="source-score">({(source.score * 100).toFixed(0)}%)</span>
    </a>
  );
};

export default SourceReference;
