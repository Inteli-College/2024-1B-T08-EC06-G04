import React, { useState } from 'react';

const ImageTable = ({ rows, openPopup }) => {
  const [sortConfig, setSortConfig] = useState({ key: null, direction: 'ascending' });

  const sortedRows = React.useMemo(() => {
    let sortableRows = [...rows];
    if (sortConfig !== null) {
      sortableRows.sort((a, b) => {
        if (a[sortConfig.key] < b[sortConfig.key]) {
          return sortConfig.direction === 'ascending' ? -1 : 1;
        }
        if (a[sortConfig.key] > b[sortConfig.key]) {
          return sortConfig.direction === 'ascending' ? 1 : -1;
        }
        return 0;
      });
    }
    return sortableRows;
  }, [rows, sortConfig]);

  const requestSort = (key) => {
    let direction = 'ascending';
    if (sortConfig.key === key && sortConfig.direction === 'ascending') {
      direction = 'descending';
    }
    setSortConfig({ key, direction });
  };

  const getSortClass = (key) => {
    if (!sortConfig) return;
    return sortConfig.key === key ? (sortConfig.direction === 'ascending' ? 'ascending' : 'descending') : undefined;
  };

  return (
    <div className="w-full overflow-x-auto">
      <table className="min-w-full bg-white border border-gray-200">
        <thead>
          <tr>
            <th
              className={`px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider cursor-pointer ${getSortClass('id')}`}
              onClick={() => requestSort('id')}
            >
              Id
            </th>
            <th
              className={`px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider cursor-pointer ${getSortClass('version')}`}
              onClick={() => requestSort('version')}
            >
              Version
            </th>
            <th
              className={`px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider cursor-pointer ${getSortClass('result')}`}
              onClick={() => requestSort('result')}
            >
              Status
            </th>
            <th className="px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider">
              Image
            </th>
          </tr>
        </thead>
        <tbody>
          {sortedRows.map((row) => (
            <tr key={row.id} className="hover:bg-gray-100">
              <td className="px-6 py-4 border-b border-gray-200 text-sm">{row.id}</td>
              <td className="px-6 py-4 border-b border-gray-200 text-sm">{row.version}</td>
              <td className="px-6 py-4 border-b border-gray-200 text-sm">{row.result}</td>
              <td className="px-6 py-4 border-b border-gray-200 text-sm">
                <img
                  src={`data:image/png;base64,${row.image}`}
                  alt="Processed"
                  className="w-10 h-10 cursor-pointer object-cover border border-gray-300"
                  onClick={() => openPopup(row.image)}
                />
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default ImageTable;
