import React, { useState, useEffect } from 'react';
import PopupNotification from './PopupMorte';

// Componente que exibe uma tabela com as imagens processadas, pede as linhas(rows), uma função openPopup, uma string deleteEndpoint e uma função fetchRows
const ImageTable = ({ rows, openPopup, deleteEndpoint, fetchRows }) => {
  const [sortConfig, setSortConfig] = useState({ key: null, direction: 'ascending' });
  const [selectedRows, setSelectedRows] = useState([]);
  const [showNotification, setShowNotification] = useState(false);

  // Ordena as linhas de acordo com a configuração de ordenação
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

  // Função para requisitar a ordenação de acordo com a chave
  const requestSort = (key) => {
    let direction = 'ascending';
    if (sortConfig.key === key && sortConfig.direction === 'ascending') {
      direction = 'descending';
    }
    setSortConfig({ key, direction });
  };

  // Função para pegar a classe de ordenação de acordo com a chave
  const getSortClass = (key) => {
    if (!sortConfig) return;
    return sortConfig.key === key ? (sortConfig.direction === 'ascending' ? 'ascending' : 'descending') : undefined;
  };

  // Função para selecionar uma linha
  const handleSelectRow = (id) => {
    setSelectedRows((prevSelected) =>
      prevSelected.includes(id) ? prevSelected.filter((rowId) => rowId !== id) : [...prevSelected, id]
    );
  };
  
  // Função para deletar as linhas selecionadas, faz um request para cada linha
  // Futuramente para melhorar isto, deverá ser feito um novo endpoint que deleta os IDs em um "range"
  const handleDelete = async () => {
    try {
      await Promise.all(
        selectedRows.map(async (id) => {
          const response = await fetch(`${deleteEndpoint}/${id}`, {
            method: 'DELETE',
            headers: {
              'Content-Type': 'application/json',
            },
          });

          if (!response.ok) {
            throw new Error(`Failed to delete row with id ${id}: ${response.statusText}`);
          }
        })
      );

      // Atualiza as linhas após apagá-las
      await fetchRows();

      // Mostra notificação de sucesso
      setShowNotification(true);
      setSelectedRows([]); // Limpa as linhas selecionadas
    } catch (error) {
      console.error('Failed to delete rows:', error);
    }
  };

  return (
    <div className="w-full overflow-x-auto">
      {showNotification && (
        <PopupNotification
          message="Colunas deletadas com sucesso!"
          onClose={() => setShowNotification(false)}
        />
      )}
      {selectedRows.length > 0 && (
        <button
          className="mb-2 px-4 py-2 bg-red-600 text-white rounded"
          onClick={handleDelete}
        >
          Deletar selecionados
        </button>
      )}
      <table className="min-w-full bg-white border border-gray-200">
        <thead>
          <tr>
            <th
              className={`px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider cursor-pointer ${getSortClass('id')}`}
              onClick={() => requestSort('id')}
            >
              ID
            </th>
            <th
              className={`px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider cursor-pointer ${getSortClass('version')}`}
              onClick={() => requestSort('version')}
            >
              Versão
            </th>
            <th
              className={`px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider cursor-pointer ${getSortClass('result')}`}
              onClick={() => requestSort('result')}
            >
              Status
            </th>
            <th className="px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider">
              Imagem
            </th>
            <th className="px-6 py-3 border-b-2 border-gray-300 bg-gray-200 text-left text-xs font-semibold text-gray-600 uppercase tracking-wider">
              Selecionar
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
              <td className="px-6 py-4 border-b border-gray-200 text-sm text-center">
                <input
                  type="checkbox"
                  checked={selectedRows.includes(row.id)}
                  onChange={() => handleSelectRow(row.id)}
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
