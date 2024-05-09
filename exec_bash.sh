#!/bin/bash

# Executa ao sair do script
trap "echo 'Limpando... (apagando venv)'; rm -rf $(pwd)/src/venv" EXIT

cd src

python3 -m venv venv

source venv/bin/activate

pip install typer inquirer

cd project_ws

# Pega o caminho da instaância do venv
TYPER_PATH=$(pip show typer | grep "Location:" | awk '{print $2}')

# Se não achar
if [ -z "$TYPER_PATH" ]; then
    echo "Typer not found. Please make sure it is installed."
    exit 1
fi

# Exporta a variável de ambiente PYTHONPATH com o caminho do venv
export PYTHONPATH="$PYTHONPATH:$TYPER_PATH"


echo "Updated PYTHONPATH: $PYTHONPATH"

# Builda e roda o ros2
colcon build

source install/local_setup.bash

ros2 run cli_interface main