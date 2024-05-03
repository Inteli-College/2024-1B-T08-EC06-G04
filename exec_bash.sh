#!/bin/bash

cd src

python3 -m venv venv

source venv/bin/activate

pip install typer inquirer

cd project_ws

TYPER_PATH=$(pip show typer | grep "Location:" | awk '{print $2}')

if [ -z "$TYPER_PATH" ]; then
    echo "Typer not found. Please make sure it is installed."
    exit 1
fi

# Export the PYTHONPATH environment variable
export PYTHONPATH="$PYTHONPATH:$TYPER_PATH"

# Optionally: Echo PYTHONPATH to verify it
echo "Updated PYTHONPATH: $PYTHONPATH"

colcon build

ros2 run cli_interface main

trap "echo 'Limpando... (apagando venv)'; rm -rf venv" EXIT
