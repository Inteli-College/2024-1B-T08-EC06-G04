import typer
import inquirer
from yaspin import yaspin
import time
import requests
from threading import Thread

app = typer.Typer()

# Constants for the API
API_BASE_URL = "http://127.0.0.1:8000"
ROBOT_MOVEMENT_ENDPOINT = f"{API_BASE_URL}/move"
ROBOT_STOP_ENDPOINT = f"{API_BASE_URL}/stop"
ROBOT_CONNECT_ENDPOINT = f"{API_BASE_URL}/connect"
ROBOT_DISCONNECT_ENDPOINT = f"{API_BASE_URL}/disconnect"

def process_action(action, data=None):
    with yaspin(text="Processing...", color="yellow") as spinner:
        if action == 'Move':
            response = requests.post(ROBOT_MOVEMENT_ENDPOINT, json=data)
        elif action == 'Stop':
            response = requests.post(ROBOT_STOP_ENDPOINT)
        elif action == 'Connect':
            response = requests.post(ROBOT_CONNECT_ENDPOINT)
        elif action == 'Disconnect':
            response = requests.post(ROBOT_DISCONNECT_ENDPOINT)

        spinner.stop()
        if response:
            typer.echo(f"Action: {action}, Response: {response.text}")

@app.command()
def robo():
    running = True
    while running:
        questions = [
            inquirer.List(
                'action',
                message='What action do you want to perform?',
                choices=['Move', 'Stop', 'Connect', 'Disconnect', 'Exit']
            ),
        ]

        answers = inquirer.prompt(questions)
        action = answers['action']

        if action == 'Move':
            move_questions = [
                inquirer.Text('x', message='Enter X position'),
                inquirer.Text('y', message='Enter Y position'),
                inquirer.Text('z', message='Enter Z position')
            ]
            move_answers = inquirer.prompt(move_questions)
            process_action(action, move_answers)

        elif action == 'Stop':
            process_action(action)

        elif action == 'Connect':
            process_action(action)

        elif action == 'Disconnect':
            process_action(action)

        elif action == 'Exit':
            typer.echo("Exiting program...")
            running = False

if __name__ == "__main__":
    app()
