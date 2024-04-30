from typing import Union
from pydantic import BaseModel

from fastapi import FastAPI

app = FastAPI()

class MoveData(BaseModel):
    x: float
    y: float
    z: float

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.post("/move")
def move_robot(data: MoveData):
    return {"message": f"Robot moving to position X: {data.x}, Y: {data.y}, Z: {data.z}"}

@app.post('/stop')
def stop_robot():
    return {"message": "Robot stopped"}

@app.post('/connect')
def connect():
    return {"message": "Robot connected"}

@app.post('/disconnect')
def disconnect():
    return {"message": "Robot disconnected"}