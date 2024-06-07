import base64
import cv2
import numpy as np
import json
from fastapi import HTTPException
from pydantic import BaseModel
from ultralytics import YOLO
import tinydb

# Carrega o modelo Yolo pré treinado
model = YOLO("../yoloModel/best.pt")

# Abre a base de dados
db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()

# Função para converter resultados em um JSON com um limite minimo de 0.7 de "confidence"
def results_to_json(results, model, confidence_threshold=0.7):
    detections = []
    for result in results:
        for box in result.boxes:
            if float(box.conf) >= confidence_threshold:
                detection = {
                    "class": int(box.cls),
                    "label": model.names[int(box.cls)],
                    "confidence": float(box.conf),
                    "box": {
                        "x_center": float(box.xywh[0][0]),
                        "y_center": float(box.xywh[0][1]),
                        "width": float(box.xywh[0][2]),
                        "height": float(box.xywh[0][3]),
                    },
                }
                detections.append(detection)
    return json.dumps(detections, indent=4)

class ImageData(BaseModel):
    image: str

async def process_image(data: ImageData):
    try:
        # Decodifica a imagem em formato base64
        image_data = base64.b64decode(data.image)
        np_arr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Verifica se a imagem veio corretamente
        if image is None:
            raise HTTPException(status_code=400, detail="Error loading image.")

        # Roda o YoloV8 na imagem
        results = model(image, conf=0.70)

        # Adiciona as anotações a imagem
        annotated_image = results[0].plot()

        # Encoda a imagem em formato base64
        _, buffer = cv2.imencode('.jpg', annotated_image)
        annotated_image_base64 = base64.b64encode(buffer).decode('utf-8')

        # Converte resultados em JSON
        results_json = results_to_json(results, model)

        result_var = False

        if results_json != "[]":
            result_var = True

        # Salva na base de dados
        new_entry = {
            "version": "1.0",
            "image": annotated_image_base64,
            "result": result_var
        }
        db.insert(new_entry)
        
        return {"processed_image": annotated_image_base64, "results": results_json}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
