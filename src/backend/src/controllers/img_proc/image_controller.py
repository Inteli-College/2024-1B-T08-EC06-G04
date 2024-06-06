import base64
import cv2
import numpy as np
import json
from fastapi import HTTPException
from pydantic import BaseModel
from ultralytics import YOLO
import tinydb

# Load your YOLO model
model = YOLO("../yoloModel/best.pt")

# Initialize the database
db = tinydb.TinyDB("../database/db.json")
User = tinydb.Query()

# Function to convert results to JSON
def results_to_json(results, model):
    detections = []
    for result in results:
        for box in result.boxes:
            detection = {
                "class": int(box.cls),
                "label": model.names[int(box.cls)],  # Add the label to the JSON
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
        # Decode the base64 image
        image_data = base64.b64decode(data.image)
        np_arr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Verify if the image was loaded correctly
        if image is None:
            raise HTTPException(status_code=400, detail="Error loading image.")

        # Run YOLOv8 inference on the image
        results = model(image)

        # Annotate the image
        annotated_image = results[0].plot()

        # Encode the annotated image to base64
        _, buffer = cv2.imencode('.jpg', annotated_image)
        annotated_image_base64 = base64.b64encode(buffer).decode('utf-8')

        # Convert results to JSON
        results_json = results_to_json(results, model)

        # Save the results to the database
        new_entry = {
            "version": "1.0",
            "image": annotated_image_base64,
            "result": results_json
        }
        db.insert(new_entry)
        
        return {"processed_image": annotated_image_base64, "results": results_json}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
