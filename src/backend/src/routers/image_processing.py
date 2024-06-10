from controllers.img_proc.image_controller import ImageData, process_image
from fastapi import APIRouter, Body, HTTPException

router = APIRouter()

# Rota para procesamento da imagem


@router.post("/process_image", status_code=200)
async def process_image_endpoint(data: ImageData = Body(...)):
    try:
        return await process_image(data)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
