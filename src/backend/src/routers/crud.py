from fastapi import APIRouter, HTTPException
from controllers.crud import update

router = APIRouter()


@router.put("/update/{id}", status_code=200)
async def update_by_id(id: int, version: str, image: str, result: str):
    try:
        return await update(version, image, result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
