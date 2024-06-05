from fastapi import APIRouter, HTTPException
from controllers.crud import update, create, delete, read

router = APIRouter()

@router.put("/update/{id}", status_code=200)
async def update_by_id(id: int, version: str, image: str, result: str):
    try:
        return await update(version, image, result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/create", status_code=201)
async def create_by_id(version: str, image: str, result: str):
    try:
        return await create(version, image, result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.delete("/delete/{id}", status_code=200)
async def delete_by_id(id: int, version: str):
    try:
        return await delete(version)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/read/{id}", status_code=200)
async def read_by_id(id: int, version: str):
    try:
        return await read(version)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

