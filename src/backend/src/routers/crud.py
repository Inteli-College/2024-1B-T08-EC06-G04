from fastapi import APIRouter, HTTPException, Query, Body
from controllers.crud.update import update
from controllers.crud.create import create
from controllers.crud.delete import delete
from controllers.crud.read import read

router = APIRouter()

@router.put("/update/{id}", status_code=200)
async def update_by_id(id: int, version: str = Body(...), image: str = Body(...), result: str = Body(...)):
    try:
        return await update(id, version, image, result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/create", status_code=201)
async def create_by_id(version: str = Body(...), image: str = Body(...), result: str = Body(...)):
    try:
        return await create(version, image, result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.delete("/delete/{id}", status_code=200)
async def delete_by_id(id: int):
    try:
        return await delete(id)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/read/{id}", status_code=200)
async def read_by_id(id: int):
    try:
        return await read(id)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
