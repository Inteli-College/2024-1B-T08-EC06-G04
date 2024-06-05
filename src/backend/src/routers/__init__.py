from fastapi import APIRouter
from . import hello
from . import crud

router = APIRouter()

router.include_router(hello.router, prefix="/hello")

router.include_router(crud.router, prefix="/crud")
