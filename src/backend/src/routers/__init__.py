from fastapi import APIRouter
from . import hello

router = APIRouter()

router.include_router(hello.router, prefix="/hello")
