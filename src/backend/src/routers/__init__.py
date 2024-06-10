from fastapi import APIRouter
from .crud import router as crud
from .image_processing import router as image_processing

router = APIRouter()

router.include_router(crud, prefix="/crud")
router.include_router(image_processing, prefix="/image_processing")