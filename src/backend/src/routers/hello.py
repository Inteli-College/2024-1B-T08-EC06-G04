from fastapi import APIRouter
from controllers import hello_controller

router = APIRouter()


@router.get("/hello")
async def hello():
    # Use 'await' se 'hello_controller()' for uma coroutine
    return await hello_controller.hello_controller()
