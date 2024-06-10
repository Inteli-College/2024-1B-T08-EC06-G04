# app.py
import routers
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

origins = ["*"]

# Habilita o CORS

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Adiciona prefixo API aos roteadores
app.include_router(routers.router, prefix="/api")

if __name__ == "__main__":
    uvicorn.run(app, port=8000)
