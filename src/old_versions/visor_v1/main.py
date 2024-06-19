import streamlit as st
import cv2
import numpy as np
import io
from PIL import Image


def capture_image_bytes(cap):
    """Captura um frame da webcam e retorna como bytes."""
    ret, frame = cap.read()
    if not ret:
        return None  # Falha ao capturar a imagem

    # Converte de BGR para RGB
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Converte a imagem do OpenCV para um objeto Image do Pillow
    pil_image = Image.fromarray(frame)

    # Salva a imagem em um objeto de bytes
    img_bytes = io.BytesIO()
    pil_image.save(img_bytes, format="JPEG")
    img_bytes.seek(0)

    return img_bytes


def main():
    st.title("Visualização da Webcam")

    # Cria uma instância de captura de vídeo usando a primeira webcam detectada
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        st.error("Não foi possível abrir a câmera")
        return

    # Cria um placeholder que será atualizado em vez de recriar a imagem
    frame_holder = st.empty()

    try:
        while True:
            img_bytes = capture_image_bytes(cap)
            if img_bytes is not None:
                # Atualiza a imagem no placeholder
                frame_holder.image(
                    img_bytes, caption="Webcam Stream", use_column_width=True
                )
    finally:
        # Libera a captura ao finalizar
        cap.release()


if __name__ == "__main__":
    main()
