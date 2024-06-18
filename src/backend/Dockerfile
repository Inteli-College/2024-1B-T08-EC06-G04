# Start with a base Python image
FROM python:3.11

# Set the working directory in the container
WORKDIR /app

# Copy the requirements file and install Python dependencies
COPY ./backend/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of your application code
COPY ./backend/src ./src
COPY ./backend/database ./database
COPY ./backend/yoloModel ./yoloModel

# Command to run your application
CMD ["uvicorn", "src.app:app"]

