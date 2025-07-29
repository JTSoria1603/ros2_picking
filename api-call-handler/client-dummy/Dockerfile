FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /app

RUN apt update && apt install -y \
    python3-pip \
    python3-dev \
    libffi-dev \
    libssl-dev \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

COPY . /app

RUN pip install --no-cache-dir flask flask-socketio eventlet flask_restx requests flask_cors

EXPOSE 8080

CMD ["python3", "main.py"]
