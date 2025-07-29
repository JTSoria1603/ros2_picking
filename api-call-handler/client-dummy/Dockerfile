FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Crear directorio de trabajo
WORKDIR /app

# Instalar pip y dependencias del sistema para eventlet + Flask
RUN apt update && apt install -y \
    python3-pip \
    python3-dev \
    libffi-dev \
    libssl-dev \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copiar el código del HMI
COPY . /app

# Instalar dependencias de Python (Flask + SocketIO + eventlet)
RUN pip install --no-cache-dir flask flask-socketio eventlet flask_restx requests

EXPOSE 8080

CMD ["python3", "main.py"]