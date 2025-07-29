import os

class Config:
    HOST = os.getenv('FLASK_RUN_HOST', '127.0.0.1')
    PORT = int(os.getenv('FLASK_RUN_PORT', 8081))
    DEBUG = os.getenv('FLASK_DEBUG', '0') == '1'