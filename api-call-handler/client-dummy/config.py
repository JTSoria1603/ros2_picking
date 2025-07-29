import os

class Config:
    HOST = os.getenv('FLASK_RUN_HOST', '0.0.0.0')
    PORT = int(os.getenv('FLASK_RUN_PORT', 8080))
    DEBUG = os.getenv('FLASK_DEBUG', '0') == '1'
    WMS_URL = os.getenv('WMS_URL', 'http://wms-server:8081')