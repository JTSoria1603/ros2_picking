from flask import Blueprint
from flask_restx import Api
from .confirm_pick import confirm_pick_ns

blueprint = Blueprint('api', __name__)
api = Api(blueprint, title='WMS API', version='1.0', description='Warehouse Management System API')

api.add_namespace(confirm_pick_ns)
