from flask import Blueprint
from flask_restx import Api
from .pick import pick_ns

blueprint = Blueprint('api', __name__)
api = Api(blueprint, title='Client', version='1.0', description='Dummy Client API')

api.add_namespace(pick_ns)
