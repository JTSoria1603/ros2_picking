from flask_restx import Namespace, Resource, fields
from clients.ros_client import RosClient
from clients.wms_client import WMSClient
from config import Config
pick_ns = Namespace('pick', description='Pick operation')


pick_model = pick_ns.model('Pick', {
    'pickId': fields.Integer(required=True, description='The ID of the order to pick'),
    'quantity': fields.Integer(required=True, description='Quantity of items to pick')
})

@pick_ns.route('/')
@pick_ns.route('')
class Pick(Resource):
    def options(self):
        '''CORS preflight handler'''
        from flask import make_response
        response = make_response('', 200)
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'POST, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type, Authorization'
        response.headers['Access-Control-Allow-Credentials'] = 'true'
        return response
    @pick_ns.expect(pick_model)
    def post(self):
        """
        Perform a pick operation.
        """
        ros_client = RosClient()
        wms_client = WMSClient(Config.WMS_URL)
        data = pick_ns.payload
        barcode = ros_client.get_barcode()
        stack_light_status = int(ros_client.get_stack_light())
        print(f"Barcode: {barcode}, Stack Light Status: {stack_light_status}")
        wms_response = {}
        if stack_light_status == 1:
            wms_response = wms_client.confirm_pick(
                pick_id=data['pickId'],
                pick_successful=False,
                error_message='Stack light is in pause mode (door open)',
                item_barcode=barcode
            )
        elif stack_light_status == -1:
            wms_response = wms_client.confirm_pick(
                pick_id=data['pickId'],
                pick_successful=False,
                error_message='Stack light is in emergency mode',
                item_barcode=barcode
            )
        elif stack_light_status == 0:
            wms_response = wms_client.confirm_pick(
                pick_id=data['pickId'],
                pick_successful=True,
                error_message='',
                item_barcode=barcode
            )
        else:
            return {
                'status': 'error',
                'message': 'Unknown stack light status',
                'barcode': barcode
            }, 400

        return {
            'status': 'success',
            'message': 'Pick operation completed',
            'stack_light_status': stack_light_status,
            'barcode': ros_client.get_barcode(),
            'wms_data': wms_response
        }, 200
