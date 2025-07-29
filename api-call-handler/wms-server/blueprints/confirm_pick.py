from flask_restx import Namespace, Resource, fields


confirm_pick_ns = Namespace('confirmPick', description='Confirm Pick operations')

confirm_pick_model = confirm_pick_ns.model('ConfirmPick', {
    'pickId': fields.Integer(required=True, description='The ID of the order to confirm'),
    'pickSuccessful': fields.Boolean(required=True, description='Indicates if the pick was successful'),
    'errorMessage': fields.String(required=False, description='Error message if the pick was not successful'),
    'itemBarcode': fields.Integer(required=True, description='Barcode of the item picked')
})

@confirm_pick_ns.route('/')
class ConfirmPick(Resource):
    @confirm_pick_ns.expect(confirm_pick_model)
    def post(self):
        """
        Confirm a pick operation.
        """
        data = confirm_pick_ns.payload
        pickSuccessful = data.get('pickSuccessful')
        if pickSuccessful is True:
            return {
                'status': 'success',
                'message': 'Pick confirmed successfully',
                'data': data
            }, 200
        else:
            return {
                'status': 'error',
                'message': data.get('errorMessage', 'Pick confirmation failed'),
                'data': data
            }, 400
    