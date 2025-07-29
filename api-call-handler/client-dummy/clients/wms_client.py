
import requests


class WMSClient:
    def __init__(self, base_url):
        self.base_url = base_url

    def confirm_pick(self, pick_id, pick_successful, error_message,item_barcode):
        url = f"{self.base_url}/confirmPick"
        payload = {
            "pickId": pick_id,
            "pickSuccessful": pick_successful,
            "errorMessage": error_message,
            "itemBarcode": item_barcode
        }
        response = requests.post(url, json=payload)
        return response.json()