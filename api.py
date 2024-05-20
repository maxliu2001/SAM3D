from flask import Flask, request, jsonify
from SAM_package.model import SAM_client
import numpy as np
import cv2
import base64
import logging

logging.basicConfig(level=logging.DEBUG)
app = Flask(__name__)

@app.route('/set_image', methods=['POST'])
def set_image():
    try:
        singleton = SAM_client()
        data = request.json.get('data')
        if not data:
            raise ValueError("No data provided")

        image_data = base64.b64decode(data)
        np_arr = np.frombuffer(image_data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        singleton.set_image(image)
        return jsonify({'message': 'Image updated in SAM successfully'})
    except Exception as e:
        logging.error(f"Error occurred: {e}", exc_info=True)
        return jsonify({'error': str(e)}), 500

@app.route('/predict', methods=['POST'])
def predict():
    try:
        singleton = SAM_client()
        input_point = request.json.get('input_point')
        input_label = request.json.get('input_label')
        mask_input = request.json.get('mask_input')
        input_box = request.json.get('input_box')
        multimask = request.json.get('multimask')

        input_point = np.array(input_point)
        input_label = np.array(input_label)
        mask_input = np.array(mask_input) if mask_input is not None else None
        input_box = np.array(input_box) if input_box else None

        if input_point.size == 0:
            input_point = None
        
        if input_label.size == 0:
            input_label = None

        if input_box is not None and input_box.size == 0:
            input_box = None

        if mask_input is not None and mask_input.size > 0:
            mask_input = mask_input[np.newaxis, :, :]
        else:
            mask_input = None

        mask, scores, logit = singleton.predict(input_point, input_label, mask_input, input_box, multimask)
        mask = mask.tolist()
        scores = scores.tolist()
        logit = logit.tolist()
        return jsonify({'message': 'Image segmented successfully',
                        'mask': mask,
                        'scores': scores,
                        'logit': logit})
    except Exception as e:
        logging.error(f"Error occurred: {e}", exc_info=True)
        return jsonify({'error': str(e)}), 500


@app.route('/hello', methods=['GET'])
def hello():
    return jsonify({'message': 'Hello from SAM server'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)