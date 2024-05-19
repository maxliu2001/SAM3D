#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from sam3d.srv import Segment, SegmentResponse
import numpy as np
from cv_bridge import CvBridge
import cv2

# image to json formatting
import json
import base64
import requests

bridge = CvBridge()

def handle_process_image(req):
    rospy.loginfo("Received an image and %d points", len(req.points))
    rospy.loginfo("Received point masks: %s", str(req.point_masks))
    rospy.loginfo("Received input box: %s", str(req.input_box))
    rospy.loginfo("Multimask flag: %s", str(req.multimask))

    # Convert ROS image to OpenCV image
    cv_image = cv2.cvtColor(bridge.imgmsg_to_cv2(req.image), cv2.COLOR_BGR2RGB)

    # Convert OpenCV image to json payload
    _, buffer = cv2.imencode('.jpg', cv_image)
    image_base64 = base64.b64encode(buffer).decode('utf-8')
    payload = json.dumps({'data': image_base64})

    # Set image for SAM
    url = ''
    headers = {'Content-Type': 'application/json'}
    response = requests.post(url, headers=headers, data=payload)

    print(response.json())

    # Extract input arrays from request
    input_box = np.array(req.input_box)
    input_point = np.array([[point.x, point.y] for point in req.points])
    input_label = np.array(req.point_masks, dtype=int)

    # Generate mask from SAM
    pred_url = ''
    pred_headers = {'Content-Type': 'application/json'}
    pred_payload = {
        'input_point': input_point.tolist(),
        'input_label': input_label.tolist(),
        'multimask': req.multimask
    }
    response = requests.post(pred_url, headers=headers, data=json.dumps(pred_payload))

    if response.status_code != 200:
        rospy.logerr("Failed to get response from prediction endpoint")
        return SegmentResponse(masks=[], scores=[], logits=[])
    
    response_data = response.json()
    masks = np.array(response_data['mask'])
    scores = np.array(response_data['scores'])
    logits = np.array(response_data['logit'])

    uint8_masks = (masks * 255).astype(np.uint8)

    # Apply mask to original image
    masked_images = []
    for mask in uint8_masks:
        masked_image = cv_image.copy()
        masked_image[mask == 0] = 0
        masked_images.append(masked_image)

    ros_masks = [bridge.cv2_to_imgmsg(mask) for mask in uint8_masks]
    ros_logits = [bridge.cv2_to_imgmsg(logit) for logit in logits]
    ros_masked_images = [bridge.cv2_to_imgmsg(masked_image, encoding="bgr8") for masked_image in masked_images]

    return SegmentResponse(masks=ros_masked_images, scores=scores.tolist(), logits=ros_logits)

def sam_node():
    rospy.init_node('sam_node')
    service = rospy.Service('segment_image', Segment, handle_process_image)
    rospy.loginfo("Service 'Segment' is ready to receive requests.")
    rospy.spin()

if __name__ == '__main__':
    sam_node()
