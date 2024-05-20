#!/usr/bin/env python

import rospy
from sam3d.srv import Segment, SegmentResponse
import numpy as np
from cv_bridge import CvBridge
import cv2
import sys
from sam3d_gui import SAMSelector
from PyQt5.QtWidgets import QApplication
import threading
import queue

# image to json formatting
import json
import base64
import requests

bridge = CvBridge()
task_queue = queue.Queue()
ros_lock = threading.Lock()
gui_event = threading.Event()

class SAMNode:
    def __init__(self):
        rospy.init_node('sam_node', anonymous=True)
        self.service = rospy.Service('segment_image', Segment, self.handle_process_image)
        rospy.loginfo("Service 'Segment' is ready to receive requests.")

    def handle_process_image(self, req):
        with ros_lock:
            rospy.loginfo("Received an image and %d points", len(req.points))
            rospy.loginfo("Received point masks: %s", str(req.point_masks))
            rospy.loginfo("Received input box: %s", str(req.input_box))
            rospy.loginfo("Received mask input: %s", str(req.mask_input.data))
            rospy.loginfo("Multimask flag: %s", str(req.multimask))
            rospy.loginfo("GUI flag: %s", str(req.manual))

            # Convert ROS image to OpenCV image
            cv_image = cv2.cvtColor(bridge.imgmsg_to_cv2(req.image, "rgb8"), cv2.COLOR_BGR2RGB)

            # Convert mask input to array
            mask_input = None
            if req.mask_input.data:
                mask_input = bridge.imgmsg_to_cv2(req.mask_input, desired_encoding="32FC1")

            # Convert OpenCV image to json payload
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            payload = json.dumps({'data': image_base64})

            # Set image for SAM
            url = ''
            headers = {'Content-Type': 'application/json'}
            response = requests.post(url, headers=headers, data=payload)

            print(response.json())

            if req.manual:
                gui_event.clear()  # Clear the event before putting a new task
                task_queue.put((cv_image, req.multimask))
                print('Selector request added to queue')
                
                # Wait for the GUI task to signal completion
                gui_event.wait()

                # Set variables to None if the arrays are empty
                input_points = np.array(self.selector.points) if self.selector.points.size > 0 else None
                input_labels = np.array(self.selector.points_masks) if self.selector.points_masks.size > 0 else None
                input_boxes = np.array(self.selector.box) if self.selector.box.size > 0 else None

                # Prepare response after GUI task completion
                pred_url = ''
                pred_headers = {'Content-Type': 'application/json'}

                # Prepare payload
                pred_payload = {
                    'input_point': input_points.tolist() if input_points is not None else None,
                    'input_label': input_labels.tolist() if input_labels is not None else None,
                    'mask_input':  None,
                    'input_box': input_boxes.tolist() if input_boxes is not None else None,
                    'multimask': req.multimask
                }

                response = requests.post(pred_url, headers=pred_headers, data=json.dumps(pred_payload))

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

                ros_logits = [bridge.cv2_to_imgmsg(logit) for logit in logits]
                ros_masked_images = [bridge.cv2_to_imgmsg(masked_image, encoding="rgb8") for masked_image in masked_images]

                return SegmentResponse(masks=ros_masked_images, scores=scores.tolist(), logits=ros_logits)

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
                'mask_input':  mask_input.tolist() if mask_input is not None else None,
                'input_box': input_box.tolist(),
                'multimask': req.multimask
            }
            response = requests.post(pred_url, headers=pred_headers, data=json.dumps(pred_payload))
            
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

            # ros_masks = [bridge.cv2_to_imgmsg(mask) for mask in uint8_masks]
            ros_logits = [bridge.cv2_to_imgmsg(logit) for logit in logits]
            ros_masked_images = [bridge.cv2_to_imgmsg(masked_image, encoding="rgb8") for masked_image in masked_images]

            return SegmentResponse(masks=ros_masked_images, scores=scores.tolist(), logits=ros_logits)

def process_tasks():
    while True:
        task = task_queue.get()
        if isinstance(task, str) and task == "exit":
            break
        else:
            cv_image, multimask = task
            selector = SAMSelector(cv_image)
            SAMNode.selector = selector  # Save the selector instance to SAMNode for later access
            selector.exec_()  # Blocking call until the dialog is closed
            
            # Signal that the GUI task is complete
            gui_event.set()

def ros_spin_thread():
    rospy.spin()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    # Initialize the ROS node in the main thread
    sam_node = SAMNode()

    # Start the ROS spinning thread
    ros_thread_instance = threading.Thread(target=ros_spin_thread)
    ros_thread_instance.start()

    # Process tasks in the main thread
    process_tasks()

    # Wait for the ROS thread to finish
    ros_thread_instance.join()
