#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from sam3d.srv import Segment, SegmentRequest
import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt


def create_test_image():
    image_path = 'src/sam3d/truck.jpg'
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image

def basic_tests(segment_image):
    # Prepare the request
    req = SegmentRequest()
    bridge = CvBridge()
    
    # Create a test image
    cv_image = create_test_image()
    req.image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    # Set the input points
    point1 = Point()
    point1.x = 500
    point1.y = 375
    point2 = Point()
    point2.x = 1125
    point2.y = 625
    req.points = [point1, point2]

    # Set the point masks
    req.point_masks = [True, True]

    # Set the boolean flag
    req.multimask = True
    req.manual = False

    # Call the service
    resp = segment_image(req)
    
    # Print the response
    rospy.loginfo("Received response from segment_image service")

    for i, (mask, score) in enumerate(zip(resp.masks, resp.scores)):
        plt.figure(figsize=(10,10))
        mask_image = bridge.imgmsg_to_cv2(mask, desired_encoding="rgb8")
        plt.imshow(mask_image)
        plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
        plt.axis('off')
        plt.show()

    rospy.loginfo("Testing response with mask input")
    # Test mask input
    new_req = SegmentRequest()
    new_req.image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    # Reuse the best logit from the previous call
    if resp.logits:
        best_logit_index = np.argmax(resp.scores)
        new_req.mask_input = resp.logits[best_logit_index]

    # Set the input points
    point1 = Point()
    point1.x = 500
    point1.y = 375
    point2 = Point()
    point2.x = 1125
    point2.y = 625
    new_req.points = [point1, point2]

    # Set the point masks
    new_req.point_masks = [1, 1]

    # Set the input box
    # new_req.input_box = [600, 200, 1200, 800]

    # Set the boolean flag
    new_req.multimask = False
    req.manual = False

    # Call the service with mask input
    resp = segment_image(new_req)

    plt.figure(figsize=(10,10))
    best_mask_image = bridge.imgmsg_to_cv2(resp.masks[0], desired_encoding="rgb8")
    plt.imshow(best_mask_image)
    plt.title(f"Best Mask As Input", fontsize=18)
    plt.axis('off')
    plt.show()
    
    # Print the response
    rospy.loginfo("Received response from segment_image service with mask input")

    rospy.loginfo("Testing response with box input")
    # Test box input
    box_req = SegmentRequest()
    box_req.image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    box_req.input_box = [600, 200, 1200, 800]
    box_req.multimask = False
    resp = segment_image(box_req)

    if resp.masks:
        best_mask_image = bridge.imgmsg_to_cv2(resp.masks[0], desired_encoding="rgb8")
        plt.imshow(best_mask_image)
        plt.title(f"Box Generated Mask", fontsize=18)
        plt.axis('off')
        plt.show()
    else:
        rospy.loginfo("No masks received in response from segment_image service")
    
    # Print the response
    rospy.loginfo("Received response from segment_image service with box input")

    rospy.loginfo("Testing response with box and point input")
    # Test box and point input
    box_req = SegmentRequest()
    box_req.image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    box_req.input_box = [425, 600, 700, 875]
    point1 = Point()
    point1.x = 575
    point1.y = 750
    box_req.points = [point1]
    box_req.point_masks = [0]
    box_req.multimask = False
    req.manual = False
    resp = segment_image(box_req)

    if resp.masks:
        best_mask_image = bridge.imgmsg_to_cv2(resp.masks[0], desired_encoding="rgb8")
        plt.imshow(best_mask_image)
        plt.title(f"Box and Point Generated Mask", fontsize=18)
        plt.axis('off')
        plt.show()
    else:
        rospy.loginfo("No masks received in response from segment_image service")
    
    # Print the response
    rospy.loginfo("Received response from segment_image service with box and point input")

def main():
    rospy.init_node('test_client')

    rospy.wait_for_service('segment_image')
    try:
        segment_image = rospy.ServiceProxy('segment_image', Segment)

        # Running basic tests from SAM notebook
        basic_tests(segment_image)

        # Testing GUI integration
        req = SegmentRequest()
        bridge = CvBridge()
        
        # Create a test image
        cv_image = create_test_image()
        req.image = bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        req.manual = True
        req.multimask = True
        resp = segment_image(req)

        if resp.masks:
            best_mask_image = bridge.imgmsg_to_cv2(resp.masks[0], desired_encoding="bgr8")
            plt.imshow(best_mask_image)
            plt.title(f"Box and Point Generated Mask", fontsize=18)
            plt.axis('off')
            plt.show()
        else:
            rospy.loginfo("No masks received in response from segment_image service")
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

if __name__ == '__main__':
    main()
