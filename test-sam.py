from SAM_package.model import SAM_client
from SAM_package.helper import *
import numpy as np
import torch
import matplotlib.pyplot as plt
import cv2

image = cv2.imread('images/truck.jpg')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

if __name__ == '__main__':
    test = SAM_client()
    test.set_image(image)

    input_point = np.array([[500, 375]])
    input_label = np.array([1])

    masks, scores, logits = test.predict(
        input_point,
        input_label,
        True
    )
    print(masks)
    print(scores)
    print(logits)

    for i, (mask, score) in enumerate(zip(masks, scores)):
        print(mask)
        plt.figure(figsize=(10,10))
        plt.imshow(image)
        show_mask(mask, plt.gca())
        show_points(input_point, input_label, plt.gca())
        plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
        plt.axis('off')
        plt.show()
    