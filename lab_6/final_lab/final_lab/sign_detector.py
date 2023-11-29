import os
import sys

import cv2
import imutils
import numpy as np
import rclpy
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms
import tqdm
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from torchvision.io import read_image
from torchvision.transforms import v2


def crop_sign(image):
    img_h, img_w, c = image.shape

    LOWER_RED_1 = np.array([0, 90, 60])
    UPPER_RED_1 = np.array([5, 255, 255])

    LOWER_RED_2 = np.array([170, 70, 60])
    UPPER_RED_2 = np.array([180, 255, 255])

    LOWER_BLUE = np.array([95, 70, 40])
    UPPER_BLUE = np.array([125, 255, 255])

    LOWER_GREEN = np.array([45, 20, 0])
    UPPER_GREEN = np.array([90, 255, 255])

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV) 
    mask_red = cv2.bitwise_or(cv2.inRange(hsv.copy(), LOWER_RED_1, UPPER_RED_1), cv2.inRange(hsv.copy(), LOWER_RED_2, UPPER_RED_2))
    mask_blue = cv2.inRange(hsv.copy(), LOWER_BLUE, UPPER_BLUE)
    mask_green = cv2.inRange(hsv.copy(), LOWER_GREEN, UPPER_GREEN)

    masks = [mask_red, mask_blue, mask_green]

    for mask_idx in range(len(masks)):  
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        mask = cv2.morphologyEx(masks[mask_idx], cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        masks[mask_idx] = mask

    cumalative_mask = mask_red

    for mask in masks:
        cumalative_mask = cv2.bitwise_or(cumalative_mask, mask)
        
    cnts = cv2.findContours(cumalative_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) == 0:
        return image
    
    c = max(cnts, key=cv2.contourArea)

    (x, y, w, h) = cv2.boundingRect(c)

    if y <= 0.1 * img_h:
        return image

    # Allowances
    ALLOWANCE = 15
    
    x -= ALLOWANCE
    y -= ALLOWANCE
    
    if x < 0:
        x = 0
    if y < 0:
        y = 0
    
    w += ALLOWANCE
    h += ALLOWANCE
    
    ret = image[y:y+h,x:x+w,:]
    
    return ret

class SignClassifier(nn.Module):
    def __init__(self, num_channels, num_classes):
        super(SignClassifier, self).__init__()
        
        self.convs = nn.Sequential(
            nn.Conv2d(num_channels, 32, kernel_size=3),
            nn.ReLU(),
            nn.BatchNorm2d(32),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, kernel_size=3),
            nn.ReLU(),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, kernel_size=3),
            nn.ReLU(),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2),
            nn.Conv2d(128, 256, kernel_size=3),
            nn.ReLU(),
            nn.BatchNorm2d(256),
            nn.MaxPool2d(2),
            nn.Dropout(0.2)
        ) 

        self.fc = nn.Sequential(
            nn.Linear(256*12*12, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, num_classes)
        )
        
        
    def forward(self, x):
        B, C, H, W = x.shape
        x = self.convs(x)
        x = x.view(B, -1)
        x = self.fc(x)
        return x


class SignClassifierNode(Node):
    def __init__(self):
        # Creates the node.
        super().__init__('sign_classifier_node')
        
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "Raw Image")
        
        #Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)
        
        # Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value # Image Window Title
        if(self._display_image):
            # Set Up Image Viewing
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow(self._titleOriginal, 50, 50)
        
        #Set up QoS Profiles for LIDAR Data
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self._video_subscriber = self.create_subscription(
                CompressedImage,
                '/image_raw/compressed',
                self._image_callback,
                qos_profile)
        self._video_subscriber # Prevents unused variable warning.
        
        self.publisher = self.create_publisher(String, 'sign', 10)
        
        self.model = SignClassifier(3, 6)
        self.model.load_state_dict(torch.load('final_lab/best_model_weights.pth'))
        
    def _image_callback(self, CompressedImage):
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        
        img = self._imgBGR.copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = crop_sign(img)
        
        if(self._display_image):
            # Display the image in a window
            self.show_image(img)
        
        
        img = cv2.resize(img, (224, 224))
        img = transforms.ToTensor()(img)
        img = img.unsqueeze(0)
        
        with torch.no_grad():
            result = self.model(img)
            prob = torch.softmax(result, dim=1)
            pred = torch.argmax(prob, dim=1).item()
        
        msg = String()
        msg.data = str(pred)
        self.publisher.publish(msg)
        
        self.get_logger().info(f"Predicted {pred}")
        
    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
        
        self._user_input = cv2.waitKey(1)
        if self._user_input == ord('q'):
            cv2.destroyAllWindows()
            raise SystemExit
        
        
def main():
    rclpy.init()
    classifier = SignClassifierNode()
    
    try:
        rclpy.spin(classifier)
    except SystemExit:
        rclpy.logging.get_logger("Classifier node...").info("Shutting Down")
    
    classifier.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
    