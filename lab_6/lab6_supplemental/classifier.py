import cv2
import sys
import csv
import time
import numpy as np
from sklearn.cluster import KMeans

COLOR_MAPS = {
    'green' : [147, 72.6, 28.6],
    'red' : [355, 67.7, 62.0],
    'blue' : [233, 58.6, 22.7],
}


# Hyperparameters
IMAGE_HEIGHT = 128
IMAGE_WIDTH = 128
NUM_CHANNELS = 3
NUM_CLASSES = 6
IMAGE_SIZE = (IMAGE_HEIGHT, IMAGE_WIDTH)

def preprocess(img):
    # Isolate the region of interest
    # TODO: Isolate based on color
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    masked_imgs = []
    
    # Get all pixels that are within the color range
    for mask in COLOR_MAPS:
        h, s, v = COLOR_MAPS[mask]
        
        upper = np.array([h + 10, s + 10, v + 10])
        lower = np.array([h - 10, s - 10, v - 10])
        
        mask = cv2.inRange(hsv, lower, upper)
        masked_imgs.append(cv2.bitwise_and(img, img, mask=mask))
    
    # Get the largest region of interest
    largest = cv2.bitwise_or(masked_imgs[0], masked_imgs[1])
    largest = cv2.bitwise_or(largest, masked_imgs[2])
    
    cv2.imshow("Largest", largest)
    cv2.waitKey()
    
    roi = cv2.bitwise_and(img, img, mask=largest)
    
    cv2.imshow("ROI", roi)
    cv2.waitKey()
        
    return bilateral

### Load training images and labels
imageDirectory = './2022Fimgs/'

with open(imageDirectory + 'train.txt', 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)

train = []
for i in range(len(lines)):
    # Get color image
    img = cv2.imread(imageDirectory + lines[i][0] + ".png", 1 if NUM_CHANNELS == 3 else 0)
    img = preprocess(img)
    train.append(img)

train = np.array(train)

# here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
train_data = train.flatten().reshape(len(lines), IMAGE_HEIGHT*IMAGE_WIDTH)
train_data = train_data.astype(np.float32)

# read in training labels
train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])


### Train classifier
# TODO: Train your classifier here



### Run test images
with open(imageDirectory + 'test.txt', 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)

correct = 0.0
confusion_matrix = np.zeros((6,6))

for i in range(len(lines)):
    original_img = cv2.imread(imageDirectory+lines[i][0]+".png", 1 if NUM_CHANNELS == 3 else 0)
    test_img = np.array(preprocess(original_img))
    if(__debug__):
        cv2.imshow(Title_images, original_img)
        cv2.imshow(Title_resized, test_img)
        key = cv2.waitKey()
        if key==27:    # Esc key to stop
            break
        
    test_img = test_img.flatten().reshape(1, IMAGE_HEIGHT*IMAGE_WIDTH)
    test_img = test_img.astype(np.float32)

    test_label = np.int32(lines[i][1])

    # TODO: Add model prediction here

    if test_label == ret:
        print(str(lines[i][0]) + " Correct, " + str(ret))
        correct += 1
        confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
    else:
        confusion_matrix[test_label][np.int32(ret)] += 1
        
        print(str(lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
        print("\tneighbours: " + str(neighbours))
        print("\tdistances: " + str(dist))



print("\n\nTotal accuracy: " + str(correct/len(lines)))
print(confusion_matrix)