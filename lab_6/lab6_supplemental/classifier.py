import cv2
import sys
import csv
import time
import numpy as np

# Hyperparameters
IMAGE_HEIGHT = 64
IMAGE_WIDTH = 64
IMAGE_SIZE = (IMAGE_HEIGHT, IMAGE_WIDTH)

def preprocess(img):
    # Isolate the region of interest
    # TODO: Isolate based on color
    
    img = cv2.resize(img, (IMAGE_SIZE))
    
    return image

### Load training images and labels
imageDirectory = './2022Fimgs/'

with open(imageDirectory + 'train.txt', 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)

train = []
for i in range(len(lines)):
    img = cv2.imread(imageDirectory + lines[i][0] + ".png", 0)
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
    original_img = cv2.imread(imageDirectory+lines[i][0]+".png",0)
    test_img = np.array(preprocess(cv2.imread(imageDirectory+lines[i][0]+".png",0),IMAGE_SIZE))
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