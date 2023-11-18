import os
import cv2
import numpy as np


DATASETS = ['2022Fheldout/', '2022Fimgs/', '2023Simgs/', '2023Fimgs/']

images = []
labels = []

for dataset in DATASETS:
    labels_file = open(dataset + 'labels.txt', 'r')
    l_d = labels_file.readlines()
    labels_file.close()
    l_d = [label.strip().split(",") for label in l_d]
    
    mapping = dict()
    
    for label in l_d:
        mapping[int(label[0])] = int(label[1])
    
    for filename in os.listdir(dataset):
        if filename.endswith('.jpg') or filename.endswith('.png'):
            img = cv2.imread(dataset + filename)
            images.append(img)
            labels.append(mapping[int(filename[:-4])])

f = open('combined/labels.txt', 'w')

for idx in range(len(images)):
    cv2.imwrite('combined/' + str(idx) + '.jpg', images[idx])
    f.write(str(idx) + ',' + str(labels[idx]) + '\n')

f.close()

