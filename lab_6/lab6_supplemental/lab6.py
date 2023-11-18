import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms
from torchvision.transforms import v2

import numpy as np
import cv2

import os
from torchvision.io import read_image

from torch.utils.data import Dataset
from torch.utils.data import DataLoader
import tqdm
import imutils

class SignClassifier(nn.Module):
    def __init__(self, num_channels, num_classes):
        super(SignClassifier, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=num_channels, out_channels=16, kernel_size=3, bias=False)
        self.bn1 = nn.BatchNorm2d(16)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(in_channels=16, out_channels=32, kernel_size=3, bias=False) 
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3, bias=False)
        self.bn3 = nn.BatchNorm2d(64)
        self.fc1 = nn.Linear(64 * 26 * 26, 1024)
        self.fc2 = nn.Linear(1024, 512)
        self.out = nn.Linear(512, num_classes)
        self.dropout = nn.Dropout(0.2)
        
    def forward(self, x):
        B, C, H, W = x.shape
        x = self.pool(F.relu(self.conv1(x)))
        x = self.bn1(x)
        x = self.pool(F.relu(self.conv2(x)))        
        x = self.bn2(x)
        x = self.pool(F.relu(self.conv3(x)))        
        x = self.bn3(x)
        x = self.dropout(x)
        x = x.view(B, -1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.out(x)

class CustomImageDataset(Dataset):
    def __init__(self, annotations_file, img_dir, transform=None, target_transform=None, test=False):
        self.img_labels = open(annotations_file, 'r').readlines()
        self.img_labels = [label.strip().split(",") for label in self.img_labels]
        self.img_labels = [(label[0], int(label[1])) for label in self.img_labels]
        self.img_dir = img_dir
        self.transform = transform
        self.target_transform = target_transform
        self.test = test

    def __len__(self):
        return len(self.img_labels)

    def __getitem__(self, idx):
        img_path = os.path.join(self.img_dir, self.img_labels[idx][0] + '.jpg')
        image = read_image(img_path)
        image = crop_sign(image.permute(1,2,0).numpy())
        image = torch.Tensor(image).permute(2,0,1)
        label = self.img_labels[idx][1]
        
        if not self.test:
            if self.transform:
                image = self.transform(image)
            if self.target_transform:
                label = self.target_transform(label)
            
        # Normalize
        image /= 255.0
        
        return image, label

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
    ALLOWANCE = 25
    
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


if __name__ == '__main__':
    model = SignClassifier(3, 6)
    model = model.cuda()
    
    transforms = transforms.Compose([
        v2.Resize((224,224), antialias=False),
        v2.RandomPerspective(p=0.2)
    ])
    
    dataset = CustomImageDataset('combined/labels.txt', 'combined/', transform=transforms)
    train_set, test_set = torch.utils.data.random_split(dataset, [int(0.8 * len(dataset)), len(dataset) - int(0.8 * len(dataset))])
    train_loader = DataLoader(train_set, batch_size=64, shuffle=True, num_workers=2, pin_memory=True)
    test_loader = DataLoader(test_set, batch_size=64, shuffle=False, num_workers=2, pin_memory=True)
    
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=1e-4, weight_decay=1e-5)
    
    for epoch in range(25):
        loop = tqdm.tqdm(train_loader)
        running_loss = 0.0
        for data in loop:
            inputs, labels = data
            inputs = inputs.cuda()
            labels = labels.cuda()
            optimizer.zero_grad(set_to_none=True)
            
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()
            loop.set_postfix(loss=running_loss)
            
        
        print('Epoch %d loss: %.3f' % (epoch + 1, running_loss / len(train_loader) * 64))
        
        correct = 0
        total = 0
        with torch.no_grad():
            for data in test_loader:
                inputs, labels = data
                inputs = inputs.cuda()
                labels = labels.cuda()
                outputs = model(inputs)
                outputs = torch.softmax(outputs, dim=1)
                pred = torch.argmax(outputs, dim=1)
                total += labels.size(0)
                correct += (pred == labels).sum().item()
        
        print('Accuracy of the network on the test images: %d %%' % (100 * correct / total))
    