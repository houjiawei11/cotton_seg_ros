import cv2
import numpy as np
import os
from skimage.measure import regionprops
from PIL import Image, ImageEnhance
from skimage.segmentation import mark_boundaries
import dlcot.test as dltest
# import matplotlib.pyplot as plt

# https://stackoverflow.com/questions/50450654/filling-in-circles-in-opencv
def fill(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(mask, contours, -1, 255, thickness=-1)

def enhance(image):
    im = Image.fromarray(image)
    converter = ImageEnhance.Color(im)
    im2 = converter.enhance(3.0)
    image = np.array(im2)
    return image

def smooth(mask):
    cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 200:
            cv2.drawContours(mask, [c], -1, 0, -1)

def segment_one(image, depth, model):
    mask = dltest.segment(image, model)
    mask = np.bitwise_and(mask, depth > 0) * 255
    smooth(mask)
    fill(mask)
    n, labels = cv2.connectedComponents(mask)
    regions = regionprops(labels)
    one = np.array([depth[tuple(r.coords.T)].min() for r in regions])
    if len(one) > 0:
        new_mask = np.zeros(mask.shape, dtype=np.uint8)
        new_mask[tuple(regions[one.argmin()].coords.T)] = 255
        return (True, new_mask)
    else:
        return (False,)
