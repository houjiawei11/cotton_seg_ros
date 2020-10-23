import cv2
import numpy as np
from glob import glob
import argparse
import os
import pickle as pkl
import features_full as ft
import regions as rg
from skimage.measure import regionprops
import time
from PIL import Image, ImageEnhance
from skimage.segmentation import mark_boundaries
# import matplotlib.pyplot as plt

REGION_AREA = 800
REGION_IGN = 100
INFER_IGN = 100
RAND_SIZE = 100
RATIO = 0.7

# https://stackoverflow.com/questions/50450654/filling-in-circles-in-opencv
def fill(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(mask, contours, -1, 255, thickness=-1)

def get_regions(mask):
    n, labels = cv2.connectedComponents(mask)
    regions = regionprops(labels)
    return [r.coords for r in regions]

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

def predict(img_rgb, regs):
    img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
    # plt.subplot(2, 2, 1)
    # plt.imshow(img_rgb)
    # plt.subplot(2, 2, 2)
    # plt.imshow(img_hsv)
    img_rgb2 = img_rgb.astype(int)
    img_g = img_rgb2[:,:,1] * 2 - img_rgb2[:,:,0] - img_rgb2[:,:,2]
    img_g = img_g > 70
    img_g = img_g.astype(np.uint8)
    img_g *= 255

    img_gg = img_rgb2[:,:,1] < 245
    img_gg = img_gg.astype(np.uint8)
    img_gg *= 255
    img_g = cv2.bitwise_and(img_g, img_gg)

    img_r = img_rgb2[:,:,0] * 2 - img_rgb2[:,:,1] - img_rgb2[:,:,2]
    img_r = img_r > 40
    img_r = img_r.astype(np.uint8)
    img_r *= 255

    img_b = img_rgb2[:,:,2] * 2 - img_rgb2[:,:,0] - img_rgb2[:,:,1]
    img_b = img_b > 40
    img_b = img_b.astype(np.uint8)
    img_b *= 255
    kernel = np.ones((5,5),np.uint8)
    mask_white = cv2.inRange(img_hsv, (0, 0, 155), (179, 100, 255))
    mask_green = cv2.inRange(img_hsv, (40, 50, 0), (100, 255, 170))
    mask_nein = cv2.inRange(img_hsv, (100, 255, 255), (179, 255, 255))
    mask_nein2 = cv2.inRange(img_hsv, (0, 255, 255), (10, 255, 255))
    mask_brown = cv2.inRange(img_hsv, (10, 60, 0), (30, 255, 150))
    mask_bb = cv2.inRange(img_gray, 200, 255)
    mask = cv2.bitwise_or(mask_green, mask_brown)
    mask = cv2.bitwise_or(mask, mask_nein)
    # mask = cv2.bitwise_or(mask, mask_nein2)
    # mask = cv2.bitwise_or(mask, img_r)
    mask = cv2.bitwise_or(mask, img_g)
    # mask = cv2.bitwise_or(mask, img_b)
    mask = cv2.bitwise_not(mask)
    mask = cv2.bitwise_and(mask, mask_white)
    # smooth(mask)
    return mask

def check_args(args):
    if not os.path.exists(args.image):
        raise ValueError("Image file does not exist")
    return args

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image" , help="Path to image", required=True)
    parser.add_argument("-o", "--output", help="Path to output image", required=True)
    args = parser.parse_args()
    return check_args(args)

def infer(model, image, regions):
    rs = regionprops(regions + 1)
    # f, _ = ft.get_features_labels(image, None, train=False, reshape=False)
    results = np.zeros(len(rs), dtype=np.uint8)
    e = enumerate(rs)
    next(e)
    mask = predict(image, regions)
    for i, r in e:
        if r.area < INFER_IGN:
            continue
        coords = r.coords
        pop = len(coords)
        n = RAND_SIZE
        if pop > n:
            choices = np.random.choice(pop, size=n, replace=False)
            choices = coords[choices]
            pop = n
        else:
            choices = coords
        # print(f.shape)
        # print(choices.shape)
        y = mask[tuple(choices.T)]
        t = np.count_nonzero(y)
        if t / pop > RATIO:
            results[i] = 255
        else:
            results[i] = 0
    mask = results[regions]
    return mask

def segment(image, raw=False):
    image = cv2.fastNlMeansDenoisingColored(image,None,10,10,7,21)
    if raw:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # import matplotlib.pyplot as plt
    # plt.imshow(image2)
    # plt.show()
    # from skimage.segmentation import mark_boundaries
    # from skimage.io import imsave
    regions = rg.to_regions(image, REGION_AREA, REGION_IGN)
    x = mark_boundaries(image, regions)
    # plt.subplot(2, 2, 3)
    # plt.imshow(x)
    # imsave('./cot1_out.jpg', x)
    # exit()
    mask = infer(model, enhance(image), regions)
    smooth(mask)
    return mask

def segment_one(image, depth, raw=False):
    # mask = np.array([[0,0,0,0,0,0,0,0],
    # [0,0,0,0,0,0,0,0],
    # [0,0,255,0,0,0,0,0],
    # [0,0,255,255,0,0,0,0],
    # [0,0,255,255,0,0,0,0],
    # [0,0,0,0,0,255,255,0],
    # [0,0,0,0,0,0,255,0],
    # [0,0,0,0,0,0,255,0]],dtype=np.uint8)

    # depth = np.array([[0,0,0,0,0,0,0,0],
    # [0,0,0,0,0,0,0,0],
    # [0,0,1,0,0,0,0,0],
    # [0,0,2,3,0,0,0,0],
    # [0,0,1,1,0,0,0,0],
    # [0,0,0,0,0,4,0,0],
    # [0,0,0,0,0,0,0,0],
    # [0,0,0,0,0,0,1,0]],dtype=np.uint8)
    mask = segment(image, raw=raw)
    fill(mask)
    n, labels = cv2.connectedComponents(mask)
    regions = regionprops(labels)
    one = np.array([depth[tuple(r.coords.T)].mean() for r in regions]).argmin()
    new_mask = np.zeros(mask.shape, dtype=np.uint8)
    new_mask[tuple(regions[one].coords.T)] = 255
    return new_mask

if __name__ == '__main__':
    # print(segment_one())
    # exit()
    args = parse_args()
    image = args.image
    output = args.output
    image = cv2.imread(image)
    # image = cv2.imread('cot3_out.jpg', 0)
    # _, image = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
    # fill(image)
    # print(get_regions(image))
    # exit()
    x = time.time()
    mask = segment(image, None, raw=True)
    y = time.time() - x
    print(y)

    img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img_hsv = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2HSV)
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
    # plt.subplot(2, 2, 4)
    # plt.imshow(mask, 'gray')
    # plt.show()

    cv2.imwrite(output, mask)
