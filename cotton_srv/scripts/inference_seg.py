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

REGION_AREA = 400
REGION_IGN = 100
INFER_IGN = 100
RAND_SIZE = 30
RATIO = 0.7

def check_args(args):
    if not os.path.exists(args.image):
        raise ValueError("Image file does not exist")
    return args

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--image" , help="Path to image", required=True)
    parser.add_argument("-m", "--model", help="Path to model", required=True)
    parser.add_argument("-o", "--output", help="Path to output image", required=True)
    args = parser.parse_args()
    return check_args(args)

def feature_and_infer(model, image, regions):
    p = time.time()
    rs = regionprops(regions + 1)
    f, _ = ft.get_features_labels(image, None, train=False, reshape=False)
    results = np.zeros(len(rs), dtype=np.uint8)
    e = enumerate(rs)
    next(e)
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
        X = f[tuple(choices.T)]
        # print(X.shape)
        y = model.predict(X)
        t = np.count_nonzero(y)
        if t / pop > RATIO:
            results[i] = 255
        else:
            results[i] = 0
    p2 = time.time()
    print(p2 - p)
    mask = results[regions]
    return mask

def segment(image, model, raw=False):
    if raw:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # image = cv2.fastNlMeansDenoisingColored(image,None,10,10,7,21)
    p = time.time()
    regions = rg.to_regions(image, REGION_AREA, REGION_IGN)
    p2 = time.time()
    print(p2-p)
    mask = feature_and_infer(model, image, regions)
    return mask

def segment_main(image, raw=False):
    model = pkl.load(open("model.p", "rb"))
    return segment(image, model, raw=False)

# https://stackoverflow.com/questions/50450654/filling-in-circles-in-opencv
def fill(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(mask, contours, -1, 255, thickness=-1)

def get_regions(mask):
    n, labels = cv2.connectedComponents(mask)
    regions = regionprops(labels)
    return [r.coords for r in regions]

if __name__ == '__main__':
    args = parse_args()
    image = args.image
    model = args.model
    output = args.output
    image = cv2.imread(image)
    # image = cv2.imread('cot3_out.jpg', 0)
    # _, image = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
    # fill(image)
    # print(get_regions(image))
    # exit()
    model = pkl.load(open(model, "rb"))
    mask = segment(image, model, raw=True)
    cv2.imwrite(output, mask)
