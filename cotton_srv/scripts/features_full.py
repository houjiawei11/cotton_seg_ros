import numpy as np
import cv2
from skimage.feature import local_binary_pattern
import sys
import regions as rg

ohta = np.array([
    [ 1/3, 1/3, 1/3 ],
    [ 1/2, 0., -1/2 ],
    [ -1/4, 1/2, -1/4 ]
])

def get_regions(img, p=100):
    return rg.to_regions(img, p)

def subsample_idx(sample_size, label):
    white = sample_size // 3
    black = sample_size - white
    ws = np.random.choice(np.where(label > 0)[0], size=white, replace=False)
    bs = np.random.choice(np.where(label == 0)[0], size=black, replace=False)
    return np.hstack((ws, bs))

# def create_binary_pattern(img, p, r):
#     #print ('[INFO] Computing local binary pattern features.')
#     lbp = local_binary_pattern(img, p, r)
#     return (lbp-np.min(lbp))/(np.max(lbp)-np.min(lbp)) * 255

def get_features_labels(img, label, raw=False, train=True, reshape=True):
    if raw:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    num_examples = 10000 # number of examples per image to use for training model

    feature_img = np.zeros((img.shape[0], img.shape[1], 4), dtype=np.uint32)
    # feature_img[:,:,:3] = img.dot(ohta.T)
    feature_img[:,:,:3] = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    feature_img[:,:,3] = local_binary_pattern(img_gray, 8, 2, method='nri_uniform')
    if reshape:
        features = feature_img.reshape(feature_img.shape[0]*feature_img.shape[1], feature_img.shape[2])
    else:
        features = feature_img
    if train == True:
        ss_idx = subsample_idx(num_examples, label.ravel())
        features = features[ss_idx]
    else:
        ss_idx = []
    if train == True:
        labels = label.ravel()[ss_idx]
    else:
        labels = None
    return features, labels

if __name__ == '__main__':
    img = cv2.imread(sys.argv[1])
    label = cv2.imread(sys.argv[1], 0)
    print('start')
    np.set_printoptions(threshold=sys.maxsize)
    regs = get_regions(img, 800)
    f = get_features(img, regs)
    l = get_labels(label, regs)
    print(f)
    print(l)
    print(f.shape, l.shape)
