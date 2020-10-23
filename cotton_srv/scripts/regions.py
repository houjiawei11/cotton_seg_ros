import numpy as np
import cv2
import matplotlib.pyplot as plt
from skimage.io import imsave
from skimage.measure import regionprops
from skimage.segmentation import slic, mark_boundaries
import networkx as nx
import sys

class Region:
    def __init__(self, pts):
        self.area = len(pts)
        self.mean = np.median(pts, axis=0)
        # self.cov = np.cov(pts, rowvar=False)

def reg_nb(X, Y):
    a = np.sum((X.mean - Y.mean) ** 2)
    # b = np.trace(X.cov) + np.trace(Y.cov)
    # c = np.sum(np.sqrt(np.linalg.eig(np.dot(X.cov, Y.cov))[0]))
    # return a + b - 2 * c < 200
    return a < 500

def get_graph(regs):
    vs_right = np.vstack([regs[:,:-1].ravel(), regs[:,1:].ravel()])
    vs_below = np.vstack([regs[:-1,:].ravel(), regs[1:,:].ravel()])
    x = np.unique(np.hstack([vs_right, vs_below]).T, axis=0)
    x.sort(axis=1)
    x = np.unique(x, axis=0)
    x = x[x[:,0] != x[:,1]]
    G = nx.Graph()
    G.add_edges_from(x)
    return G

def to_regions(img, p=100, raw=False, ig=300):
    if raw:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    n_seg = img.shape[0] * img.shape[1] // p
    regs = slic(img, n_segments=n_seg, min_size_factor=0.1)
    # return regs
    G = get_graph(regs)
    props = regionprops(regs + 1)
    n = len(props)
    print(n)
    C = 1

    img = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
    rs = [Region(img[tuple(prop.coords.T)]) for prop in props]

    visit = np.zeros(n, dtype=bool)
    lbl = np.zeros(n, dtype=np.uint32)
    for i in range(n):
        if visit[i]:
            continue
        visit[i] = True
        if rs[i].area < ig:
            lbl[i] = 0
            continue
        nbs = G.neighbors(i)
        nbs = filter(lambda x: reg_nb(rs[i], rs[x]), nbs)
        nbs = list(nbs)
        lbl[i] = C
        j = 0
        while j < len(nbs):
            nb = nbs[j]
            if visit[nb]:
                j += 1
                continue
            visit[nb] = True
            if rs[nb].area < ig:
                lbl[nb] = 0
                j += 1
                continue
            lbl[nb] = C
            nbs2 = G.neighbors(nb)
            nbs2 = filter(lambda x: reg_nb(rs[nb], rs[x]), nbs2)
            nbs.extend(nbs2)
            j += 1
        C += 1
    regs = lbl[regs]
    print(C)
    return regs

def get_regions(img, regs, raw=True):
    if raw:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    if len(img.shape) > 2:
        img = img.reshape(-1, 3)
    else:
        img = img.ravel()
    regs = regs.ravel()
    mu = np.unique(regs)
    m = [[] for i in range(len(mu))]
    for r, i in zip(regs, img):
        m[r].append(i)
    m = [np.array(l) for l in m]
    return m

def get_indices(regs):
    return np.unique(regs, return_index=True)[1]

if __name__ == '__main__':
    img = cv2.imread(sys.argv[1])
    # img = cv2.GaussianBlur(img, (3, 3), 0)
    img = cv2.fastNlMeansDenoisingColored(img,None,10,10,7,21)
    imr = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    print('start')
    regs = to_regions(imr, 800, raw=True)
    print('end')
    bb = mark_boundaries(imr, regs)
    # bb = (bb*255).astype(np.uint8)
    # bb = cv2.cvtColor(bb, cv2.COLOR_RGB2BGR)
    imsave('./x.png', bb)
    # plt.imshow(bb)
    # plt.show()
