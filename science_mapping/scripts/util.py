import numpy as np
import time
import pickle
import cv2
import sklearn.gaussian_process as gp
from glob import glob

N = 31
M = 16

def softmax(x, axis=2):
    e_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
    return e_x / e_x.sum(axis=axis, keepdims=True)

def timeit(method):
    def timed(*args, **kw):
        ts = time.time()
        result = method(*args, **kw)
        te = time.time()
        if 'log_time' in kw:
            name = kw.get('log_name', method.__name__.upper())
            kw['log_time'][name] = int((te - ts) * 1000)
        else:
            print('%r  %2.2f ms' % \
                  (method.__name__, (te - ts) * 1000))
        return result
    return timed

def make_image_classifier_model_example():
    """
    This an example of how the image classifier will be made.
    We will need to get actual features from Image classification team.
    Creates "Image_Classifier_Model.p" using pickle dump ("wb")
    :return: None
    """

    FEATURE_EXTRACTOR = lambda image: [image[:, :, 0].mean(), image[:, :, 1].mean(), image[:, :, 2].mean()]
    redish_images = np.random.normal([100, 100, 160], [20, 20, 20], (200, 100, 100, 3))
    blueish_images = np.random.normal([160, 100, 100], [20, 20, 20], (200, 100, 100, 3))
    x = np.vstack(([FEATURE_EXTRACTOR(x) for x in redish_images], [FEATURE_EXTRACTOR(x) for x in blueish_images]))
    y = np.ones(400)
    y[:200] = 0 #red=0 blue=1

    GPC_images_classifier = gp.GaussianProcessClassifier(2.0 * gp.kernels.RBF([2.0, 1.0, 2.0]))
    GPC_images_classifier.fit(x, y)
    pickle.dump(GPC_images_classifier, open("Image_Classifier_Model.p", "wb"))

    MAP = np.zeros((N*100, M*100, 3))
    cmap = []
    for i in range(0,N):
        i = i+1 if i < M else N-i
        column = (M-i)*"b" + i*"r"
        cmap.append(column)


    for j, row in enumerate(cmap):
        for i, color in enumerate(row):
            position_x = j*100
            position_y = i*100
            color = [100, 100, 160] if color == "r" else [160, 100, 100]
            picture = np.random.normal(color, [20, 20, 20], (100, 100, 3))
            MAP[position_x:position_x+100, position_y:position_y+100] = picture
    cv2.imwrite("MAP.png", MAP)

make_image_classifier_model_example()