#!/usr/bin/env python3

import sklearn.gaussian_process as gp
import os
import pickle
import numpy as np
import util
import time
import matplotlib.pyplot as plt

N = util.N
M = util.M
K = 2

#known_probabilities = [] #((p(C=1|x1,y1),p(C=2|x1,y1)),...,(p(C=1|xn,yn),p(C=2|xn,yn)))
#known_locaations = [] #[[x1,y1],[x2,y2],[x3,y3],...,[xn,yn]]

imageClassifier = pickle.load(open("Image_Classifier_Model.p", "rb"))

def new_image(image_feature, x, y):
	if os.path.exists("observations.p"):
		known_probabilities, known_locations = pickle.load(open("observations.p", "rb"))
	else:
		known_probabilities, known_locations = [], []

	P = imageClassifier.predict_proba(np.array([image_feature]))[0]
	if (x, y) in known_locations:
		index = known_locations.index((x, y))
		known_probabilities[index] = util.softmax(P+known_probabilities[index], axis=0)
	else:
		known_probabilities.append(P)
		known_locations.append((x, y))
	pickle.dump((known_probabilities, known_locations), open("observations.p", "wb"))

@util.timeit
def get_image_map():
	if not os.path.exists("observations.p"):
		return np.ones((N, M, K))/K

	while True:
		try:
		    known_probabilities, known_locations = pickle.load(open("observations.p", "rb"))
		    break
		except EOFError:
		    time.sleep(0.001)
	locations = np.array(np.meshgrid(np.arange(N), np.arange(M))).T.reshape((-1, 2))
	regressor = gp.GaussianProcessRegressor(gp.kernels.RBF(6.0) + gp.kernels.WhiteKernel(.01), optimizer=None)
	known_probabilities = (np.array(known_probabilities) - 0.5)*4
	regressor = regressor.fit(known_locations, known_probabilities)
	predictions = regressor.predict(locations).reshape((N, M, K))
	predictions += 0.5

	return util.softmax(predictions)


def setup():
	if os.path.exists("observations.p"):
		os.remove("observations.p")
	else:
		pass

