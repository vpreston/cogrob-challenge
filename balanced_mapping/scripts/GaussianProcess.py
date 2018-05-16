import sklearn.gaussian_process as gp
import os
import pickle
import numpy as np
import util
import time

# THESE ARE THE WORLD PARAMETERS
N = util.N
M = util.M
K = util.K


def new_image(p, x, y):
    """
    updates the known information.
    :param p: [p(env_xy = c0), p(env_xy = c1), ... ,  p(env_xy = ck)]
    :param x: global x position of measurement
    :param y: global y position of measurement
    :return: None
    """
    if os.path.exists("observations.p"):
        known_probabilities, known_locations = pickle.load(open("observations.p", "rb"))
    else:
        known_probabilities, known_locations = [], []

    if (x, y) in known_locations:
        index = known_locations.index((x, y))
        known_probabilities[index] = util.softmax(p+known_probabilities[index], axis=0)

    else:
        known_probabilities.append(p)
        known_locations.append((x, y))

    pickle.dump((known_probabilities, known_locations), open("observations.p", "wb"))


class GPRegressor:
    def __init__(self, a=0.5, b=8):
        self.a = a
        self.b = b
        if not os.path.exists("observations.p"):
            self.regressor = None

        else:
            while True:
                try:
                    known_probabilities, known_locations = pickle.load(open("observations.p", "rb"))
                    break

                except EOFError:
                    time.sleep(0.001)

            regressor = gp.GaussianProcessRegressor(gp.kernels.RBF(6.0) + gp.kernels.WhiteKernel(.01), optimizer=None)
            latent_values = (np.array(known_probabilities) - self.a)*self.b
            self.regressor = regressor.fit(known_locations, latent_values)

    def get_probability(self, x, y):
        if self.regressor is None:
            return np.ones(K)/K
        else:
            return util.softmax(self.regressor.predict(np.array([[x, y]])), axis=1)

    def __call__(self, x, y):
        return self.get_probability(x, y)[0]

def get_image_map(a=0.5, b=8):
    """
    Gets probability map
    :param a: float (0, 1) hyper parameter for regressor. should be left alone once optimal param found.
    :param b: float hyper parameter for regressor. should be left alone once optimal param found.
    :return: probability map np.array(size=<N,M,K>)
    """
    if not os.path.exists("observations.p"):
        return np.ones((N, M, K))/float(K)
    while True:
        try:
            known_probabilities, known_locations = pickle.load(open("observations.p", "rb"))
            break

        except EOFError:
            time.sleep(0.001)

    locations = np.array(np.meshgrid(np.arange(N), np.arange(M))).T.reshape((-1, 2))
    regressor = gp.GaussianProcessRegressor(gp.kernels.RBF(6.0) + gp.kernels.WhiteKernel(.01), optimizer=None)
    latent_values = (np.array(known_probabilities) - a)*b
    regressor = regressor.fit(known_locations, latent_values)
    latent_predictions = regressor.predict(locations).reshape((N, M, K))
    predicted_probabilities = util.softmax(latent_predictions)
    return predicted_probabilities


def setup():
    """
    Wipes the known information. DO NOT FORGET TO RUN THIS!!
    :return: None
    """
    if os.path.exists("observations.p"):
        os.remove("observations.p")
    else:
        pass

