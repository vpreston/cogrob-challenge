import sklearn.gaussian_process as gp
import os
import pickle
import numpy as np
import util
import time
import matplotlib.pyplot as plt

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
    if os.path.exists("obs.p"):
        known_probabilities, known_locations = pickle.load(open("obs.p", "rb"))
    else:
        known_probabilities, known_locations = [], []

    if (x, y) in known_locations:
        index = known_locations.index((x, y))
        known_probabilities[index] = util.softmax(p+known_probabilities[index], axis=0)

    else:
        known_probabilities.append(p)
        known_locations.append((x, y))

    pickle.dump((known_probabilities, known_locations), open("obs.p", "wb"))


class GPRegressor:
    def __init__(self, a=0.5, b=2):
        self.a = a
        self.b = b
        if not os.path.exists("obs.p"):
            self.regressor = None
            self.known_probabilities = []

        else:
            while True:
                try:
                    self.known_probabilities, self.known_locations = pickle.load(open("obs.p", "rb"))
                    break

                except EOFError:
                    time.sleep(0.001)

            regressor = gp.GaussianProcessRegressor(gp.kernels.RBF([0.3, 0.3]) + gp.kernels.WhiteKernel(.01), optimizer=None)
            latent_values = (np.array(self.known_probabilities) - self.a)*self.b
            self.regressor = regressor.fit(self.known_locations, latent_values)

    def get_probability(self, x, y):
        if self.regressor is None:
            return np.ones(K)/K
        else:
            return util.softmax(self.regressor.predict(np.array([[x, y]])), axis=1)

    def __call__(self, x, y):
        return self.get_probability(x, y)[0]

    def visualize(self, c=0, minx=-5, miny=-5, maxx=5, maxy=5, granularity=100, file_path='/home/vpreston/Documents/misc/'):
        plt.clf()
        if self.regressor != None:
            xi = np.linspace(minx, maxx, granularity)
            yi = np.linspace(miny, maxy, granularity)
            locations =  np.array(np.meshgrid(xi, yi)).T.reshape((-1, 2))
            DEM = util.softmax(self.regressor.predict(locations), axis=1)[:,c]
            DEM = DEM.reshape((granularity,granularity))
            levels = np.arange(0, 1, 0.1)
            plt.contour(DEM, levels, linewidths=0.2,colors='k')
            plt.imshow(DEM,cmap ='RdYlGn_r',origin='lower', vmin=0, vmax=1)
            plt.colorbar()
            xs = np.array(self.known_locations)[:,0]
            ys = np.array(self.known_locations)[:,1]
            xArrayNormalized = xs/(maxx-minx)
            xArrayNormalized -= minx/(maxx-minx)*0.5
            yArrayNormalized = ys/(maxy-miny)
            yArrayNormalized -= miny/(maxx-minx)*0.5
            plt.xticks([])
            plt.yticks([])
            plt.title("Probability Map of Class {}".format(c))
            plt.scatter(granularity*yArrayNormalized, granularity*xArrayNormalized, color='k', alpha=0.25)
            if file_path:
                plt.savefig(file_path)
            else:
                plt.show()

def get_image_map(a=0.5, b=8):
    """
    Gets probability map
    :param a: float (0, 1) hyper parameter for regressor. should be left alone once optimal param found.
    :param b: float hyper parameter for regressor. should be left alone once optimal param found.
    :return: probability map np.array(size=<N,M,K>)
    """
    if not os.path.exists("obs.p"):
        return np.ones((N, M, K))/float(K)
    while True:
        try:
            known_probabilities, known_locations = pickle.load(open("obs.p", "rb"))
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
    print "CLEANING IMAGE PATH"
    if os.path.exists("obs.p"):
        os.remove("obs.p")
    else:
        pass

