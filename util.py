import math
import numpy as np
from IPython import embed
from contextlib import contextmanager
import time
import numpy as np

import matplotlib
import matplotlib.pyplot as plt

plt.ion()


def Plot(y_list, title, num_fig=1, path=None):
    plt.figure(num_fig, clear=True, figsize=(5.5, 4))
    plt.title(title)

    i = 0
    for y in y_list:
        plt.plot(y[0], label=y[1])
        i += 1

    plt.legend(loc=2)
    plt.show()
    plt.pause(0.001)
    if path is not None:
        plt.savefig(path, format="png")


class TimeChecker():
    def __init__(self):
        self._timeList = {}

    @contextmanager
    def check(self, key):
        if key not in self._timeList:
            self._timeList[key] = 0
        _st = time.time()
        yield
        self._timeList[key] += time.time() - _st

    def printAll(self):
        for k, v in self._timeList.items():
            print("{} : {}s".format(k, v))

    def clear(self):
        self._timeList = {}


class RunningMeanStd(object):
    # https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm
    def __init__(self, shape=()):
        self.mean = np.zeros(shape, 'float32')
        self.var = np.ones(shape, 'float32')
        self.count = 1e-4
        self.epsilon = 1e-8
        self.clip = 10

    def update(self, x):
        batch_mean = np.mean(x, axis=0)
        batch_var = np.var(x, axis=0)
        batch_count = x.shape[0]
        self.update_from_moments(batch_mean, batch_var, batch_count)

    def update_from_moments(self, batch_mean, batch_var, batch_count):
        self.mean, self.var, self.count = update_mean_var_count_from_moments(
            self.mean, self.var, self.count, batch_mean, batch_var, batch_count)

    def apply(self, x, update=True):
        if update:
            self.update(x)
        x = np.clip((x - self.mean) / np.sqrt(self.var + self.epsilon), -self.clip, self.clip)
        return x



def update_mean_var_count_from_moments(mean, var, count, batch_mean, batch_var, batch_count):
    delta = batch_mean - mean
    tot_count = count + batch_count

    new_mean = mean + delta * batch_count / tot_count
    m_a = var * count
    m_b = batch_var * batch_count
    M2 = m_a + m_b + np.square(delta) * count * batch_count / tot_count
    new_var = M2 / tot_count
    new_count = tot_count

    return new_mean, new_var, new_count