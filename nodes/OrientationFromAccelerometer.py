import numpy as np
import math

class AccelDataAccumulator(object):
    
    def __init__(self, gravityNorm, frequency=100) -> None:
        self.gravityNorm = gravityNorm
        self.frequency = 100
        self.timeWindow = int(frequency * 2)
        self.movingData = np.empty((0, 4), dtype=float)

    def add(self, txyz):
        if self.movingData.shape[0] > self.timeWindow:
            s = self.movingData.shape[0] - self.timeWindow
        else:
            s = 0
        self.movingData = np.vstack((self.movingData[s:, :], txyz))
    
    def reset(self):
        self.movingData = np.empty((0, 4), dtype=float)

    def stats(self):
        median = np.median(self.movingData, axis=0)
        std = np.std(self.movingData, axis=0)
        return median, std


class RollPitchEstimator(AccelDataAccumulator):
    def __init__(self, gravityNorm, frequency=100) -> None:
        super().__init__(gravityNorm, frequency)

    def bias(self, prior_bias = np.array([0, 0, 0])):
        median, std = super().stats()
        median[1:] = median[1:] - prior_bias
        return median

    def rollAndPitch(self, bias = np.array([0, 0, 0])):
        median, std = super().stats()
        accel = median[1:] - bias
        # eq 25 and 26 in https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
        roll = 180 * math.atan2(accel[1], accel[2])/math.pi
        pitch = 180 * math.atan2(-accel[0], math.sqrt(accel[1]*accel[1] + accel[2]*accel[2]))/math.pi
        return [roll, pitch]


