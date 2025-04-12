import numpy as np

import OrientationFromAccelerometer as ofa

# def testOrientation():
rpestimator = ofa.RollPitchEstimator(9.794, 200)
rpestimator.add([1, -0.1, 9.8, 0.2])

rpestimator.add([2, 0, 9.8, 0])

rpestimator.add([3, 0.1, 9.8, -0.2])

bias = rpestimator.bias()
assert np.alltrue(bias == [2, 0, 9.8, 0])
rp = rpestimator.rollAndPitch()
# To manually verify this, define a world frame with xyz right, forward, up.
assert np.alltrue(rp == [90.0, 0])

rpestimator.reset()
rpestimator.add([1, 9.8, 0.1, 0.2])

rpestimator.add([2, 9.9, -0.1, 0.0])

rpestimator.add([3, 9.7, 0.0, -0.3])

bias = rpestimator.bias()
print('bias {}'.format(bias))
assert np.alltrue(bias == [2, 9.8, 0, 0.])
rp = rpestimator.rollAndPitch()
print('roll and pitch {}'.format(rp))
assert np.alltrue(rp == [0, -90.0])

