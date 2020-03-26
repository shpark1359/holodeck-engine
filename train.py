
from ppo import TrackingController

tc = TrackingController()
tc.initialize(
    session_name="test2",
    num_slaves=8,
)

for i in range(100000):
    tc.runTraining(1)
