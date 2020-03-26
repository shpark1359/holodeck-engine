
from ppo import TrackingController

tc = TrackingController()
tc.initialize(
    session_name="test3",
    num_slaves=8,
)
tc.loadNetworks(directory="./output/test2", network_type=None)

for i in range(100000):
    tc.runTraining(1)
