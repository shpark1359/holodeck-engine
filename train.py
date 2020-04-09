
from ppo import TrackingController

tc = TrackingController()
tc.initialize(
    session_name="test13",
    num_slaves=64,
    tps=10000
)
# tc.loadNetworks(directory="./output/test11", network_type=None)

# TODO

for i in range(100000):
    tc.runTraining(1)
