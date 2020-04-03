
from ppo import TrackingController

tc = TrackingController()
tc.initialize(
    session_name="test7",
    num_slaves=64,
    tps=10000
)
# tc.loadNetworks(directory="./output/test4", network_type=None)

# TODO

for i in range(100000):
    tc.runTraining(1)
