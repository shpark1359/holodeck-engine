
from ppo import TrackingController

tc = TrackingController()
tc.initialize(
    session_name="test10",
    num_slaves=64,
    tps=10000
)
# tc.loadNetworks(directory="./output/test8", network_type=None)

# TODO

for i in range(100000):
    tc.runTraining(1)
