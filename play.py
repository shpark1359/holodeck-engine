from ppo import TrackingController

tc = TrackingController()
tc.initialize(session_name="play", num_slaves=1, tps=500)
# tc.loadNetworks(directory="./output/test4", network_type=None)

tc.play()
