from ppo import TrackingController

tc = TrackingController()
tc.initialize(session_name="play", num_slaves=1)
tc.loadNetworks(directory="./output/test2", network_type=None)

tc.play()
