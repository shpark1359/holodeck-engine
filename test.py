import numpy as np

import holodeck
from holodeck import agents
from holodeck.environments import *
from holodeck import sensors
from IPython import embed

def test_example():

    agents = [holodeck.agents.AgentDefinition(agent_name="android"+str(i),
                                            agent_type=holodeck.agents.AndroidAgent,
                                            sensors=[holodeck.sensors.CustomSensor],
                                            starting_loc=(-1,0,.3),
                                            starting_rot=(0,0,0),
                                            is_main_agent=True
                                            ) for i in range(4)]
    # agent2 = holodeck.agents.AgentDefinition(agent_name="android2",
    #                                         agent_type=holodeck.agents.AndroidAgent,
    #                                         sensors=[holodeck.sensors.JointRotationSensor],
    #                                         starting_loc=(1,0,.3),
    #                                         starting_rot=(0,0,0),
    #                                         is_main_agent=True
    #                                         )

    env = HolodeckEnvironment(agent_definitions=agents, start_world=False)

    command = [0, 0, 30000*100, 30*100, 0, 0]

    for i in range(100):
        env.reset()
        for j in range(4000):
            if(j%500 == 0):
                command[0] = 1
                command[1] = j*0.002
            else:
                command[0] = 0
                command[1] = 0
            # time.sleep(0.1)
            for i in range(4):
                env.act("android"+str(i), command)
            state = env.tick()
            #print(state["android1"])

# test_example()

from ppo import TrackingController

tc = TrackingController()
tc.initialize(
    session_name="test",
    num_slaves=8,
)

for i in range(100000):
    tc.runTraining(1)
