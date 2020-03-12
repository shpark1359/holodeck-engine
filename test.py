import numpy as np

import holodeck
from holodeck import agents
from holodeck.environments import *
from holodeck import sensors

def test_example():

    agent = holodeck.agents.AgentDefinition(agent_name="android",
                                            agent_type=holodeck.agents.AndroidAgent,
                                            sensors=[holodeck.sensors.JointRotationSensor],
                                            starting_loc=(0,0,.25),
                                            starting_rot=(0,0,0),
                                            is_main_agent=True
                                            )

    env = HolodeckEnvironment(agent_definitions=[agent], start_world=False)

    command = [30, 1.2, 0, 0]
    #command = [0, 0, 0, 0]

    for i in range(100):
        print(i)
        env.reset()
        for j in range(1000):
            # time.sleep(0.1)
            env.act("android", command)
            state = env.tick()
            print(state)

test_example()