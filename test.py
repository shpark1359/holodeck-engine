import numpy as np

import holodeck
from holodeck import agents
from holodeck.environments import *
from holodeck import sensors
from IPython import embed

def test_example():

    agent = holodeck.agents.AgentDefinition(agent_name="android",
                                            agent_type=holodeck.agents.AndroidAgent,
                                            sensors=[holodeck.sensors.CustomSensor],
                                            starting_loc=(-1,0,.3),
                                            starting_rot=(0,0,0),
                                            is_main_agent=True
                                            )
    agent2 = holodeck.agents.AgentDefinition(agent_name="android2",
                                            agent_type=holodeck.agents.AndroidAgent,
                                            sensors=[holodeck.sensors.JointRotationSensor],
                                            starting_loc=(1,0,.3),
                                            starting_rot=(0,0,0),
                                            is_main_agent=True
                                            )

    env = HolodeckEnvironment(agent_definitions=[agent, agent2], start_world=False)

    command = [20000*500, 20*500, 0, 0]

    for i in range(100):
        env.reset()
        for j in range(1000):
            # time.sleep(0.1)
            env.act("android", command)
            state = env.tick()
            print(state["android"])

test_example()