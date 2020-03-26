import os
from pathlib import Path
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import time
import random
import datetime

import numpy as np
import tensorflow as tf
from tensorflow.python import pywrap_tensorflow

import util

from replaybuffer import ReplayBuffer
from replaybuffer import Transition
from replaybuffer import Episode

from util import RunningMeanStd
from util import Plot
import holodeck
from holodeck.environments import *


from IPython import embed


policyLayerSize = 1024
policyLayerNumber = 4

valueLayerSize = 256
valueLayerNumber = 2

activationFunction = tf.nn.relu

class Policy:
    def __init__(self, action_size):
        self._scope = "policy"

        self.createModel(action_size)

    def createModel(self, action_size):
        # log std
        self._logstd = tf.Variable(
            initial_value=np.zeros(action_size),
            trainable=True,
            name='logstd',
            dtype=tf.float32
        )

        # std

        self._layers = [tf.keras.layers.Dense(policyLayerSize,
                                              activation=activationFunction,
                                              dtype=tf.float32) for _ in
                        range(policyLayerNumber)]
        self._layers.append(tf.keras.layers.Dense(action_size, dtype=tf.float32))

        self._mean = tf.keras.Sequential(self._layers)

    @tf.function
    def getActionAndNeglogprob(self, states):
        actions = self.action(states)
        return actions, self.neglogprob(actions, states)

    @tf.function
    def getMeanAction(self, states):
        return self._mean(states)

    @tf.function
    def std(self):
        return tf.exp(self._logstd)

    @property
    def logstd(self):
        return self._logstd

    @property
    def mean(self):
        return self._mean

    @tf.function
    def action(self, states):
        mean = self.mean(states)
        return mean + self.std() * tf.random.normal(tf.shape(mean))

    @tf.function
    def neglogprob(self, actions, states):
        return 0.5 * tf.math.reduce_sum(tf.math.square((actions - self._mean(states)) / self.std()),
                                        axis=-1) + 0.5 * tf.math.log(
            tf.constant(2.0 * 3.1415926535, dtype=tf.float32)) * tf.cast(tf.shape(actions), tf.float32)[
                   -1] + tf.math.reduce_sum(self._logstd, axis=-1)

    def trainable_variables(self):
        return self._mean.trainable_variables + [self._logstd]

    def build(self, state_size):
        self._mean.build((None, state_size))


class ValueFunction:
    def __init__(self):
        self._scope = "valueFunction"
        self.createModel()

    def createModel(self):
        self._layers = [tf.keras.layers.Dense(valueLayerSize,
                                              activation=activationFunction,
                                              dtype=tf.float32) for _ in
                        range(valueLayerNumber)]
        self._layers.append(tf.keras.layers.Dense(1, dtype=tf.float32))

        self._value = tf.keras.Sequential(self._layers)

    @property
    def value(self):
        return self._value

    @tf.function
    def getValue(self, states):
        return self.value(states)[:, 0]

    def trainable_variables(self):
        return self._value.trainable_variables

    def build(self, state_size):
        self._value.build((None, state_size))


class TrackingController:
    def __init__(self):
        random.seed(int(time.time()))
        np.random.seed(int(time.time()))

        self._startTime = time.time()
        self._summary_sim_time = 0
        self._summary_train_time = 0

        self._timeChecker = util.TimeChecker()

    def initialize(self,
                   session_name="default_session",
                   num_slaves=8,
                   use_evaluation=False
                   ):


        # get parameters from config
        self._numSlaves = num_slaves

        self._gamma = 0.99
        self._lambd = 0.95
        self._clipRange = 0.2

        self._learningRatePolicy = 1e-4
        self._learningRatePolicyDecay = 0.9993
        self._learningRateValueFunction = 1e-3

        self._batchSize = 1024
        self._transitionsPerIteration = 20000

        # if useEvaluation is true, evaluation of training progress is performed by evaluation function, else it is done by transitions collected in training session.
        self._useEvaluation = use_evaluation

        self._sessionName = session_name

        # initialize environment
        # TODO
        agents = [holodeck.agents.AgentDefinition(agent_name="android" + str(i),
                                                  agent_type=holodeck.agents.AndroidAgent,
                                                  sensors=[holodeck.sensors.CustomSensor],
                                                  starting_loc=(-1, 0, .25),
                                                  starting_rot=(0, 0, 0),
                                                  is_main_agent=True
                                                  ) for i in range(self._numSlaves)]
        self._env = HolodeckEnvironment(agent_definitions=agents, start_world=False)
        # self._env = holodeck.make("PPO")
        # self._env.should_render_viewport(False)

        self._stateSize = 18 * 3 + 5 * 3 + 5 * 3
        self._rewardSize = 5
        self._eoeSize  = 2
        self._actionSize = 18 * 3

        # initialize networks
        self._policy = Policy(self._actionSize)
        self._policy.build(self._stateSize)
        self._valueFunction = ValueFunction()
        self._valueFunction.build(self._stateSize)

        # initialize RunningMeanStd
        self._rms = RunningMeanStd(shape=(self._stateSize))

        # initialize replay buffer
        self._replayBuffer = ReplayBuffer()

        self._policyOptimizer = tf.keras.optimizers.Adam(learning_rate=self.decayedLearningRatePolicy)
        self._valueFunctionOptimizer = tf.keras.optimizers.Adam(learning_rate=self._learningRateValueFunction)

        # initialize saver
        # self._saver = tf.train.Saver(var_list=tf.trainable_variables(), max_to_keep=1)
        # save maximum step network
        self._smax = 0
        # save maximum reward network
        self._rmax = 0

        # initialize statistics variables
        # TODO
        self._summary_num_log = 0
        self._summary_num_episodes_total = 0
        self._summary_num_transitions_total = 0

        self._summary_max_episode_length = 0

        self._summary_total_rewards = []
        self._summary_total_rewards_by_parts = np.array([[]] * 5)
        self._summary_mean_rewards = []
        self._summary_transition_per_episodes = []
        self._summary_noise_records = []

        self._summary_evaluation_total_rewards = []
        self._summary_evaluation_total_rewards_by_parts = np.array([[]] * 5)
        self._summary_evaluation_mean_rewards = []
        self._summary_evaluation_transition_per_episodes = []

        # initialize checkpoint
        self._ckpt = tf.train.Checkpoint(
            policy_mean=self._policy.mean,
            policy_logstd=self._policy.logstd,
            valueFunction=self._valueFunction.value
            # policyOptimizer=self._policyOptimizer,
            # valueFunctionOptimizer=self._valueFunctionOptimizer
        )

        self._isNetworkLoaded = False
        self._loadedNetwork = ""

    def decayedLearningRatePolicy(self):
        return self._learningRatePolicy

    # load trained networks & rms
    def loadNetworks(self, directory, network_type=None):
        # load rms
        rms_dir = "{}/rms/".format(directory)
        if (network_type is None) or (network_type == ""):
            mean_dir = rms_dir + "mean.npy"
            var_dir = rms_dir + "var.npy"
        else:
            mean_dir = rms_dir + "mean_{}.npy".format(network_type)
            var_dir = rms_dir + "var_{}.npy".format(network_type)

        if os.path.exists(mean_dir):
            print("Loading RMS parameters")
            self._rms.mean = np.load(mean_dir)
            self._rms.var = np.load(var_dir)
            self._rms.count = 200000000

        # load netowrk
        if network_type is not None:
            network_dir = "{}/network-{}".format(directory, network_type)
        else:
            network_dir = "{}/network".format(directory)
        print("Loading networks from {}".format(network_dir))

        self.restore(network_dir)

        self._isNetworkLoaded = True
        self._loadedNetwork = "{}".format(network_dir)

    def computeTDAndGAE(self):
        self._collectedStates = [None] * self._summary_num_transitions_per_iteration
        self._collectedActions = [None] * self._summary_num_transitions_per_iteration
        self._collectedNeglogprobs = [None] * self._summary_num_transitions_per_iteration
        self._collectedTDs = [None] * self._summary_num_transitions_per_iteration
        self._collectedGAEs = [None] * self._summary_num_transitions_per_iteration

        startIdx = 0
        for epi in self._collectedEpisodes:
            data = epi.data
            size = len(data)

            # update max episorde length
            if size > self._summary_max_episode_length:
                self._summary_max_episode_length = size

            states, actions, rewards, values, neglogprobs, TDs, GAEs = zip(*data)
            values = tf.convert_to_tensor(values).numpy()
            values = np.concatenate((values, [0]), axis=0)
            advantages = np.zeros(size)
            ad_t = 0

            for i in reversed(range(size)):
                delta = rewards[i] + values[i + 1] * self._gamma - values[i]
                ad_t = delta + self._gamma * self._lambd * ad_t
                advantages[i] = ad_t

            TD = values[:size] + advantages
            self._collectedStates[startIdx:startIdx + size] = list(states)
            self._collectedActions[startIdx:startIdx + size] = list(actions)
            self._collectedNeglogprobs[startIdx:startIdx + size] = list(neglogprobs)
            self._collectedTDs[startIdx:startIdx + size] = list(TD)
            self._collectedGAEs[startIdx:startIdx + size] = list(advantages)

            startIdx += size

        self._collectedStates = np.array(self._collectedStates, dtype=np.float32)
        self._collectedActions = tf.convert_to_tensor(self._collectedActions).numpy()
        self._collectedNeglogprobs = tf.convert_to_tensor(self._collectedNeglogprobs).numpy()
        self._collectedTDs = np.array(self._collectedTDs, dtype=np.float32)
        self._collectedGAEs = np.array(self._collectedGAEs, dtype=np.float32)

    def optimize(self):
        self.computeTDAndGAE()
        if len(self._collectedStates) < self._batchSize:
            return

        GAE = np.array(self._collectedGAEs)
        GAE = (GAE - GAE.mean()) / (GAE.std() + 1e-5)

        ind = np.arange(len(GAE))

        np.random.shuffle(ind)

        for s in range(int(len(ind) // self._batchSize)):
            selectedIndex = ind[s * self._batchSize:(s + 1) * self._batchSize]

            selectedStates = tf.convert_to_tensor(self._collectedStates[selectedIndex])
            selectedActions = tf.convert_to_tensor(self._collectedActions[selectedIndex])
            selectedNeglogprobs = tf.convert_to_tensor(self._collectedNeglogprobs[selectedIndex])
            selectedTDs = tf.convert_to_tensor(self._collectedTDs[selectedIndex])
            selectedGAEs = tf.convert_to_tensor(GAE[selectedIndex])

            self.optimizeStep(selectedActions, selectedStates, selectedNeglogprobs, selectedTDs, selectedGAEs)

    def optimizeStep(self, a, s, nl, td, gae):
        with tf.GradientTape() as tape:
            curNeglogprob = self._policy.neglogprob(a, s)
            ratio = tf.exp(nl - curNeglogprob)
            clippedRatio = tf.clip_by_value(ratio, 1.0 - self._clipRange, 1.0 + self._clipRange)
            policyLoss = -tf.reduce_mean(tf.minimum(ratio * gae, clippedRatio * gae))

        gradients = tape.gradient(policyLoss, self._policy.trainable_variables())
        gradients, _grad_norm = tf.clip_by_global_norm(gradients, 0.5)
        self._policyOptimizer.apply_gradients(zip(gradients, self._policy.trainable_variables()))

        # optimize value function
        with tf.GradientTape() as tape:
            valueLoss = tf.reduce_mean(tf.square(self._valueFunction.getValue(s) - td))
        gradients = tape.gradient(valueLoss, self._valueFunction._value.trainable_variables)
        gradients, _grad_norm = tf.clip_by_global_norm(gradients, 0.5)
        self._valueFunctionOptimizer.apply_gradients(zip(gradients, self._valueFunction._value.trainable_variables))

    def reset(self):
        return

    def act(self, index, action):
        self._env.act("android"+str(index), action)


    def step(self, actions):
        for _ in range(10):
            for i in range(self._numSlaves):
                self.act(i, actions[i])
            res = self._env.tick()

        states = []
        rewards = []
        eoes = []
        for i in range(self._numSlaves):
            s = res["android"+str(i)]["CustomSensor"]
            states.append(s[:self._stateSize])
            rewards.append(s[self._stateSize:self._stateSize+self._rewardSize])
            eoes.append(s[self._stateSize+self._rewardSize:])

        return states, rewards, eoes

    def runTraining(self, num_iteration=1):
        # create logging directory
        if not os.path.exists("output/"):
            os.mkdir("output/")
        self._directory = 'output/' + self._sessionName + '/'

        if not os.path.exists(self._directory):
            os.mkdir(self._directory)

        directory = self._directory + "rms/"
        if not os.path.exists(directory):
            os.mkdir(directory)

        directory = directory + "cur/"
        if not os.path.exists(directory):
            os.mkdir(directory)

        self.printParameters()

        while True:
            print("\nTraining start")
            self._summary_num_episodes_per_epoch = 0
            self._summary_num_transitions_per_epoch = 0
            self._summary_reward_per_epoch = 0
            self._summary_reward_by_part_per_epoch = []
            self._summary_max_episode_length = 0

            for it in range(num_iteration):
                self._summary_sim_time -= time.time()
                self._collectedEpisodes = []

                nan_count = 0

                # TODO : implement reset
                actions = [None] * self._numSlaves
                for i in range(self._numSlaves):
                    actions[i] = [1, random.random()]
                next_states, _, _ = self.step(actions)

                rewards = [None] * self._numSlaves
                episodes = [None] * self._numSlaves

                terminated = [False] * self._numSlaves
                resetRequired = [False] * self._numSlaves

                for j in range(self._numSlaves):
                    episodes[j] = Episode()

                self._summary_num_transitions_per_iteration = 0
                last_print = 0
                while True:
                    # get states
                    states = np.array(next_states)
                    states_for_update = states[~np.array(terminated)]
                    states_for_update = self._rms.apply(states_for_update)
                    states[~np.array(terminated)] = states_for_update

                    # set action
                    actions, logprobs = self._policy.getActionAndNeglogprob(states)
                    values = self._valueFunction.getValue(states)

                    action_with_reset_signal = [None] * self._numSlaves
                    for j in range(self._numSlaves):
                        action_with_reset_signal[j] = [0, 0] + actions[j].numpy().tolist()
                        if resetRequired[j]:
                            action_with_reset_signal[j][0] = 1
                            action_with_reset_signal[j][1] = random.random()

                    # run one step
                    next_states, r, e = self.step(action_with_reset_signal)

                    for j in range(self._numSlaves):
                        if terminated[j]:
                            continue

                        is_terminal = e[j][0] > 0.5 and True or False
                        nan_occur = e[j][1] > 0.5 and True or False
                        # push tuples only if nan did not occur
                        if nan_occur is not True:
                            if resetRequired[j]:
                                resetRequired[j] = False
                            else:
                                rewards[j] = r[j][0]
                                self._summary_reward_per_epoch += rewards[j]
                                self._summary_reward_by_part_per_epoch.append(r[j])
                                episodes[j].push(states[j], actions[j], rewards[j], values[j], logprobs[j])
                                self._summary_num_transitions_per_iteration += 1
                        else:
                            nan_count += 1

                        # if episode is terminated
                        if is_terminal:
                            # push episodes
                            if len(episodes[j].data) != 0:
                                self._collectedEpisodes.append(episodes[j])

                            if self._summary_num_transitions_per_iteration < self._transitionsPerIteration:
                                episodes[j] = Episode()
                                resetRequired[j] = True
                            else:
                                terminated[j] = True

                    # if local step exceeds t_p_i: wait for others to terminate
                    if self._summary_num_transitions_per_iteration >= self._transitionsPerIteration:
                        if all(t is True for t in terminated):
                            print('\r{}/{} : {}/{}'.format(it + 1, num_iteration,
                                                         self._summary_num_transitions_per_iteration,
                                                         self._transitionsPerIteration), end='')
                            break

                    # print progress per 100 steps
                    if last_print + 100 < self._summary_num_transitions_per_iteration:
                        print('\r{}/{} : {}/{}'.format(it + 1, num_iteration, self._summary_num_transitions_per_iteration,
                                                     self._transitionsPerIteration), end='')
                        last_print = self._summary_num_transitions_per_iteration

                self._summary_sim_time += time.time()
                self._summary_train_time -= time.time()

                # optimization
                print('')
                if (nan_count > 0):
                    print("nan_count : {}".format(nan_count))

                self._summary_num_episodes_per_epoch += len(self._collectedEpisodes)
                self._summary_num_transitions_per_epoch += self._summary_num_transitions_per_iteration

                self.optimize()  ##SM) after getting all tuples, optimize once

                self._summary_train_time += time.time()

            # decay learning rate
            if self._learningRatePolicy > 1e-5:
                self._learningRatePolicy = self._learningRatePolicy * self._learningRatePolicyDecay

            print('Training end\n')

            self._summary_total_rewards.append(self._summary_reward_per_epoch / self._summary_num_episodes_per_epoch)
            self._summary_total_rewards_by_parts = np.insert(self._summary_total_rewards_by_parts,
                                                             self._summary_total_rewards_by_parts.shape[1],
                                                             np.asarray(self._summary_reward_by_part_per_epoch).sum(
                                                                 axis=0) / self._summary_num_episodes_per_epoch, axis=1)
            self._summary_mean_rewards.append(np.asarray(self._summary_total_rewards)[-10:].mean())
            self._summary_noise_records.append(self._policy.std().numpy().mean())

            self._summary_num_episodes_total += self._summary_num_episodes_per_epoch
            self._summary_num_transitions_total += self._summary_num_transitions_per_epoch
            t_per_e = 0
            if self._summary_num_episodes_per_epoch is not 0:
                t_per_e = self._summary_num_transitions_per_epoch / self._summary_num_episodes_per_epoch
            self._summary_transition_per_episodes.append(t_per_e)

            # print summary
            self.printSummary()


    def play(self):
        # create logging directory
        if not os.path.exists("output/"):
            os.mkdir("output/")
        self._directory = 'output/' + self._sessionName + '/'

        if not os.path.exists(self._directory):
            os.mkdir(self._directory)

        directory = self._directory + "rms/"
        if not os.path.exists(directory):
            os.mkdir(directory)

        directory = directory + "cur/"
        if not os.path.exists(directory):
            os.mkdir(directory)
        self.printParameters()

        actions = [None] * self._numSlaves
        for i in range(self._numSlaves):
            actions[i] = [1, 0.0]
        next_states, _, _ = self.step(actions)

        rewards = [None] * self._numSlaves
        episodes = [None] * self._numSlaves

        terminated = [False] * self._numSlaves
        resetRequired = [False] * self._numSlaves

        last_print = 0
        while True:
            # get states
            states = np.array(next_states)
            states_for_update = states[~np.array(terminated)]
            states_for_update = self._rms.apply(states_for_update)
            states[~np.array(terminated)] = states_for_update

            # set action
            if self._isNetworkLoaded:
                # actions, _ = self._policy.getActionAndNeglogprob(states)
                actions = self._policy.getMeanAction(states)
            else:
                actions = np.zeros(shape=(self._numSlaves, self._actionSize))

            action_with_reset_signal = [None] * self._numSlaves
            for j in range(self._numSlaves):
                action_with_reset_signal[j] = [0, 0] + np.array(actions[j]).tolist()
                if resetRequired[j]:
                    action_with_reset_signal[j][0] = 1
                    action_with_reset_signal[j][1] = random.random()

            # run one step
            next_states, r, e = self.step(action_with_reset_signal)

            for j in range(self._numSlaves):

                is_terminal = e[j][0] > 0.5 and True or False
                nan_occur = e[j][1] > 0.5 and True or False
                # push tuples only if nan did not occur
                if nan_occur is not True:
                    if resetRequired[j]:
                        resetRequired[j] = False

                # if episode is terminated
                if is_terminal:
                    resetRequired[j] = True


        # optimization
        print('')

    def printParameters(self):
        # print on shell
        print("===============================================================")
        print(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        print("Elapsed time         : {:.2f}s".format(time.time() - self._startTime))
        print("Session Name         : {}".format(self._sessionName))
        print("Slaves number        : {}".format(self._numSlaves))
        print("State size           : {}".format(self._stateSize))
        print("Action size          : {}".format(self._actionSize))
        print("Learning rate        : {:.6f}".format(self._learningRatePolicy))
        print("Gamma                : {}".format(self._gamma))
        print("Lambda               : {}".format(self._lambd))
        print("Batch size           : {}".format(self._batchSize))
        print("Transitions per iter : {}".format(self._transitionsPerIteration))
        print("PPO clip range       : {}".format(self._clipRange))
        print("Loaded netowrks      : {}".format(self._loadedNetwork))
        print("===============================================================")

        # print to file
        out = open(self._directory + "parameters", "w")
        out.write(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S\n"))
        out.write("Session Name         : {}\n".format(self._sessionName))
        out.write("Slaves number        : {}\n".format(self._numSlaves))
        out.write("State size           : {}\n".format(self._stateSize))
        out.write("Action size          : {}\n".format(self._actionSize))
        out.write("Learning rate        : {:.6f}\n".format(self._learningRatePolicy))
        out.write("Gamma                : {}\n".format(self._gamma))
        out.write("Lambda               : {}\n".format(self._lambd))
        out.write("Batch size           : {}\n".format(self._batchSize))
        out.write("Transitions per iter : {}\n".format(self._transitionsPerIteration))
        out.write("PPO clip range       : {}\n".format(self._clipRange))
        out.write("Loaded netowrks      : {}\n".format(self._loadedNetwork))
        out.close()

        # pre make results file
        out = open(self._directory + "results", "w")
        out.close()

    def printSummary(self):
        np.save(self._directory + "rms/mean.npy".format(self._summary_num_log), self._rms.mean)
        np.save(self._directory + "rms/var.npy".format(self._summary_num_log), self._rms.var)

        print('===============================================================')
        print(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        print("Elapsed time         : {:.2f}s".format(time.time() - self._startTime))
        print("Simulation time      : {}s".format(self._summary_sim_time))
        print("Training time        : {}s".format(self._summary_train_time))
        print("Session Name         : {}".format(self._sessionName))
        print("Logging Count        : {}".format(self._summary_num_log))
        print('Noise                : {:.3f}'.format(self._summary_noise_records[-1]))
        print('Learning rate        : {:.6f}'.format(self._learningRatePolicy))
        print('Total episode        : {}'.format(self._summary_num_episodes_total))
        print('Total trans          : {}'.format(self._summary_num_transitions_total))
        total_t_per_e = 0
        if self._summary_num_episodes_total is not 0:
            total_t_per_e = self._summary_num_transitions_total / self._summary_num_episodes_total
        print('Total trans per epi  : {:.2f}'.format(total_t_per_e))
        print('Episode              : {}'.format(self._summary_num_episodes_per_epoch))
        print('Transition           : {}'.format(self._summary_num_transitions_per_epoch))
        print('Trans per epi        : {:.2f}'.format(self._summary_transition_per_episodes[-1]))
        print('Max episode length   : {}'.format(self._summary_max_episode_length))
        print('Rewards per episodes : {:.2f}'.format(self._summary_total_rewards[-1]))

        print('===============================================================')

        # print plot
        y_list = [[np.asarray(self._summary_total_rewards_by_parts[0]), 'r'],
                  [np.asarray(self._summary_mean_rewards), 'r_mean'],
                  [np.asarray(self._summary_transition_per_episodes), 'steps'],
                  [np.asarray(self._summary_total_rewards_by_parts[1]), 'p'],
                  [np.asarray(self._summary_total_rewards_by_parts[2]), 'v'],
                  [np.asarray(self._summary_total_rewards_by_parts[3]), 'com'],
                  [np.asarray(self._summary_total_rewards_by_parts[4]), 'ee']]
        Plot(y_list, self._sessionName, 1, path=self._directory + "result.png")

        for i in range(len(y_list)):
            y_list[i][0] = np.array(y_list[i][0]) / np.array(self._summary_transition_per_episodes)
        y_list[1][0] = np.asarray(self._summary_noise_records)
        y_list[1][1] = 'noise'

        Plot(y_list, self._sessionName + "_per_step", 2, path=self._directory + "result_per_step.png")

        # log to file
        out = open(self._directory + "results", "a")
        out.write('===============================================================\n')
        out.write(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S\n"))
        out.write("Elapsed time         : {:.2f}s\n".format(time.time() - self._startTime))
        out.write("Simulation time      : {}s\n".format(self._summary_sim_time))
        out.write("Training time        : {}s\n".format(self._summary_train_time))
        out.write("Session Name         : {}\n".format(self._sessionName))
        out.write("Logging Count        : {}\n".format(self._summary_num_log))
        out.write('Noise                : {:.3f}\n'.format(self._summary_noise_records[-1]))
        out.write('Learning rate        : {:.6f}\n'.format(self._learningRatePolicy))
        out.write('Total episode        : {}\n'.format(self._summary_num_episodes_total))
        out.write('Total trans          : {}\n'.format(self._summary_num_transitions_total))
        out.write('Total trans per epi  : {:.2f}\n'.format(total_t_per_e))
        out.write('Episode              : {}\n'.format(self._summary_num_episodes_per_epoch))
        out.write('Transition           : {}\n'.format(self._summary_num_transitions_per_epoch))
        out.write('Trans per epi        : {:.2f}\n'.format(self._summary_transition_per_episodes[-1]))
        out.write('Max episode length   : {}\n'.format(self._summary_max_episode_length))
        out.write('Rewards per episodes : {:.2f}\n'.format(self._summary_total_rewards[-1]))

        out.write('===============================================================\n')
        out.close()

        # save network
        self.save(self._directory + "network")

        t_per_e = self._summary_transition_per_episodes[-1]
        tr = self._summary_total_rewards[-1]

        if t_per_e > self._smax:
            self._smax = t_per_e
            np.save(self._directory + "rms/mean_smax.npy", self._rms.mean)
            np.save(self._directory + "rms/var_smax.npy", self._rms.var)

            os.system(str(Path("copy {}/network.data-00000-of-00001 {}/network-smax.data-00000-of-00001".format(self._directory,
                                                                                                     self._directory))))
            os.system(str(Path("copy {}/network.data-00000-of-00002 {}/network-smax.data-00000-of-00002".format(self._directory,
                                                                                                     self._directory))))
            os.system(str(Path("copy {}/network.data-00001-of-00002 {}/network-smax.data-00001-of-00002".format(self._directory,
                                                                                                     self._directory))))
            os.system(str(Path("copy {}/network.index {}/network-smax.index".format(self._directory, self._directory))))

        if tr > self._rmax:
            self._rmax = tr
            np.save(self._directory + "rms/mean_rmax.npy", self._rms.mean)
            np.save(self._directory + "rms/var_rmax.npy", self._rms.var)

            os.system(str(Path("copy {}/network.data-00000-of-00001 {}/network-rmax.data-00000-of-00001".format(self._directory,
                                                                                                     self._directory))))
            os.system(str(Path("copy {}/network.data-00000-of-00002 {}/network-rmax.data-00000-of-00002".format(self._directory,
                                                                                                     self._directory))))
            os.system(str(Path("copy {}/network.data-00001-of-00002 {}/network-rmax.data-00001-of-00002".format(self._directory,
                                                                                                     self._directory))))
            os.system(str(Path("copy {}/network.index {}/network-rmax.index".format(self._directory, self._directory))))

        self._summary_num_log = self._summary_num_log + 1

        return

    def save(self, path):
        self._ckpt.write(path)

    def restore(self, path):
        self._ckpt.restore(path)
