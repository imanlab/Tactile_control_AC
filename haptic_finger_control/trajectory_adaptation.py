#! /usr/bin/env python3
from argparse import _ActionsContainer
from os import device_encoding
import rospy
import time
import pickle
import datetime
import numpy as np
import pandas as pd
import message_filters
from pickle import load
import numpy.matlib as mat
from numpy.random import seed
from pykalman import KalmanFilter
# from xela_server.msg import XStream
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Float64
from scipy.spatial.transform import Rotation as R
from python.CppPythonSocket.server import Server

## Optimizer
from scipy.optimize import Bounds
from scipy.spatial import distance
from scipy.optimize import minimize
from scipy.optimize import BFGS
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint

## Pytorch
import torch
import torch.onnx
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

import onnx
import onnxruntime

from ACTP.ACTP_model import ACTP
# from ClassifierLSTM.classifier_model import ClassifierLSTM
from ClassifierLSTM.seq_classifier_lstm import ClassifierLSTM

from scipy.special import expit
from scipy.misc import derivative


seed = 42
context_frames = 10
sequence_length = 20
torch.manual_seed(seed)
torch.backends.cudnn.benchmark = False
torch.backends.cudnn.deterministic = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class MPCtracker():
    def __init__(self):
		self.stop = 0.0
		self.time_step = 0
		self.prev_time_step = 0
		self.slip_flag = False
		self.translation_x = 0.35
		self.translation_y = 0.46
		# self.previous_w = np.random.normal(0, 0.1, 5)
		self.robot_data 	      = np.zeros((1000, 6))
		self.action_data 	      = np.zeros((1000, 6))
		self.robot_data_scaled 	  = np.zeros((1000, 6))
		self.action_data_scaled   = np.zeros((1000, 6))
		self.predicted_slip_class = np.zeros((1000, 10, 1))
		self.tactile_predictions  = np.zeros((1000, 10, 48))
		self.tactile_predictions_descaled = np.zeros((1000, 10, 48))

		self.optimal_weights      = np.zeros((700, 18))
		self.ref_spherical_trajectory = np.zeros((700, 10, 6))
		self.spherical_trajectory = np.zeros((700, 10, 6))
		self.optimal_trajectory   = np.zeros((700, 10, 6))
		self.opt_execution_time   = np.zeros(700)
		self.optimality           = np.zeros(700)
		self.num_itr              = np.zeros(700)
		self.constr_violation     = np.zeros(700)