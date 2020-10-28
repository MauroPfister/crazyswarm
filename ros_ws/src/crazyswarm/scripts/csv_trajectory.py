#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import csv


def load_trajectories(pos_path):
    vel_path = pos_path.replace('pos', 'vel')
    acc_path = pos_path.replace('pos', 'acc')

    with open(pos_path, 'r') as f:
        read_data = csv.reader(f, delimiter=",")
        pos_cf = np.array(list(read_data)).astype(np.float32)
        n_cfs = pos_cf.shape[1] / 3  # Number of crazyflies
        pos_cf = np.hsplit(pos_cf, n_cfs)  # Split large array into list with pos for each cf

    with open(vel_path, 'r') as f:
        read_data = csv.reader(f, delimiter=",")
        vel_cf = np.array(list(read_data)).astype(np.float32)
        vel_cf = np.hsplit(vel_cf, n_cfs)  # Split large array into list with pos for each cf

    with open(acc_path, 'r') as f:
        read_data = csv.reader(f, delimiter=",")
        acc_cf = np.array(list(read_data)).astype(np.float32)
        acc_cf = np.hsplit(acc_cf, n_cfs)  # Split large array into list with pos for each cf

    return pos_cf, vel_cf, acc_cf


if __name__ == "__main__":
    """Load trajectory (pos, vel, acc) from csv files and track them using
    `cmdPosition` or `cmdFullState`."""

    pos_path = 'pos_cf.csv'
    pos_cf, vel_cf, acc_cf = load_trajectories(pos_path)
    cmd_mode = 'cmdPosition'
    Z = 0.3

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.setParam("stabilizer/controller", 2)  # Set controller
    allcfs.setParam("kalman/resetEstimation", 1)  # Reset kalman estimator

    # Takeoff
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    # Go to initial positions
    for i, cf in enumerate(allcfs.crazyflies):
        cf.goTo(np.array([pos_cf[i][0, 0], pos_cf[i][0, 1], 0.4]), 0, 2)
        timeHelper.sleep(2)
        cf.goTo(pos_cf[i][0], 0, 2)
        # timeHelper.sleep(2)
    timeHelper.sleep(2)

    # Track trajectory
    k_end = pos_cf[0].shape[0]
    for k in range(k_end - 20):
        for i, cf in enumerate(allcfs.crazyflies):
            pos = pos_cf[i][k]
            vel = vel_cf[i][k]
            acc = acc_cf[i][k]
            yaw = 0
            omega = np.zeros(3)  # NOTE: Setting to zero because we cannot calculate it

            if cmd_mode == 'cmdFullState':
                cf.cmdFullState(pos, vel, acc, yaw, omega)
            elif cmd_mode == 'cmdPosition':
                cf.cmdPosition(pos, yaw)
            else:
                raise Exception('Unknown mode')

        timeHelper.sleep(0.1)

    # Land
    for cf in allcfs.crazyflies:
        cf.notifySetpointsStop(remainValidMillisecs=100)
        cf.land(targetHeight=0.02, duration=4)
        # cf.goTo(cf.initialPosition + np.array([0, 0, Z]), 0, 2)

    timeHelper.sleep(2)

    # allcfs.land(targetHeight=0.02, duration=2.0+Z)
    # timeHelper.sleep(1.0+Z)
