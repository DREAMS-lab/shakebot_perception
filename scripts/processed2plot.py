#!/usr/bin/python3

import json
from math import sqrt
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.signal import butter, filtfilt
from datetime import timedelta


class processed2plot:
    def __init__(self):
        self.data = {}
    
    def extractData(self, data):
        for i in data.keys():
            if i == "acc":
                acc_plot = data[i]
            elif i == "pos":
                pos_plot = data[i]
            elif i == "vel_acc":
                vel_acc_plot = data[i]
            elif i == "vel_pos":
                vel_pos_plot = data[i]
            elif i == "vel_est":
                vel_est_plot = data[i]
                            
        return acc_plot, pos_plot, vel_acc_plot, vel_pos_plot, vel_est_plot
        
    
    def plot(self, data, labels):
        markerSize = 4**2
        xLimRight = 0.8
        gridStyle = "dashdot"
        saveFl = True
        plt.rc('font', size=12)
        global axs
        fig, axs = plt.subplots(3)
        count=0
        for idx, i, label in zip(range(len(data)),data,labels):
            if "velocity" in label:
                if count == 0:
                    axs[2].scatter([x[0] for x in i], [x[1] for x in i], label=r'$v_a$', marker="x", s=markerSize)
                    count+=1
                elif count == 1:
                    axs[2].scatter([x[0] for x in i], [x[1] for x in i], label=r'$v_d$', marker="o", s=markerSize)
                    count+=1
                else:
                    axs[2].plot([x[0] for x in i], [x[1] for x in i], label=r'$\hat{v}$', markersize=2)
                    count+=1
                axs[2].set(xlabel=r"time ($s$)", ylabel=r"$\hat{v} ~ (m/s)$")
                axs[2].set_xlim(right=xLimRight)
                axs[2].tick_params(direction="in")
            else:
                if "m/s^2" in label:
                    axs[idx].scatter([x[0] for x in i], [x[1] for x in i], label=label, marker="x", s=markerSize)
                    axs[idx].set_xticklabels([])
                    axs[idx].tick_params(direction="in")
                    axs[idx].set(ylabel=label)
                    axs[idx].grid(linestyle=gridStyle)
                    axs[idx].set_xlim(right=xLimRight)
                    
                else:
                    axs[idx].scatter([x[0] for x in i], [x[1]*100 for x in i], label=label, marker="x", s=markerSize)
                    axs[idx].set_xticklabels([])
                    axs[idx].tick_params(direction="in")
                    axs[idx].set(ylabel=label)
                    axs[idx].grid(linestyle=gridStyle)
                    axs[idx].set_xlim(right=xLimRight)
                    
                    
        plt.legend(loc="upper right", prop={"size":7})
        plt.grid(linestyle=gridStyle)
        if saveFl:
            plt.savefig('data\\results_imgs\\velocity_fusion.png', dpi=300)
        plt.show()
        
    def read_data(self, file):
        with open(file, "r") as f:
            data = json.load(f)
            f.close()
        return data
    
    def main(self):
        # self.fname = sorted(glob.glob("/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/data/processed/record*"))[-1]         # Use this for linux
        self.fname = sorted(glob.glob("data\\processed\\record*"))[-1]      # Use this for Windows
        # recorded_09_07_22_12_49_28
        # self.fname = "/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/data/processed/recorded_09_07_22_12_49_28.json"      # Use this for linux
        # self.fname = "data\\processed\\recorded_09_07_22_12_49_28.json"     # Use this for windows

        self.data = self.read_data(self.fname)
        # self.extractData(self.data)
        acc_plot, pos_plot, vel_acc_plot, vel_pos_plot, vel_est_plot = self.extractData(self.data)
        
        self.plot([acc_plot[:], pos_plot[:], vel_acc_plot[:], vel_pos_plot[:], vel_est_plot[:]], [r"$a~(m/s^2)$", r"$d~(cm)$", "velocity_acc", "velocity_pos", "velocity_estimate"])

if __name__=="__main__":
    # try:
    #     s = processed2plot()
    #     s.main()
    # except Exception as e:
    #     print(e)
        
    s = processed2plot()
    s.main()