#!/usr/bin/python3

import json
import os
import glob
import numpy as np
import ast
import matplotlib.pyplot as plt
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.signal import butter, filtfilt
from datetime import timedelta


class velocity_calc:
    def __init__(self):
        self.data = {}
        
    def format_pd(self, df):
        df = df.transpose()
        df.columns = df.iloc[0]
        df = df[1:]
        df["timestamp"] = df.index
        df.set_index(np.arange(1, df.shape[0] + 1), inplace=True)
        return df
    
    def extractData(self, var, data):
        ex_data=[]
        if var == "posx":
            baseTime = 0
            for tstamp, value in data.items():
                if "pose" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["pose"]["position"]["x"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["pose"]["position"]["x"]])
        
        elif var == "posy":
            baseTime = 0
            for tstamp, value in data.items():
                if "pose" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["pose"]["position"]["y"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["pose"]["position"]["y"]])
                    
        elif var == "posz":
            baseTime = 0
            for tstamp, value in data.items():
                if "pose" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["pose"]["position"]["z"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["pose"]["position"]["z"]])
    
        elif var == "accx":
            baseTime = 0
            for tstamp, value in data.items():
                if "acceleration" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["acceleration"]["x"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["acceleration"]["x"]])
                    
        elif var == "accy":
            baseTime = 0
            for tstamp, value in data.items():
                if "acceleration" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["acceleration"]["y"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["acceleration"]["y"]])
                
        elif var == "accz":
            baseTime = 0
            for tstamp, value in data.items():
                if "acceleration" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["acceleration"]["z"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["acceleration"]["z"]])
                
        return ex_data
    
    def plot(self, data, labels):
        fig, axs = plt.subplots(len(data)-1)
        count=0
        for idx, i, label in zip(range(len(data)),data,labels):
            if "velocity" in label:
                if count == 0:
                    axs[2].scatter([x[0] for x in i], [x[1] for x in i], label="velocity_acc", marker="x")
                    count+=1
                else:
                    axs[2].scatter([x[0] for x in i], [x[1] for x in i], label="velocity_pos", marker="x")
                axs[2].set(xlabel="time (s)", ylabel="velocity")
            else:
                axs[idx].scatter([x[0] for x in i], [x[1] for x in i], label=label, marker="x")
                axs[idx].set(xlabel="time (s)", ylabel=label)
        plt.legend()
        plt.show()
        
    def read_data(self, file):
        with open(file, "r") as f:
            data = json.load(f)
            f.close()
        return data
    
    def get_velocity_facc(self, tstamp, filtered_acc):
        f = InterpolatedUnivariateSpline(tstamp, filtered_acc, k=1)  # k=1 gives linear interpolation
        vel = [f.integral(tstamp[0], tstamp[i]) for i in range(len(tstamp))]
        return vel
    
    def get_velocity_fpos(self, tstamp, filtered_disp):
        f = InterpolatedUnivariateSpline(tstamp, filtered_disp, k=1)  # k=1 gives linear interpolation
        fp = f.derivative()
        vel = [fp(i) for i in range(len(tstamp))]
        return vel
                
    def butter_lowpass_filter(self, sample_data, cutoff, fs, order, n):
        data = sample_data[:n]
        nyq = 0.5 * fs  # Nyquist Frequency:
        normal_cutoff = cutoff / nyq
        # Get the filter coefficients 
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data)
        return y
    
    def main(self):
        fname = sorted(glob.glob("/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/scripts/record*"))[-1]
        self.data = self.read_data(fname)
        posx = self.extractData("posx", self.data)
        # posx = sorted(posx, key=lambda x:x[0])
        # posy = self.extractData("posy", self.data)
        # posz = self.extractData("posz", self.data)
        # accx = self.extractData("accx", self.data)
        accy = self.extractData("accy", self.data)
        # accy = sorted(accy, key=lambda x:x[0])
        # accz = self.extractData("accz", self.data)
        
        # displacement processing to get velocity
        pos_tstamp = [x[0] for x in accy]
        pos_tbf = [x[1] for x in accy]
        vel_pos = self.get_velocity_fpos(pos_tstamp, pos_tbf)
        vel_pos_plot = [[i,j] for i,j in zip(pos_tstamp, vel_pos)]
        
        # acceleration processing to get velocity
        T = [x[0] for x in accy][-1]        # Sample period
        fs = 80           # Accelerometer sampling rate, Hz
        cutoff = 30       # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
        order = 2         # butterworth filter order
        n = int(T * fs)   # total number of samples
        
        acc_tstamp = [x[0] for x in accy]
        acc_tbf = [x[1] for x in accy]
        filtered_acc = self.butter_lowpass_filter(acc_tbf, cutoff, fs, order, n)
        acc_plot = [[i,j] for i,j in zip(acc_tstamp[:n], filtered_acc)]
        vel_acc = self.get_velocity_facc(acc_tstamp[:n], filtered_acc)
        vel_acc_plot = [[i,j] for i,j in zip(acc_tstamp[:n], vel_acc)]
        
        # ploting data
        # self.plot([accy[:], posx[:], vel_acc_plot[:], vel_pos_plot[:]], ["acceleration", "position", "velocity_acc", "velocity_pos"])
        self.plot([acc_plot[:], posx[:], vel_acc_plot[:], vel_pos_plot[:]], ["acceleration", "position", "velocity_acc", "velocity_pos"])
    
if __name__=="__main__":
    s = velocity_calc()
    s.main()