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
        vel_acc = [f.integral(tstamp[0], tstamp[i]) for i in range(len(tstamp))]
        return vel_acc
    
    def get_velocity_fpos(self, tstamp, filtered_disp):
        # dx = np.diff(filtered_disp)
        # dt = np.diff(tstamp)
        # vel_pos = dx/dt
        # print(vel_pos)
        f = InterpolatedUnivariateSpline(tstamp, filtered_disp, k=1)  # k=1 gives linear interpolation
        fp = f.derivative()
        vel_pos = [fp(i) for i in tstamp]
        # p1 = filtered_disp[:-1]
        # p2 = filtered_disp[1:]
        # t1 = tstamp[:-1]
        # t2 = tstamp[1:]
        # dp = np.array(p2)-np.array(p1)
        # dt = np.array(t2)-np.array(t1)
        # vel_pos = [i/j for i,j in zip(dp, dt)]
        return vel_pos
                
    def butter_lowpass_filter(self, sample_data, cutoff, fs, order, n):
        data = sample_data[:n]
        nyq = 0.5 * fs  # Nyquist Frequency:
        normal_cutoff = cutoff / nyq
        # Get the filter coefficients 
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data)
        return y
    
    def butter_highpass_filter(self, sample_data, cutoff, fs, order, n):
        data = sample_data[:n]
        nyq = 0.5 * fs  # Nyquist Frequency:
        normal_cutoff = cutoff / nyq
        # Get the filter coefficients 
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        y = filtfilt(b, a, data)
        return y
    
    def main(self):
        fname = sorted(glob.glob("/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/scripts/record*"))[-6]
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
        T_pos = [x[0] for x in posx][-1]        # Sample period
        fs_pos = 60           # Accelerometer sampling rate, Hz
        cutoff_pos = 5       # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
        order_pos = 2         # butterworth filter order
        n_pos = int(T_pos * fs_pos)   # total number of samples
        
        pos_tstamp = [x[0] for x in posx]
        pos_tbf = [x[1] for x in posx]
        filtered_pos = self.butter_lowpass_filter(pos_tbf, cutoff_pos, fs_pos, order_pos, n_pos)
        pos_plot = [[i,j] for i,j in zip(pos_tstamp[:n_pos], filtered_pos)]
        vel_pos = self.get_velocity_fpos(pos_tstamp[:n_pos], filtered_pos)
        vel_pos_plot = [[i,j] for i,j in zip(pos_tstamp[:n_pos], vel_pos)]
        # vel_pos = self.get_velocity_fpos(pos_tstamp, pos_tbf)
        # vel_pos_plot = [[i,j] for i,j in zip(pos_tstamp, vel_pos)]
        
        # acceleration processing to get velocity
        T_acc = [x[0] for x in accy][-1]        # Sample period
        fs_acc = 80           # Accelerometer sampling rate, Hz
        cutoff_acc = 30       # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
        order_acc = 2         # butterworth filter order
        n_acc = int(T_acc * fs_acc)   # total number of samples
        
        acc_tstamp = [x[0] for x in accy]
        acc_tbf = [x[1] for x in accy]
        filtered_acc = self.butter_lowpass_filter(acc_tbf, cutoff_acc, fs_acc, order_acc, n_acc)
        acc_plot = [[i,j] for i,j in zip(acc_tstamp[:n_acc], filtered_acc)]
        vel_acc = self.get_velocity_facc(acc_tstamp[:n_acc], filtered_acc)
        vel_acc_plot = [[i,j] for i,j in zip(acc_tstamp[:n_acc], vel_acc)]
        
        # ploting data
        # self.plot([accy[:], posx[:], vel_acc_plot[:], vel_pos_plot[:]], ["acceleration", "position", "velocity_acc", "velocity_pos"])
        self.plot([acc_plot[:], pos_plot[:], vel_acc_plot[:], vel_pos_plot[:]], ["acceleration", "position", "velocity_acc", "velocity_pos"])
        plt.plot(pos_tstamp,vel_pos)
        plt.show()
    
if __name__=="__main__":
    s = velocity_calc()
    s.main()