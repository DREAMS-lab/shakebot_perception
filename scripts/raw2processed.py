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


class raw2processed:
    def __init__(self):
        self.data = {}
        self.alpha = self.readAlpha()/1000
        
    def readAlpha(self):
        # outf = "/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/config/perceptionCalib.yaml"     # Use this for linux
        outf = "config\perceptionCalib.yaml"       # Use this for Windows
        with open(outf,"r") as f:
            param = f.read()
            param = float(param.strip('[').strip("]"))
        return param
    
    def extractData(self, var, data):
        ex_data=[]

        if var == "pos":
            baseTime = 0
            for tstamp, value in data.items():
                if "pose" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["pose"]["position"]["x"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["pose"]["position"]["x"]])
        
        elif var == "acc":
            baseTime = 0
            for tstamp, value in data.items():
                if "acceleration" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["acceleration"]["y"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["acceleration"]["y"]])

        return ex_data
        
    def read_data(self, file):
        with open(file, "r") as f:
            data = json.load(f)
            f.close()
        return data
    
    def saveProcessed(self, data):
        jsonData = {}
        for idx,i in enumerate(data):
            if idx == 0:
                key = "acc"
            elif idx == 1:
                key = "pos"
            elif idx == 2:
                key = "vel_acc"
            elif idx == 3:
                key = "vel_pos"
            elif idx == 4:
                key = "vel_est"
                
            jsonData.update({key: i})
        
        self.fname = self.fname.split(".")[0] + "_processed.json"
        self.fname = self.fname.replace("raw", "processed")
        with open(self.fname, "w") as f:
            json.dump(jsonData, f)
            f.close()
        
    
    def get_velocity_facc(self, tstamp, filtered_acc):
        f = InterpolatedUnivariateSpline(tstamp, filtered_acc, k=1)  # k=1 gives linear interpolation
        vel_acc = [f.integral(tstamp[0], tstamp[i]) for i in range(len(tstamp))]
        return vel_acc
    
    def get_velocity_fpos(self, tstamp, filtered_disp):
        f = InterpolatedUnivariateSpline(tstamp, filtered_disp, k=1)  # k=1 gives linear interpolation
        fp = f.derivative()
        vel_pos = [fp(i) for i in tstamp]
        return vel_pos
                
    def butter_lowpass_filter(self, sample_data, cutoff, fs, order, n):
        data = sample_data[:n]
        nyq = 0.5 * fs  # Nyquist Frequency:
        normal_cutoff = cutoff / nyq
        # Get the filter coefficients 
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data, padlen=len(data)-1)
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
        # self.fname = sorted(glob.glob("/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/data/raw/record*"))[-1]
        # recorded_09_07_22_12_49_28
        # fname = "/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/data/raw/recorded_09_07_22_12_49_28.json"       # Use this for linux
        self.fnamelist = sorted(glob.glob("data\\raw\\record*"))      # Use this for Windows
        
        for i in self.fnamelist:
            try:
                self.fname = i

                # print(fname)
                self.data = self.read_data(self.fname)
                pos = self.extractData("pos", self.data)
                acc = self.extractData("acc", self.data)
                
                # displacement processing to get velocity
                T_pos = [x[0] for x in pos][-1]        # Sample period
                fs_pos = 25          # Accelerometer sampling rate, Hz
                cutoff_pos = 5       # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
                order_pos = 2         # butterworth filter order
                n_pos = int(T_pos * fs_pos)   # total number of samples
                
                pos_tstamp = [x[0] for x in pos]
                pos_tbf = [x[1]*self.alpha for x in pos]
                filtered_pos = self.butter_lowpass_filter(pos_tbf, cutoff_pos, fs_pos, order_pos, n_pos)
                pos_plot = [[i,j] for i,j in zip(pos_tstamp[:n_pos], filtered_pos)]
                vel_pos = self.get_velocity_fpos(pos_tstamp[:n_pos], [x for x in filtered_pos])
                vel_pos_plot = [[i,float(j)] for i,j in zip(pos_tstamp[:n_pos], vel_pos)]
                # vel_pos = self.get_velocity_fpos(pos_tstamp, pos_tbf)
                # vel_pos_plot = [[i,j] for i,j in zip(pos_tstamp, vel_pos)]
                
                # acceleration processing to get velocity
                T_acc = [x[0] for x in acc][-1]        # Sample period
                fs_acc = 80           # Accelerometer sampling rate, Hz
                cutoff_acc = 30       # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
                order_acc = 2         # butterworth filter order
                n_acc = int(T_acc * fs_acc)   # total number of samples
                
                acc_tstamp = [x[0] for x in acc]
                acc_tbf = [x[1]*9.81 for x in acc]
                filtered_acc = self.butter_lowpass_filter(acc_tbf, cutoff_acc, fs_acc, order_acc, n_acc)
                acc_plot = [[i,j] for i,j in zip(acc_tstamp[:n_acc], filtered_acc)]
                vel_acc = self.get_velocity_facc(acc_tstamp[:n_acc], [x for x in filtered_acc])
                vel_acc_plot = [[i,j] for i,j in zip(acc_tstamp[:n_acc], vel_acc)]
                
                # ploting data
                est_model = np.poly1d(np.polyfit(np.array(acc_tstamp[:n_acc]+pos_tstamp[:n_pos]), np.array(vel_acc+vel_pos), 3))
                vel_est_plot = [[i,j] for i,j in zip(acc_tstamp[:n_acc], est_model(acc_tstamp[:n_acc]))]
                self.saveProcessed([acc_plot[:], pos_plot[:], vel_acc_plot[:], vel_pos_plot[:], vel_est_plot[:]])
            except (IndexError, ValueError):
                pass

if __name__=="__main__":
    # try:
    #     s = raw2processed()
    #     s.main()
    # except Exception as e:
    #     print(e)
        
    s = raw2processed()
    s.main()