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
                        ex_data.append([timedelta(microseconds=(int(tstamp) - int(baseTime))/1000).total_seconds(), value["pose"]["position"]["x"]])
        
        elif var == "posy":
            baseTime = 0
            for tstamp, value in data.items():
                if "pose" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["pose"]["position"]["y"]])
                    else:
                        ex_data.append([timedelta(microseconds=(int(tstamp) - int(baseTime))/1000).total_seconds(), value["pose"]["position"]["y"]])
                    
        elif var == "posz":
            baseTime = 0
            for tstamp, value in data.items():
                if "pose" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["pose"]["position"]["z"]])
                    else:
                        ex_data.append([timedelta(microseconds=(int(tstamp) - int(baseTime))/1000).total_seconds(), value["pose"]["position"]["z"]])
        
        # elif var == "qx":
        #     for i in range(1,len(df)):
        #         x = ast.literal_eval(df["pose"][i])
        #         data.append(x["orientation"]["x"])
                
        # elif var == "qy":
        #     for i in range(1,len(df)):
        #         x = ast.literal_eval(df["pose"][i])
        #         data.append(x["orientation"]["y"])
                
        # elif var == "qz":
        #     for i in range(1,len(df)):
        #         x = ast.literal_eval(df["pose"][i])
        #         data.append(x["orientation"]["z"])
                
        # elif var == "qw":
        #     for i in range(1,len(df)):
        #         x = ast.literal_eval(df["pose"][i])
        #         data.append(x["orientation"]["w"])
    
        elif var == "accx":
            baseTime = 0
            for tstamp, value in data.items():
                if "acceleration" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["acceleration"]["x"]])
                    else:
                        ex_data.append([timedelta(microseconds=(int(tstamp) - int(baseTime))/1000).total_seconds(), value["acceleration"]["x"]])
                    
        elif var == "accy":
            baseTime = 0
            for tstamp, value in data.items():
                if "acceleration" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["acceleration"]["y"]])
                    else:
                        ex_data.append([timedelta(microseconds=(int(tstamp) - int(baseTime))/1000).total_seconds(), value["acceleration"]["y"]])
                
        elif var == "accz":
            baseTime = 0
            for tstamp, value in data.items():
                if "acceleration" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["acceleration"]["z"]])
                    else:
                        ex_data.append([timedelta(microseconds=(int(tstamp) - int(baseTime))/1000).total_seconds(), value["acceleration"]["z"]])
                
        return ex_data

    def cal_time_period(self, lst):
        print(lst[-1], lst[0])
        t = (float(lst[-1]) - float(lst[0]))
        t = timedelta(microseconds=t/1000).total_seconds()
        return t
    
    def plot(self, data, labels):
        fig, axs = plt.subplots(3)
        # axs[0].plot([x[0] for x in data[0]], [x[1] for x in data[0]], label=labels[0])
        # #     axs[idx].set(xlabel="time (s)", ylabel=label)
        for idx, i, label in zip(range(len(data)),data,labels):
            axs[idx].plot([x[0] for x in i], [x[1] for x in i], label=label)
            axs[idx].set(xlabel="time (s)", ylabel=label)
        plt.show()
        
    def read_data(self, file):
        with open(file, "r") as f:
            data = json.load(f)
            f.close()
        return data
        
    def butter_lowpass_filter(self, sample_data, cutoff, fs, order, n, ):
        data = sample_data[:n]
        nyq = 0.5 * fs  # Nyquist Frequency:
        normal_cutoff = cutoff / nyq
        # Get the filter coefficients 
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data)
        return y
    
    def main(self):
        fname = sorted(glob.glob("/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/scripts/record*"))[-3]
        self.data = self.read_data(fname)
        posx = self.extractData("posx", self.data)
        # posy = self.extractData("posy", self.data)
        # posz = self.extractData("posz", self.data)
        # accx = self.extractData("accx", self.data)
        accy = self.extractData("accy", self.data)
        # accz = self.extractData("accz", self.data)

        # T = self.cal_time_period(sorted(self.data["timestamp"]))        # Sample period
        # print(T);
        # fs = 40       # Accelerometer sampling rate, Hz
        # cutoff = 30     # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
        # order = 2       # butterworth filter order
        # n = int(T * fs) # total number of samples

        # filtered_acc = self.butter_lowpass_filter(accy, cutoff, fs, order, n)
        self.plot([accy[:], posx[:]], ["acceleration", "position"])
    
if __name__=="__main__":
    s = velocity_calc()
    s.main()