#!/usr/bin/python3

from cProfile import label
import pandas as pd
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
    
    def extractData(self, var, df):
        data=[]
        if var == "posx":
            for i in range(1,len(df)):
                x = ast.literal_eval(df["pose"][i])
                data.append([x["position"]["x"], df["timestamp"][i]])
        
        elif var == "posy":
            for i in range(1,len(df)):
                x = ast.literal_eval(df["pose"][i])
                data.append([x["position"]["y"], df["timestamp"][i]])
        
        elif var == "posz":
            for i in range(1,len(df)):
                x = ast.literal_eval(df["pose"][i])
                data.append([x["position"]["z"], df["timestamp"][i]])
        
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
            for i in range(1,len(df)):
                x = ast.literal_eval(df["acceleration"][i])
                data.append([x["x"], df["timestamp"][i]])
                
        elif var == "accy":
            for i in range(1,len(df)):
                x = ast.literal_eval(df["acceleration"][i])
                data.append([x["y"], df["timestamp"][i]])
                
        elif var == "accz":
            for i in range(1,len(df)):
                x = ast.literal_eval(df["acceleration"][i])
                data.append([x["z"], df["timestamp"][i]])
                
        return data

    def cal_time_period(self, lst):
        print(lst[-1], lst[0])
        t = (float(lst[-1]) - float(lst[0]))
        t = timedelta(microseconds=t/1000).total_seconds()
        return t
    
    def plot(self, data, labels):
        f = plt.figure(dpi=300)
        plt.xlabel("time (sec)")
        plt.ylabel("acc (m/s^2)")
        for i, label in zip(data,labels):
            plt.plot([x[1] for x in i], [x[0] for x in i], label=label)
        plt.legend()
        plt.show()
        
    def butter_lowpass_filter(self, sample_data, cutoff, fs, order, n, ):
        data = sample_data[:n]
        nyq = 0.5 * fs  # Nyquist Frequency:
        normal_cutoff = cutoff / nyq
        # Get the filter coefficients 
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        y = filtfilt(b, a, data)
        return y
    
    def main(self):
        self.data = pd.read_csv("~/catkin_ws/src/shakebot_perception/scripts/recorded.csv")
        self.data = self.format_pd(self.data)
        posx = self.extractData("posx", self.data)
        # posy = self.extractData("posy", self.data)
        # posz = self.extractData("posz", self.data)
        # qx = self.extractData("qx", self.data)
        # qy = self.extractData("qy", self.data)
        # qz = self.extractData("qz", self.data)
        # qw = self.extractData("qw", self.data)
        # accx = self.extractData("accx", self.data)
        accy = self.extractData("accy", self.data)
        # accz = self.extractData("accz", self.data)
        
        # T = self.cal_time_period(sorted(self.data["timestamp"]))        # Sample period
        # print(T)
        # fs = 40       # Accelerometer sampling rate, Hz
        # cutoff = 30     # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
        # order = 2       # butterworth filter order
        # n = int(T * fs) # total number of samples

        # filtered_acc = self.butter_lowpass_filter(accy, cutoff, fs, order, n)
        self.plot([posx[:], accy[:]], ["position", "acceleration"])
    
if __name__=="__main__":
    s = velocity_calc()
    s.main()