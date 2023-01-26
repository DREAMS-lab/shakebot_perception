#!/usr/bin/python3

import json
from math import sqrt
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.signal import butter, filtfilt, savgol_filter
from datetime import timedelta


class velocity_calc:
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
        
    def format_pd(self, df):
        df = df.transpose()
        df.columns = df.iloc[0]
        df = df[1:]
        df["timestamp"] = df.index
        df.set_index(np.arange(1, df.shape[0] + 1), inplace=True)
        return df
    
    def extractData(self, var, data):
        ex_data=[]
        # if var == "pos":
        #     basetime = 0
        #     for tstamp, value in data.items():
        #         if "pose" in value:
        #             # print(value["pose"]["position"]["x"], value["pose"]["position"]["y"])
        #             px = value["pose"]["position"]["x"]
        #             py = value["pose"]["position"]["y"]
        #             if len(ex_data) == 0:
        #                 baseTime = tstamp
        #                 ex_data.append([0, sqrt(px**2 + py**2)])
        #             else:
        #                 ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), sqrt(px**2 + py**2)])
                        
        # elif var == "acc":
        #     baseTime = 0
        #     for tstamp, value in data.items():
        #         if "acceleration" in value:
        #             # print(value["acceleration"]["x"], value["acceleration"]["y"])
        #             ax = value["acceleration"]["x"] * 9.81
        #             ay = value["acceleration"]["y"] * 9.81
        #             if len(ex_data) == 0:
        #                 baseTime = tstamp
        #                 ex_data.append([0, sqrt(ax**2 + ay**2)])
        #             else:
        #                 ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), sqrt(ax**2 + ay**2)])
            
        if var == "pos":
            baseTime = 0
            for tstamp, value in data.items():
                if "pose" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["pose"]["position"]["x"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["pose"]["position"]["x"]])
        
        # elif var == "posy":
        #     baseTime = 0
        #     for tstamp, value in data.items():
        #         if "pose" in value:
        #             if len(ex_data) == 0:
        #                 baseTime = tstamp
        #                 ex_data.append([0, value["pose"]["position"]["y"]])
        #             else:
        #                 ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["pose"]["position"]["y"]])
                    
        # elif var == "posz":
        #     baseTime = 0
        #     for tstamp, value in data.items():
        #         if "pose" in value:
        #             if len(ex_data) == 0:
        #                 baseTime = tstamp
        #                 ex_data.append([0, value["pose"]["position"]["z"]])
        #             else:
        #                 ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["pose"]["position"]["z"]])
    
        # elif var == "accx":
        #     baseTime = 0
        #     for tstamp, value in data.items():
        #         if "acceleration" in value:
        #             if len(ex_data) == 0:
        #                 baseTime = tstamp
        #                 ex_data.append([0, value["acceleration"]["x"]])
        #             else:
        #                 ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["acceleration"]["x"]])
                    
        elif var == "acc":
            baseTime = 0
            for tstamp, value in data.items():
                if "acceleration" in value:
                    if len(ex_data) == 0:
                        baseTime = tstamp
                        ex_data.append([0, value["acceleration"]["y"]])
                    else:
                        ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["acceleration"]["y"]])
                
        # elif var == "accz":
        #     baseTime = 0
        #     for tstamp, value in data.items():
        #         if "acceleration" in value:
        #             if len(ex_data) == 0:
        #                 baseTime = tstamp
        #                 ex_data.append([0, value["acceleration"]["z"]])
        #             else:
        #                 ex_data.append([timedelta(microseconds=(float(tstamp) - float(baseTime))/1000).total_seconds(), value["acceleration"]["z"]])
                
        return ex_data
    
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
                    # axs[2].plot([x[0] for x in i], [x[1] for x in i], label=r'$v_a$', markersize=2)
                    count+=1
                elif count == 1:
                    axs[2].scatter([x[0] for x in i], [x[1] for x in i], label=r'$v_d$', marker="o", s=markerSize)
                    # axs[2].plot([x[0] for x in i], [x[1] for x in i], label=r'$v_d$', markersize=2)
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
        # fname = sorted(glob.glob("/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/data/raw/record*"))[-1]
        # recorded_09_07_22_12_49_28, recorded_01_24_23_12_02_42.json(0.1, 0.1), recorded_01_24_23_15_55_18.json(0.1, 0.1), recorded_01_25_23_11_29_46.json(0.1,0.1)
        # fname = "/home/"+os.environ.get("USERNAME")+"/catkin_ws/src/shakebot_perception/data/raw/recorded_01_25_23_11_29_46.json"       # Use this for linux
        fname = "data\\raw\\recorded_01_25_23_11_29_46.json"
        # fname = sorted(glob.glob("data\\raw\\record*"))[-1]      # Use this for Windows

        # print(fname)
        winSize = 8
        self.data = self.read_data(fname)
        pos = self.extractData("pos", self.data)
        acc = self.extractData("acc", self.data)
        
        pos = pos[1:-2]
        pos_tstamp = [x[0] for x in pos]
        acc = acc[4:-13]
        acc_tstamp = [x[0] for x in acc]
        
        # displacement processing to get velocity
        T_pos = [x[0] for x in pos][-1]        # Sample period
        fs_pos = 25          # Accelerometer sampling rate, Hz
        cutoff_pos = 2       # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
        order_pos = 2         # butterworth filter order
        n_pos = int(T_pos * fs_pos)   # total number of samples
        
        pos_tbf = [x[1]*-self.alpha for x in pos]
        filtered_pos = self.butter_lowpass_filter(pos_tbf, cutoff_pos, fs_pos, order_pos, n_pos)
        pos_savgol = savgol_filter(filtered_pos, winSize, 3, mode='nearest')
        pos_plot = [[i,j] for i,j in zip(pos_tstamp[:n_pos], pos_savgol)]
        pos_savgol_fn = InterpolatedUnivariateSpline(pos_tstamp[:n_pos], pos_savgol, k=1)
        
        # acceleration processing to get velocity
        T_acc = [x[0] for x in acc][-1]        # Sample period
        fs_acc = 80           # Accelerometer sampling rate, Hz
        cutoff_acc = 2       # desired cutoff frequency of the filter, Hz, slightly higher than actual 30 Hz 
        order_acc = 2         # butterworth filter order
        n_acc = int(T_acc * fs_acc)   # total number of samples
        
        acc_tbf = [((x[1]*-9.81)+0.25) for x in acc]
        filtered_acc = self.butter_lowpass_filter(acc_tbf, cutoff_acc, fs_acc, order_acc, n_acc)
        acc_savgol = savgol_filter(filtered_acc, winSize, 3, mode='nearest')
        acc_plot = [[i,j] for i,j in zip(acc_tstamp[:n_acc], filtered_acc)]
        acc_savgol_fn = InterpolatedUnivariateSpline(acc_tstamp[:n_acc], acc_savgol, k=1)
        
        # velocity Estimation
        nofSamples = 20
        combined_tstamp = [x[0] for x in acc_plot+pos_plot]
        sampled_tstamp = np.linspace(combined_tstamp[0], combined_tstamp[-1], nofSamples)
        vel_pos = self.get_velocity_fpos(sampled_tstamp, [x for x in pos_savgol_fn(sampled_tstamp)])
        vel_pos_plot = [[i,j] for i,j in zip(sampled_tstamp, vel_pos)]
        vel_acc = self.get_velocity_facc(sampled_tstamp, [x for x in acc_savgol_fn(sampled_tstamp)])
        vel_acc_plot = [[i,j] for i,j in zip(sampled_tstamp, vel_acc)]
        # print(len(np.array(list(sampled_tstamp)+list(sampled_tstamp))))
        
        # ploting data
        est_model = np.poly1d(np.polyfit(np.array(list(sampled_tstamp)+list(sampled_tstamp)), np.array(vel_acc+vel_pos), 6))
        # est_model = savgol_filter(vel_pos+vel_acc, winSize+8, 3, mode='nearest')
        vel_est_plot = [[i,j] for i,j in zip(sampled_tstamp, est_model(sampled_tstamp))]
        self.plot([acc_plot[:], pos_plot[:], vel_acc_plot[:], vel_pos_plot[:], vel_est_plot[:]], [r"$a~(m/s^2)$", r"$d~(cm)$", "velocity_acc", "velocity_pos", "velocity_estimate"])

if __name__=="__main__":
    # try:
    #     s = velocity_calc()
    #     s.main()
    # except Exception as e:
    #     print(e)
        
    s = velocity_calc()
    s.main()