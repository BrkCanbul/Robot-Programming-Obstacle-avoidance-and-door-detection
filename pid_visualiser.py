#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import argparse
import pandas as pd


def main(filename='pid_history_20:6:47.csv'):
    # read the data from the CSV file
    df = pd.read_csv(filename)
    print(df.head(5))
    # Grafik oluşturma
    
    t     = df['time'].to_numpy()
    t_rel = t - t[0]
    
    desired = df['desired_yaw'].to_numpy()
    actual  = df['actual_yaw'].to_numpy()
    w_cmd   = df['w_command'].to_numpy()
    v_cmd   = df['v_command'].to_numpy()

    # 3) Çizim
    plt.figure(figsize=(12, 6))

    # 3a) Yaw plot
    plt.subplot(2,1,1)
    plt.plot(t_rel, desired, label='Desired Yaw', color='blue')
    plt.plot(t_rel, actual,  label='Actual Yaw',  color='orange')
    plt.ylabel('Yaw Açısı (rad)')
    plt.legend()
    plt.grid(True)

    # 3b) Komutlar
    plt.subplot(2,1,2)
    plt.plot(t_rel, w_cmd, label='ω Command', color='green')
    plt.plot(t_rel, v_cmd, label='v Command', color='red')
    plt.xlabel('Zaman (s)')
    plt.ylabel('Komut Değeri')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show() 
    
if __name__ == "__main__":
    
    argparse = argparse.ArgumentParser(description='PID Controller Visualiser')
    argparse.add_argument('--filename','-f', type=str, default='pid_history.csv',
                            help='CSV file containing PID history data')
    args = argparse.parse_args()
    filename = args.filename
    main(filename=filename)
    print(f"Visualising PID history from {filename}")
