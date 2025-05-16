import sys
import numpy as np
import matplotlib.pyplot as plt
import math

import pandas as pd

pd.set_option('display.float_format', '{:.19f}'.format)

epochs = 20000
n_robots = 15

# Define your function - placeholder
def compute_value(x, y, z):
    # x = x/100
    # y = y/100
    # z = z/100
    
    offload = compute_offload(x, y, z, epochs)
    no_offload = compute_no_offload(x, y, z, epochs)
    return 100*(offload - no_offload) / offload

def compute_no_offload(x, y, z, epochs):
    return epochs*(1/(y+z))/((1/(y+z)) + (1/x))

def compute_offload(x, y, z, epochs):
    return epochs*(1/y)/((1/y) + (1/x))

def plot_heatmap(filename=None):
    def dump_data(data, z):
        data_d = {}
        data_d["x"] = []
        data_d["y"] = []
        data_d["z"] = []

        l = len(data[0])
        for i in range(l):
            for j in range(l):
                data_d["x"].append(discharge_rate[j]*100)
                data_d["y"].append(charge_rate[i]*100)
                data_d["z"].append(data[i][j])
        
        pd.DataFrame(data_d).to_csv(f"data_{z}.csv", index=False)

    df = pd.read_csv(filename)

    # Define the ranges for your variables
    charge_rate = sorted(df["charge_rate"].unique())
    discharge_rate = sorted(df["discharge_rate"].unique())
    computation = sorted(df["AI_computation"].unique())

    n_plots = len(computation)
    n_cols = 5
    n_rows = math.ceil(n_plots / n_cols)

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 4 * n_rows))
    axes = axes.flatten()  # Flatten in case it's a 2D array of axes
    fig2, axes2 = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 4 * n_rows))
    axes2 = axes2.flatten()  # Flatten in case it's a 2D array of axes

    # print(df)
    for id_z, z in enumerate(computation):
        min_val = 1500000
        max_val = 0
        min_val_2 = 1500000
        max_val_2 = 0

        a = np.zeros((len(charge_rate), len(discharge_rate)))
        a2 = np.zeros((len(charge_rate), len(discharge_rate)))
        for id_x, x in enumerate(charge_rate):
            for id_y, y in enumerate(discharge_rate):
                ch_t = 100/charge_rate[id_x]
                d_t_o = 100/discharge_rate[id_y]
                d_t_no = 100/(discharge_rate[id_y]+computation[id_z])

                best_offload = ((epochs * d_t_o)/(d_t_o + ch_t))*n_robots
                no_offload = ((epochs * d_t_no)/(d_t_no + ch_t))*n_robots
                # best_offload = df[(df["charge_rate"] == x) & (df["discharge_rate"] == y) & (df["AI_computation"] == z)]["best_offload"].values[0]
                # no_offload = df[(df["charge_rate"] == x) & (df["discharge_rate"] == y) & (df["AI_computation"] == z)]["no_offload"].values[0]
                impl_offload = df[(df["charge_rate"] == x) & (df["discharge_rate"] == y) & (df["AI_computation"] == z)]["offload"].values[0]
                value = 100*(best_offload - no_offload) / best_offload
                value2 = 100*(best_offload - impl_offload) / best_offload

                # if value < 0:
                #     value = 0
                if value2 < 0:  
                    value2 = 0

                if value < min_val:
                    min_val = value
                if value > max_val:
                    max_val = value

                if value2 < min_val_2:
                    min_val_2 = value2
                if value2 > max_val_2:
                    max_val_2 = value2

                a[id_x][id_y] = value
                a2[id_x][id_y] = value2

        if round(z*100, 2) in [0.23, 0.45, 0.78, 1]:
            dump_data(a2, round(z*100, 2))

        im = axes[id_z].imshow(a, extent=(discharge_rate[0]*100, discharge_rate[-1]*100, charge_rate[0]*100, charge_rate[-1]*100), cmap='viridis', aspect='auto',origin='lower')#, vmin=0, vmax=100)
        axes[id_z].set_title(f'Computation = {round(z*100, 2)} (% tot battery)')
        axes[id_z].set_xlabel('Discharge rate (% tot battery)')
        axes[id_z].set_ylabel('Charge rate (% tot battery)')

        im2 = axes2[id_z].imshow(a2, extent=(discharge_rate[0]*100, discharge_rate[-1]*100, charge_rate[0]*100, charge_rate[-1]*100), cmap='viridis', aspect='auto',origin='lower')#, vmin=0, vmax=50)
        axes2[id_z].set_title(f'Computation = {round(z*100, 2)} (% tot battery)')
        axes2[id_z].set_xlabel('Discharge rate (% tot battery)')
        axes2[id_z].set_ylabel('Charge rate (% tot battery)')
        
        fig.colorbar(im, ax=axes[id_z])
        fig2.colorbar(im2, ax=axes2[id_z])

    fig.tight_layout()
    fig2.tight_layout()
    fig.savefig("prova.png")
    fig2.savefig("prova2.png")

plot_heatmap(filename="results.csv")


    

