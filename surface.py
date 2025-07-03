import numpy as np
import matplotlib.pyplot as plt
import math
#import tikzplotlib
import os

import pandas as pd

pd.set_option('display.float_format', '{:.19f}'.format)

epochs = 30000
n_robots = 15

filtered_computation = [0.6, 0.9, 1.2, 1.5]

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
        
        pd.DataFrame(data_d).to_csv(os.path.join(basedir, f"data_{z}.csv"), index=False)

    df = pd.read_csv(filename)

    # Define the ranges for your variables
    charge_rate = sorted(df[df["charge_rate"] >= 0.004]["charge_rate"].unique())
    discharge_rate = sorted(df[df["discharge_rate"] >= 0.004]["discharge_rate"].unique())
    computation = sorted(df[df["AI_computation"] >= 0.004]["AI_computation"].unique())

    n_plots = len(computation)
    n_cols = 5
    n_rows = math.ceil(n_plots / n_cols)

    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 4 * n_rows))
    axes = axes.flatten()  # Flatten in case it's a 2D array of axes
    fig2, axes2 = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 4 * n_rows))
    axes2 = axes2.flatten()  # Flatten in case it's a 2D array of axes
    fig3, axes3 = plt.subplots(figsize=(5, 4))

    a3 = {}
    a4 = {}
    a5 = {}
    a6 = {}
    # print(df)
    for id_z, z in enumerate(computation):
        min_val = 1500000
        max_val = 0
        min_val_2 = 1500000
        max_val_2 = 0
        a3[round(z*100, 2)] = []
        a4[round(z*100, 2)] = []
        a5[round(z*100, 2)] = []
        a6[round(z*100, 2)] = []

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
                value = 100*(best_offload - no_offload) / no_offload
                value3 = 100*(best_offload - impl_offload) / impl_offload
                value2 = 100*(impl_offload - no_offload) / no_offload

                a3[round(z*100, 2)].append(value3)
                a4[round(z*100, 2)].append(value)
                a5[round(z*100, 2)].append(df[(df["charge_rate"] == x) & (df["discharge_rate"] == y) & (df["AI_computation"] == z)]["wasted_computation_no_offload"].values[0] / (n_robots*epochs))
                a6[round(z*100, 2)].append(df[(df["charge_rate"] == x) & (df["discharge_rate"] == y) & (df["AI_computation"] == z)]["wasted_computation_offload"].values[0] / (n_robots*epochs))

                # if value < 0:
                #     value = 0
                # if value2 < 0:  
                #     value2 = 0

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

        if round(z*100, 2) in filtered_computation:
            dump_data(a2, round(z*100, 2))

        im = axes[id_z].imshow(a, extent=(discharge_rate[0]*100, discharge_rate[-1]*100, charge_rate[0]*100, charge_rate[-1]*100), cmap='viridis', aspect='auto',origin='lower')#, vmin=0, vmax=170)
        axes[id_z].set_title(f'Computation = {round(z*100, 2)} (% tot battery)')
        axes[id_z].set_xlabel('Discharge rate (% tot battery)')
        axes[id_z].set_ylabel('Charge rate (% tot battery)')

        im2 = axes2[id_z].imshow(a2, extent=(discharge_rate[0]*100, discharge_rate[-1]*100, charge_rate[0]*100, charge_rate[-1]*100), cmap='viridis', aspect='auto',origin='lower')#, vmin=0, vmax=30)
        axes2[id_z].set_title(f'Computation = {round(z*100, 2)} (% tot battery)')
        axes2[id_z].set_xlabel('Discharge rate (% tot battery)')
        axes2[id_z].set_ylabel('Charge rate (% tot battery)')
        
        fig.colorbar(im, ax=axes[id_z])
        fig2.colorbar(im2, ax=axes2[id_z])

    cdf_data = {}

    for label, data in a3.items():
        sorted_data = np.sort(data)
        cdf = np.arange(1, len(sorted_data) + 1) / len(sorted_data)
        axes3.plot(sorted_data, cdf, marker='.', linestyle='none', label=label, markersize=3)
        if label not in cdf_data:
            cdf_data[str(round(label, 1)) + "_x"] = sorted_data
            cdf_data[str(round(label, 1)) + "_y"] = cdf

    pd.DataFrame(cdf_data).to_csv(os.path.join(basedir, "cdf_data_RANDOM.csv"), index=False)

    cdf_data = {}

    for label, data in a4.items():
        sorted_data = np.sort(data)
        cdf = np.arange(1, len(sorted_data) + 1) / len(sorted_data)
        if label not in cdf_data:
            cdf_data[str(round(label, 1)) + "_x"] = sorted_data
            cdf_data[str(round(label, 1)) + "_y"] = cdf

    pd.DataFrame(cdf_data).to_csv(os.path.join(basedir, "cdf_data_offload_server.csv"), index=False)
    pd.DataFrame(a5).to_csv(os.path.join(basedir, "wasted_computation_no_offload.csv"), index=False)
    pd.DataFrame(a6).to_csv(os.path.join(basedir, "wasted_computation_offload_RANDOM.csv"), index=False)

    # Customize plot
    axes3.set_xlabel('Operating time gain (%)')
    axes3.set_ylabel('CDF')
    axes3.grid(True)
    axes3.legend(title="Computation (%)")
    fig3.savefig(os.path.join(basedir, "res_RANDOM_offloading_benefits.png"))

    #pd.DataFrame(a3).to_csv("CDF.csv", index=False)
    fig.tight_layout()
    fig2.tight_layout()
    fig.savefig(os.path.join(basedir, "res_offloading_benefits.png"))
    fig2.savefig(os.path.join(basedir, "res_RANDOM_offloading_gap.png"))

basedir = "results/15nodes"    

plot_heatmap(filename=os.path.join(basedir, "results_RANDOM.csv"))


    

