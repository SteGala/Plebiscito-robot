import numpy as np
import matplotlib.pyplot as plt
import math

# Define the ranges for your variables
charge_rate = np.linspace(0, 100, 100)
discharge_rate = np.linspace(0, 100, 100)
computation = [i*10 for i in range(1, 13)]  # Customize as needed

tot_battery = 11880  # Example total battery
epochs = 10000

# Create a meshgrid for x and y
X, Y = np.meshgrid(charge_rate, discharge_rate, indexing='ij')

# Define your function - placeholder
def compute_value(x, y, z):
    offload = epochs*(tot_battery/y)/((tot_battery/y) + (tot_battery/x))
    no_offload = epochs*(tot_battery/(y+z))/((tot_battery/(y+z)) + (tot_battery/x))
    return 100*(offload - no_offload) / offload

# Setup subplot layout
n_plots = len(computation)
n_cols = 3
n_rows = math.ceil(n_plots / n_cols)

fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 4 * n_rows))
axes = axes.flatten()  # Flatten in case it's a 2D array of axes

# Plot heatmaps
for i, z in enumerate(computation):
    result = compute_value(X, Y, z)
    im = axes[i].imshow(result, extent=(discharge_rate[0], discharge_rate[-1], charge_rate[0], charge_rate[-1]),
                        origin='lower', cmap='viridis', aspect='auto')
    axes[i].set_title(f'Computation = {z} (units)')
    axes[i].set_xlabel('Discharge rate (units)')
    axes[i].set_ylabel('Charge rate (units)')
    fig.colorbar(im, ax=axes[i])

# Hide any unused subplots
for j in range(i + 1, len(axes)):
    fig.delaxes(axes[j])

plt.tight_layout()
plt.savefig("prova.png")
