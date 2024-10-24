import numpy as np
import matplotlib.pyplot as plt

# Create a sample 2D array (5x5 array for example)
arr = np.arange(1, 26).reshape(5, 5)  # 2D array with values from 1 to 25

# Slice parameters (you can change these)
start_row, end_row, step_row = 0, 3, 1
start_col, end_col, step_col = 3, 3, 1

# Perform slicing
sliced_arr = arr[start_row:end_row:step_row, start_col:end_col:step_col]

# Visualize the original 2D array and the sliced 2D array
fig, axs = plt.subplots(1, 2, figsize=(12, 5))

# Plot the original 2D array as a heatmap
axs[0].imshow(arr, cmap='Blues', aspect='auto')
axs[0].set_title('Original 2D Array')
axs[0].set_xlabel('Columns')
axs[0].set_ylabel('Rows')

# Annotate the original array for clarity
for i in range(arr.shape[0]):
    for j in range(arr.shape[1]):
        axs[0].text(j, i, arr[i, j], ha='center', va='center', color='black')

# Plot the sliced 2D array as a heatmap
axs[1].imshow(sliced_arr, cmap='Greens', aspect='auto')
axs[1].set_title('Sliced 2D Array')
axs[1].set_xlabel('Columns')
axs[1].set_ylabel('Rows')

# Annotate the sliced array for clarity
for i in range(sliced_arr.shape[0]):
    for j in range(sliced_arr.shape[1]):
        axs[1].text(j, i, sliced_arr[i, j], ha='center', va='center', color='black')

# Display the plot
plt.tight_layout()
plt.show()
