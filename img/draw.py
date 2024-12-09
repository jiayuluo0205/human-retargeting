import matplotlib.pyplot as plt
import numpy as np

# Example Data
categories = ['A', 'B', 'C', 'D']
means = [20, 35, 30, 35]
std_devs = [2, 3, 4, 1]
attempts_for_10_success = [25, 40, 35, 45]  # Background bars

# Positions of the bars on the x-axis
x_pos = np.arange(len(categories))

# Define bar widths
background_bar_width = 0.6  # Width for brown background bars

# Create the figure and the first axis
fig, ax1 = plt.subplots(figsize=(10, 7))

# Create a second y-axis sharing the same x-axis
ax2 = ax1.twinx()

# Plot brown background bars on the secondary y-axis
background_bars = ax2.bar(
    x_pos, 
    attempts_for_10_success, 
    background_bar_width, 
    align='center', 
    alpha=0.6,               # Transparency
    color='saddlebrown', 
    edgecolor='black',
    label='Attempts for 10 Successes'
)

# Plot red line with error bars on the primary y-axis
line = ax1.errorbar(
    x_pos, 
    means, 
    yerr=std_devs, 
    fmt='-o',                # Line with circle markers
    color='red', 
    ecolor='salmon',         # Error bar color
    elinewidth=2, 
    capsize=5, 
    label='Mean ± Std Dev'
)

# Set the y-axis limits to start from 0
ax1.set_ylim(bottom=0)
ax2.set_ylim(bottom=0)

# Set labels and title
ax1.set_ylabel('Mean Value', fontsize=12, color='red')
ax2.set_ylabel('Attempts for 10 Successes', fontsize=12, color='saddlebrown')
ax1.set_xticks(x_pos)
ax1.set_xticklabels(categories, fontsize=12)
ax1.set_title('Mean ± Std Dev and Attempts for 10 Successes by Category', fontsize=14)

# Match the color of y-axis labels to the corresponding elements
ax1.tick_params(axis='y', labelcolor='red')
ax2.tick_params(axis='y', labelcolor='saddlebrown')

# Combine legends from both axes
bars = [background_bars, line]
labels = [bar.get_label() for bar in bars]
ax1.legend(bars, labels, loc='upper left')

# Remove grid lines for a cleaner look
ax1.yaxis.grid(False)
ax1.xaxis.grid(False)
ax2.yaxis.grid(False)

# Adjust layout to prevent clipping
plt.tight_layout()

# Display the plot
plt.show()