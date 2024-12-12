import matplotlib.pyplot as plt
import numpy as np

categories = ['Pick&Place', 'Plug', 'Pour', 'Functional']
means = [15, 25, 20, 30]
std_devs = [3, 2, 5, 2]
attempts_for_10_success = [12, 15, 18, 10] 

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
    color='peru', 
    edgecolor='black',
    label='Attempts for 10 Successes'
)

# Plot blue line with error bars on the primary y-axis
line = ax1.errorbar(
    x_pos, 
    means, 
    yerr=std_devs, 
    fmt='-o',                # Line with circle markers
    color='blue', 
    ecolor='blue',         # Error bar color
    elinewidth=2, 
    capsize=5, 
    label='Mean ± Std Compelete Time'
)

# Set the y-axis limits to start from 0
ax1.set_ylim(0, 40)  
ax2.set_ylim(0, 50) 

# Set labels and title
ax1.set_ylabel('Mean Compelete Time (s)', fontsize=12, color='black')
ax2.set_ylabel('Attempts for 10 Successes', fontsize=12, color='black')
ax1.set_xticks(x_pos)
ax1.set_xticklabels(categories, fontsize=12)
ax1.set_title('Our teleop system w/ FuScope assistant', fontsize=14)

# Match the color of y-axis labels to the corresponding elements
ax1.tick_params(axis='y', labelcolor='blue')
ax2.tick_params(axis='y', labelcolor='peru')

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