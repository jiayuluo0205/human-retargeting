import matplotlib.pyplot as plt
import numpy as np

# 类别
categories = ['Pick&Place', 'Plug', 'Pour', 'Functional']

# 原始系统数据
original_means = [20, 35, 30, 35]
original_std_devs = [2, 3, 4, 1]
original_attempts_for_10_success = [25, 40, 35, 45]

# FuScope 辅助系统数据
foscope_means = [15, 25, 20, 30]  # 较原始系统更短的完成时间
foscope_std_devs = [3, 2, 5, 2]    # 任意标准差
foscope_attempts_for_10_success = [12, 15, 18, 10]  # 降低到10到20之间

# x轴位置
x_pos = np.arange(len(categories))

# 定义柱状图宽度
bar_width = 0.35

# 创建图形和轴
fig, ax1 = plt.subplots(figsize=(12, 8))

# 创建第二个y轴
ax2 = ax1.twinx()

# 绘制原始系统的10次成功所需次数柱状图
original_bars = ax2.bar(
    x_pos - bar_width/2, 
    original_attempts_for_10_success, 
    bar_width, 
    label='Original Attempts for 10 Successes',
    alpha=0.6, 
    color='saddlebrown', 
    edgecolor='black'
)

# 绘制FuScope辅助系统的10次成功所需次数柱状图
foscope_bars = ax2.bar(
    x_pos + bar_width/2, 
    foscope_attempts_for_10_success, 
    bar_width, 
    label='FuScope Attempts for 10 Successes',
    alpha=0.6, 
    color='peru', 
    edgecolor='black'
)

# 绘制原始系统的平均完成时间线
original_line = ax1.errorbar(
    x_pos, 
    original_means, 
    yerr=original_std_devs, 
    fmt='-o', 
    color='red', 
    ecolor='salmon', 
    elinewidth=2, 
    capsize=5, 
    label='Original Mean Completion Time'
)

# 绘制FuScope辅助系统的平均完成时间线
foscope_line = ax1.errorbar(
    x_pos, 
    foscope_means, 
    yerr=foscope_std_devs, 
    fmt='-o', 
    color='blue', 
    ecolor='lightblue', 
    elinewidth=2, 
    capsize=5, 
    label='FuScope Mean Completion Time'
)

# 设置y轴范围（固定原始的纵轴范围）
ax1.set_ylim(0, 40)   # 对于完成时间
ax2.set_ylim(0, 50)   # 对于尝试次数

# 设置标签和标题
ax1.set_ylabel('Mean Completion Time (s)', fontsize=12, color='black')
ax2.set_ylabel('Attempts for 10 Successes', fontsize=12, color='black')
ax1.set_xticks(x_pos)
ax1.set_xticklabels(categories, fontsize=12)
ax1.set_title('Comparison of Original Teleop System and FuScope Assistant', fontsize=14)

# 匹配y轴标签颜色
ax1.tick_params(axis='y', labelcolor='black')
ax2.tick_params(axis='y', labelcolor='saddlebrown')

# 合并图例
bars = [original_bars, foscope_bars]
lines = [original_line, foscope_line]
labels = [bar.get_label() for bar in bars] + [line.get_label() for line in lines]
ax1.legend(bars + lines, labels, loc='upper left')

# 移除网格线
ax1.yaxis.grid(False)
ax1.xaxis.grid(False)
ax2.yaxis.grid(False)

# 调整布局以防止裁剪
plt.tight_layout()

# 显示图表
plt.show()