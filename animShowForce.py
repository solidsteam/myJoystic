


import matplotlib.pyplot as plt
from numpy import *
from matplotlib import animation
import serial  # 
import serialPart

import time

# from math import *
ser = serialPart.serial_open()


fig = plt.figure(1)
ax = fig.add_subplot(1, 1, 1)
t = [0]
t_now = 0
m = [sin(t_now)]

view_length = 10
dot_list = list(range(view_length))
i = arange(0, 0.08, 0.01)

t_now = i * 0.01


line, = ax.plot(cos((1.5 + (t_now % 13) * 0.1) * (t_now - 1.2)), sin(3 * t_now), '.', color='red')
# another_line = ax.plot(cos((1.5 + (t_now % 13) * 0.1) * (t_now - 1.2)), sin(3 * t_now), '.', color='blue')
plt.xlim(-400, 400)
plt.ylim(-400, 400)
# /————————————————

ax = plt.gca()  # 得到图像的Axes对象
ax.set_aspect(1)  # 这句会使得横纵两轴的单位长度严格相等
plt.grid()  # 这句添加网格
ax.spines['right'].set_color('none')  # 将图像右边的轴设为透明
ax.spines['top'].set_color('none')  # 将图像上面的轴设为透明
ax.xaxis.set_ticks_position('bottom')  # 将x轴刻度设在下面的坐标轴上
ax.yaxis.set_ticks_position('left')  # 将y轴刻度设在左边的坐标轴上
ax.spines['bottom'].set_position(('data', 0))  # 将两个坐标轴的位置设在数据点原点
ax.spines['left'].set_position(('data', 0))


# ————————————————/


# x_stop_center = 216
# y_stop_center = 70
x_stop_center, y_stop_center = serialPart.get_avg_stop_point(ser)



def update(i):
    xyz_read = serialPart.read_one_period(ser)
    z_read, y_read, x_read = xyz_read.values()# order adjusted for the stick
    x = -(x_read - x_stop_center)  # 在输出值在100以内不再解三角函数算相对角度增量了，虽然更为合理，粗略第当作线性近似吧
    y = y_read - y_stop_center
    line.set_xdata(x)
    line.set_ydata(y)
    theta_z = abs(arctan(sqrt(x_read ** 2 + y_read ** 2) / z_read))
    return line,


def init():
    pass
    # line, = ax.plot(cos(2 * (t_now - 1.2)), sin(3 * t_now), '.', color='red', alpha=0.1)
    line.set_ydata(sin(3 * t_now))
    return line,
    #

    # if i > view_length:#comment when watching Lissajous-Figure
    #     ax.lines.pop(0)
    #     for j, dot in enumerate(ax.lines):
    #         # dot.Config(alpha=j / view_length)
    #         # dot.alpha=j/view_length
    #         dot.set(alpha=(j / view_length)**3)
    #     # ax.lines.remove(lines[0])# heard to be equivalent to ax.lines.pop(0)
    #     plt.xlim(t_now-view_length, t_now)#comment when watching Lissajous-Figure


ani = animation.FuncAnimation(fig=fig, func=update, init_func=init, interval=2, blit=True)
plt.show()
ser.close()  # 关闭串口
