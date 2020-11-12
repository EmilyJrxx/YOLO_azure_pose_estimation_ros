import turtle as tl
import math

'''
作者：北理咕小头
简介：
    这是一个导入图形傅里叶级数信息，并利用这些级数通过turtle复原图形的程序。
程序：
    1.读取傅里叶级数信息，解析，并放入data列表中
    2.用data列表中的参数带入傅里叶级数的方程求得二维坐标
    3.用turtle依次走过这些坐标达到绘图的效果
    4.可以绘制多条路径
'''

data = []
points = 5000 #不同精度的图片绘制点数不同

N = 1000 + 1 # N由上个程序中计算出的级数数量决定，加1是因为有一个角速度为0的量（直流分量）
x = [0] * N
y = [0] * N
rob_x = [0] * 5000
rob_y = [0] * 5000
rob_x_min=0
rob_x_max=0
rob_y_min=0
rob_y_max=0
f = open("datas0"+".txt","r")
tl.penup()
tl.pensize(2)  # 画笔粗细
for line in f:
    line = eval(line)
    data.append(line)

    # tl.setup(960,720)

    # 储存原始代码的电脑因新型肺炎疫情被隔离了，这是我根据印象重新做的，可能存在错误，疫情结束后会更正。
    # 三角函数中的值是n * 2 * pi * t , 其中n取0，1，-1，2，-2……，t的范围是[0,1]，当然t取大了没关系，会重复描已经画好的图形
for t in range(points):
    for i in range(len(data)):
        if i % 2 == 0:
            x[i] = data[i][0] * math.cos(i / points * 3.14 * t) - data[i][1] * math.sin(i / points * 3.14 * t)
            y[i] = data[i][0] * math.sin(i / points * 3.14 * t) + data[i][1] * math.cos(i / points * 3.14 * t)
        else:
            x[i] = data[i][0] * math.cos(-(i+1) / points * 3.14 * t) - data[i][1] * math.sin(-(i+1) / points * 3.14 * t)
            y[i] = data[i][0] * math.sin(-(i+1) / points * 3.14 * t) + data[i][1] * math.cos(-(i+1) / points * 3.14 * t)

    rob_x[t] = sum(x)/3750
    rob_y[t] = sum(y)/3750
    print("rob_x[(%d)]= %f"%(t,rob_x[t]))
    print("rob_y[(%d)]= %f"%(t,rob_y[t]))
    if rob_x_max < rob_x[t]:
        rob_x_max=rob_x[t]
    if rob_x_min>rob_x[t]:
        rob_x_min=rob_x[t]
    if rob_y_max<rob_y[t]:
        rob_y_max=rob_y[t]
    if rob_y_min>rob_y[t]:
        rob_y_min=rob_y[t]
    tl.goto(int(sum(x))/2, -int(sum(y)/2)) # 正负可以控制图形的左右镜像，上下镜像,乘除可以控制缩放
    tl.pendown()

print("rob_x_min= %f"%(rob_x_min))
print("rob_x_max= %f"%(rob_x_max))
print("rob_y_min= %f"%(rob_y_min))
print("rob_y_max= %f"%(rob_y_max))
