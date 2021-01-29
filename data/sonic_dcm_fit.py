#!/usr/bin/python
# -*- coding: utf-8 -*-
# import sys
# sys.path.append("../")
# author : sgl
# time: 2020-06-26

import matplotlib.pyplot as plt 
import numpy as np
from scipy.optimize import curve_fit


# final result:  D(mm) = 0.1938 * x + 53.836
#  limitation: 47,1040 mm
# not synchronization, setup distance 25cm; synchronized：10cm ok.
"""
s1(ul): 
[ 0.19379818 50.71229465]
[   5.15996487 -261.65767125]
s2(ur):
[ 0.18776606 44.71181006]
[   5.32558705 -238.04358226]
s3(dl): 
[ 0.18651286 51.92410095]
[   5.36145324 -278.34761382]
s4(dr):
[ 0.19253614 45.97917378]
[   5.19363456 -238.72483811]
"""

fname = "/home/sgl/catkin_new/src/sonar_array/data/dcm_new_data.txt"

limitation = [47, 1040] #mm

def fit_func(x, a, b):
    return a*x + b

# sonic frequency: 111-44=67  ; 60hz
x1_list=[]
x2_list=[]
x3_list=[]
x4_list=[]
y_list=[]
with open(fname) as f:
    # l = f.readline()
    for l in f:
        lt=l.split()
        # print(lt)
        x1 = int(lt[0])#*10  # here transferred to mm
        x2 = int(lt[1])#*10  # here transferred to mm
        x3 = int(lt[2])#*10  # here transferred to mm
        x4 = int(lt[3])#*10  # here transferred to mm
        y = int(lt[4])
        # print(y)
        x1_list.append(x1)
        x2_list.append(x2)
        x3_list.append(x3)
        x4_list.append(x4)
        y_list.append(y)

p0 = np.polyfit(x1_list, y_list, 1)
p1 = np.polyfit(x2_list, y_list, 1)
p2 = np.polyfit(x3_list, y_list, 1)
p3 = np.polyfit(x4_list, y_list, 1)
print("ranges[0] = "+ str(p0[0]) + " * ranges[0] + " + str(p0[1]) + ";")
print("ranges[1] = "+ str(p1[0]) + " * ranges[1] + " + str(p1[1]) + ";")
print("ranges[2] = "+ str(p2[0]) + " * ranges[2] + " + str(p2[1]) + ";")
print("ranges[3] = "+ str(p3[0]) + " * ranges[3] + " + str(p3[1]) + ";")

params0 = curve_fit(fit_func, x1_list, y_list)
params1 = curve_fit(fit_func, x2_list, y_list)
params2 = curve_fit(fit_func, x3_list, y_list)
params3 = curve_fit(fit_func, x4_list, y_list)
print(params0[0])
print(params1[0])
print(params2[0])
print(params3[0])


# t1=range(0,len(d1_list))
# t2=range(0,len(d2_list))
# t3=range(0,len(d3_list))
# t4=range(0,len(d4_list))
# f, ax1 = plt.subplots(1,1)
# # ax1 = fig1.add_subplot(111, projection='3d')
plt.figure()
plt.xlabel("s")
plt.ylabel("distance(mm)")
plt.plot(x1_list, y_list,'o-')
plt.plot(x2_list, y_list,'*-')
plt.plot(x3_list, y_list,'-')
plt.plot(x4_list, y_list,'--')

plt.show()


