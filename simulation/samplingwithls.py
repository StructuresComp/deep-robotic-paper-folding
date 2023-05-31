import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import os
import time
import subprocess
from multiprocessing import Pool

import math

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def computeAngle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

#############################################################
# RunBASim function will simply execute a single command
def RunBASim(cmdline):
    BASimProcess = subprocess.Popen(cmdline, shell=True)
    BASimProcess.communicate()
#############################################################


nProcesses = 30  #Number of processes to be run simultaneously
last_time = time.time()

# compute Lgb
density = 1000;
youngM = 1.8e6;
rodRadius = 1.6e-3;
Lgb = (youngM * pow(rodRadius,2)/(8*density * 10))**(1.0/3);

sampleRate = 1e-2 # set up sampling rate
Ls = np.linspace(0.01, 0.25, 25)
Ls = Ls/Lgb

filecontent = []
for i, ls in enumerate(Ls):
   cmdL = "./simBC optionBC.txt -- hangLength {:}".format(ls)
   filecontent.append(cmdL)

p = Pool(processes = nProcesses)
p.map(RunBASim, filecontent)
now = time.time() - last_time
minute = now%60


# pre processing
parseDir = "parse"
if os.path.isdir(parseDir):
    print("Directory exists.")
else:
    os.mkdir("parse")
    print("Directory created.")
count = 0
for i, ls in enumerate(Ls):
    data = np.loadtxt("bcfiles/simDER{:.4g}.txt".format(ls))
    thetaV = data[:, 0]
    up = data[:, 1:4]
    bot = data[:, 4:7]

    n = 0
    # filter the wrong data
    while n < up.shape[0]:
        upDis = up[n, :]
        botDis = bot[n, :]
        upDis = np.sqrt(np.sum(upDis**2))
        botDis = np.sqrt(np.sum(botDis**2))

        if upDis > ls or botDis > ls:
            up = np.delete(up, n, axis = 0)
            bot = np.delete(bot, n, axis = 0)
            thetaV = np.delete(thetaV, n)
        else:
            n = n + 1
    # dense point
    gap = int((thetaV[-1] - thetaV[0])/(sampleRate/ls))
    thetaI = np.linspace(thetaV[0], thetaV[-1], gap)
    upD = np.zeros((gap, 2), dtype = np.float32)
    upD[:, 0]= np.interp(thetaI, thetaV, up[:, 0])
    upD[:, -1] = np.interp(thetaI, thetaV, up[:, -1])

    botD = np.zeros((gap, 2), dtype = np.float32)
    botD[:, 0]= np.interp(thetaI, thetaV, bot[:, 0])
    botD[:, -1] = np.interp(thetaI, thetaV,  bot[:, -1])

    middleD = (upD + botD)/2

    upD = 2*(upD - middleD) + middleD
    botD = 2*(botD - middleD) + middleD
    parsePoint = []

    for i in range(middleD.shape[0]):
        px = np.linspace(upD[i, :], botD[i, :], int(0.05/sampleRate))
        for j in range(px.shape[0]):
            parsePoint.append(px[j, :])
    parsePoint = np.asarray(parsePoint)
    # plt.plot(parsePoint[:, 0], parsePoint[:, 1])
    # plt.plot(up[:, 0], up[:, 2])
    # plt.plot(bot[:, 0], bot[:, 2])
    # plt.show()
    count = count + parsePoint.shape[0]
    np.savetxt("parse/parsePoint_{:.4g}.txt".format(ls), parsePoint)

# sampling forces
filecontent = []
for i, ls in enumerate(Ls):
   parsefile = "parse/parsePoint_{:.4g}.txt".format(ls)
   cmdL = "./simF optionF.txt -- hangLength {:} -- filename {}".format(ls, parsefile)
   filecontent.append(cmdL)
p = Pool(processes = nProcesses)
p.map(RunBASim, filecontent)

now = time.time() - last_time
print ('time taken to run simulations of find Boundary : {}', now)



## post process Data
# count = 0
# for i, ls in enumerate(Ls):
#     data = np.loadtxt("datafiles/force_hangLength_{:.4g}.txt".format(ls))
#     print(data.shape)
#     mP1 = data[:,0:3]
#     mP2 = data[:,3:6]
#     Fs1 = data[:,6:9]
#     Fs2 = data[:,9:12]
#     zoffset = data[:, 12]
#     Fs = Fs1 + Fs2
#     Ft = np.abs(Fs[:, 0])
#     Fn = np.abs(Fs[:, 2])
#     a = Fs1[:, 2] >=0
#     b = Fs2[:, 2] >=0
#     c = zoffset >=0
#     idx = (a&b&c)
#
#     T = (mP1 - mP2)
#     T = T/(np.sqrt(np.sum(T**2, 1))).reshape(-1, 1)
#     x_axis = np.array([1, 0, 0])
#     z_axis = np.array([0, 0, 1])
#
#     cvalues = np.sum(T * x_axis, axis = 1)
#     svalues = np.sum(T * z_axis, axis = 1)
#
#     alpha = np.arccos(cvalues)
#     alpha[svalues <=0] = 2*np.pi - alpha[svalues <=0]
#     ratio = Ft/Fn
#     saveData = np.hstack((mP1[:,0].reshape(-1,1), mP1[:,2].reshape(-1,1), ratio.reshape(-1,1), alpha.reshape(-1,1)))
#     saveData = saveData[idx, :]
#     count = count + saveData.shape[0]
#     # plt.scatter(saveData[:, 0], saveData[:, 1], 1, saveData[:, 2])
#     # # plt.plot(saveData[:, 0], saveData[:, 2])
#     # plt.show()
# print(count)
# np.savetxt("sampleData_{:g}.txt".format(ls), saveData)
