'''
This script is used to plot csv data, the function of boxplot require python>3.7, 
if failed, please comment this line and use the second line.
'''

import numpy as np
import matplotlib.pyplot as plt

#======================CONFIGURATIONS====================================
plotBw=1  #if plot all files related to transport
plotCpu=0   #if plot all files related to CPU
plotRates=0 #if plot all files related to decompression rate 

plotinfo=[("png_rate","PNG compression level","Frame per second [fps]","PNG rate"),
           ("png_bandwidth","PNG compression level","bandwidth [Mb/s]","PNG bandwidth"),
           ("jpg_rate","JPEG quality ","Frame per second [fps]","JPG rate"),
           ("jpg_bandwidth","JPEG quality ","bandwidth [Mb/s]","JPG bandwidth"),
           ("theora_rate","Theora quality ","Frame per second [fps]","THEORA rate"),
           ("theora_bandwidth","Theora quality ","bandwidth [Mb/s]","THEORA bandwidth"),
           ("raw_rate","Measure sample ","Frame per second [fps]","RAW rate"),
           ("raw_bandwidth","Measure sample","bandwidth [Mb/s]","RAW bandwidth"),
           ]
plot_simple_info=[("png_cpu_use_compress","Time [s]","cpu usage [%]","PNG compression cpu usage on pepper"),
           ("theora_cpu_use_compress","Time [s]","cpu usage [%]","THEORA compression cpu usage on pepper"),
           ("jpg_cpu_use_compress","Time [s]","cpu usage [%]","JPEG compression cpu usage on pepper"),
            ]
plot_decompress_info=[("png_rate_decompress","Time [s]","rate [fps]","PNG decompression rate on distant machine"),
           ("theora_rate_decompress","Time [s]","rate [fps]","THEORA decompression rate on distant machine"),
           ("jpg_rate_decompress","Time [s]","rate [fps]","JPEG decompression rate on distant machine"),
            ]
#=====================================================================



#Function to plot all files related to transport
def csv2figure(csvName,xlabel,ylabel,title):
    plt.clf()
    y = np.loadtxt(csvName+".csv", delimiter=',')
    x  = y[:,0].flatten()
    #skip the two measurements at two ends which are sometimes not proprely registred 
    plt.boxplot(y[:,2 :-1].transpose(), labels=[f"{lx:.0f}" for lx in x])  #require python>3.7
    #plt.boxplot(y[:,2 :-1].transpose(), labels=[f"{:.0f}".format(lx) for lx in x]) #for lower version
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.savefig(csvName+".png", dpi=200)
    print("figure is saved")



#Function to plot all files related to cpu and decompression rate
def csv2figure_simple(csvName,xlabel,ylabel,title):
    plt.clf()
    y = np.loadtxt(csvName+".csv", delimiter=',')[1:]
    x = range(len(y))
    plt.plot(x,y)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.savefig(csvName+".png", dpi=200)
    print("figure is saved")


if __name__ == '__main__':
    if plotBw:
        for csvFile in plotinfo:
            try:
                csv2figure(csvFile[0],csvFile[1],csvFile[2],csvFile[3])
            except IOError:
                print("File doesn't exist")
    if plotCpu:
        for csvFile2 in plot_simple_info:
            try:
                csv2figure_simple(csvFile2[0],csvFile2[1],csvFile2[2],csvFile2[3])
            except IOError:
                print("File doesn't exist")
    if plotRates:
        for csvFile3 in plot_decompress_info:
            try:
                csv2figure_simple(csvFile3[0],csvFile3[1],csvFile3[2],csvFile3[3])
            except IOError:
                print("File doesn't exist")
