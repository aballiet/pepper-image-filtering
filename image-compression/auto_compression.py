'''
This script is used to measure the impact of compression of different method and compression level 
on transport between the robot Pepper and a remote PC.
Please run this script on remote PC
'''

import rospy,rostopic

import time, sys
import numpy as np
from pympler.asizeof import asizeof
from subprocess import Popen, PIPE

global i, dataSize, t1, temp_bw, temp_rate

#==============CONFIGURATIONS==============================#
topicName="pepper_robot/camera/front/image_raw"
duration=2  #the duration of each measurement 
mes_times=10  #number of measurement for each mode

raw_times=5  #number of measurement for raw image
png_levels = range(1,10)
jpeg_qualities = range(10,100,10)
theora_qualities=range(10,64,5)

#choose type of images to measure
measureRaw=1
measureJPG=0
measurePNG=0
measureTheora=0
#=============================================================#

#Calculate bandwidth and reception rate on topics
def callback(data):
    global i, dataSize, t1, temp_bw, temp_rate  # counters and recorders of measurement of each msg
    i += 1
    dataSize += asizeof(data)
    t_delta = time.time() - t1
    if(t_delta>=duration):
        bw = dataSize/t_delta/1024**2
        rate = i/t_delta
        t1 = time.time()
        i = 0
        dataSize = 0
        temp_bw.append(bw)
        temp_rate.append(rate)
        print("average rate="+str(rate))
        print("bw="+str(bw)+"Mb/s")
        print("bw/rate="+str(100*bw/rate**2))
        print("===========================")


if __name__ == '__main__':
    i = 0
    dataSize = 0
    t1 = time.time()
    rospy.init_node('myCompressor')
    temp_bw, temp_rate = [], []
#Measure raw
    if measureRaw:
        #Subscribe to the topic <topicName> which sends raw images
        s = rospy.Subscriber(topicName, rospy.AnyMsg, callback)
        print("raw start")
        raw_bandwidth = np.zeros((raw_times,mes_times+1))
        raw_rate = np.zeros((raw_times,mes_times+1))
        
        for i in range(raw_times):
            temp_bw, temp_rate = [], []
            rospy.sleep(duration*5)  #each raw measurement lasts 5*duration seconds, callback is called 5 times 
            raw_bandwidth[i,1:len(temp_bw)+1] = np.array(temp_bw)
            raw_rate[i,1:len(temp_rate)+1] = np.array(temp_rate)
            raw_bandwidth[i,0]=i;
            raw_rate[i,0]=i;
            print("raw_time="+str(i)+"==============fini==============")
            np.savetxt("raw_bandwidth.csv", raw_bandwidth, delimiter=",")
            np.savetxt("raw_rate.csv", raw_rate, delimiter=",")	

#Measure Compressed
    #Subscribe to the topic <topicName>/compressed which sends compressed(jpg/png) images
    s = rospy.Subscriber(topicName+"/compressed", rospy.AnyMsg, callback)
    #measure png
    if measurePNG:
        #Set compression type 'png' dynamically on topic <topicName>/compressed 
        proc = Popen("rosrun dynamic_reconfigure dynparam set " +topicName+"/compressed"+  "format 'png'", shell=True, stdout=PIPE, stderr=PIPE)
        print("png start")
        png_bandwidth = np.zeros((len(png_levels),mes_times+1))
        png_rate = np.zeros((len(png_levels),mes_times+1))
        #Set compression level dynamically on topic <topicName>/compressed 
        for counter, w in enumerate(png_levels):
            temp_bw, temp_rate = [], []
            proc = Popen("rosrun dynamic_reconfigure dynparam set " +topicName+"/compressed"+ " png_level "+str(w), shell=True, stdout=PIPE, stderr=PIPE)
            rospy.sleep(duration*mes_times)
            png_bandwidth[counter,1:len(temp_bw)+1] = np.array(temp_bw)
            png_rate[counter,1:len(temp_rate)+1] = np.array(temp_rate)
            png_bandwidth[counter,0]=w;
            png_rate[counter,0]=w;
            print("png_level="+str(w)+"==============fini==============")
        np.savetxt("png_bandwidth.csv", png_bandwidth, delimiter=",")
        np.savetxt("png_rate.csv", png_rate, delimiter=",")

    #Measure jpeg
    if measureJPG:
        #Set compression type 'jpeg' dynamically on topic <topicName>/compressed 
        proc = Popen("rosrun dynamic_reconfigure dynparam set "+ topicName+"/compressed"+" format 'jpeg'", shell=True, stdout=PIPE, stderr=PIPE)
        print("jpeg start")
        jpg_bandwidth = np.zeros((len(jpeg_qualities),mes_times+1))
        jpg_rate = np.zeros((len(jpeg_qualities),mes_times+1))
         #Set compression level dynamically on topic <topicName>/compressed 
        for counter, j in enumerate(jpeg_qualities):
            temp_bw, temp_rate = [], []
            proc = Popen("rosrun dynamic_reconfigure dynparam set " + topicName+"/compressed"+ " jpeg_quality "+str(j), shell=True, stdout=PIPE, stderr=PIPE)
            rospy.sleep(duration*mes_times)
            jpg_bandwidth[counter,1:len(temp_bw)+1] = np.array(temp_bw)
            jpg_rate[counter,1:len(temp_rate)+1] = np.array(temp_rate)
            jpg_bandwidth[counter,0]=j;
            jpg_rate[counter,0]=j;
            print("jpeg_quality="+str(j)+"==============fini==============")
        np.savetxt("jpg_bandwidth.csv", jpg_bandwidth, delimiter=",")
        np.savetxt("jpg_rate.csv", jpg_rate, delimiter=",")
    

#Measure Theora
    if measureTheora:
        #Subscribe to topic <topicName>/theora which sends compressed (theora) images
        s = rospy.Subscriber(topicName+"/theora", rospy.AnyMsg,  callback)
        #Set compression type 'theora', and choose optimize for 'quality' dynamically on topic <topicName>/theora
        proc = Popen("rosrun dynamic_reconfigure dynparam set " +topicName+"/theora"+  "optimize_for 1", shell=True, stdout=PIPE, stderr=PIPE)
        print("theora start")
        theora_bandwidth = np.zeros((len(theora_qualities),mes_times+1))
        theora_rate = np.zeros((len(theora_qualities),mes_times+1))
        #Set compression level dynamically on topic <topicName>/theora 
        for counter, j in enumerate(theora_qualities):
            temp_bw, temp_rate = [], []
            proc = Popen("rosrun dynamic_reconfigure dynparam set " +topicName+"/theora"+ " quality "+str(j), shell=True, stdout=PIPE, stderr=PIPE)
            rospy.sleep(duration*mes_times)
            theora_bandwidth[counter,1:len(temp_bw)+1] = np.array(temp_bw)
            theora_rate[counter,1:len(temp_rate)+1] = np.array(temp_rate)
            theora_bandwidth[counter,0]=j;
            theora_rate[counter,0]=j;
            print("quality="+str(j)+"==============fini==============")

        np.savetxt("theora_bandwidth.csv", theora_bandwidth, delimiter=",")
        np.savetxt("theora_rate.csv", theora_rate, delimiter=",")
    rospy.spin()
