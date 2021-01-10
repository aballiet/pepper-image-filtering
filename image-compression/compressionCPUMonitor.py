'''
This script is used to monitor the impact of compression of different method on ressources of calculations 
on robot Pepper and also on remote PC
Please change the varaibles "onPepper" and "onDistance" according to the used platform
'''

from subprocess import Popen, PIPE
import rospy,rostopic
import psutil
import numpy as np
from datetime import datetime
import time

#===========Configuration before run script================================#
monitoring_duration=1*60 #monitoring duration in seconds
onPepper=1# if this script run on pepper
onDistance=0 # if this script run on distant machine
choice_compression_method=1 # 0,1,2 corresponding compression_method index
check_raw=1  #if want to check the raw topic rate
#===========================================#
compression_method=["png","jpg","theora"]
#raw_topic_name="pepper_robot/camera/front/image_raw"  #raw topic used to do a first measure when blurry filtered topic is not present
raw_topic_name="pepper_robot/camera/front/image_blurry_filtered"   # raw topic after filtered, use this for real application
compress_topic_name="pepper_robot/camera/front/image_repub_compressed"
decompress_topic_name="pepper_robot/camera/front/decompressed"
#IMPORTANT:on peper, version of python 2.7.15, psutil.Process(PID).cpu_percent(interval=2) 
#             on distant machine, python 2.7.6,   psutil.Process(PID).get_cpu_percent(interval=2) 
global i,t1,rate_list  #i: counter of frames per duration, t1: start time stamp of each measurement, rate_list: list of each rate mesurement 
duration=1 #duration by seconds of each measurement 

#Function to monitor filtered raw image reception rate 
def monitorRaw():
    s=rospy.Subscriber(raw_topic_name, rospy.AnyMsg, callback)
    rospy.sleep(monitoring_duration)
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    np.savetxt(current_time+"_raw_rate.csv", rate_list, delimiter=",")
    print("finished save===========================================")
    
#Callback of subscribed topic, measuring reception rate 
def callback(data):
    global i, t1,rate_list
    i += 1
    t_delta = time.time() - t1
    if(t_delta>=duration):
        t1 = time.time()
        rate=i/t_delta
        rate_list.append(rate)
        i = 0
        print("i received de/compressed photo, rate = "+str(rate))

#if onPepper:   Monitoring compression rate on pepper and CPU usage of compression on pepper during the assigned monitoring duration
#if onDistantMachine:   Monitoring decompression rate on distant machine and CPU usage of decompression on it during the assigned monitoring duration
def monitor():
    start = time.time()
    if onPepper:
        if(choice_compression_method not in [0,1,2]):
            print("please change choice_compression_method (0, 1, 2)");
            return

        #Create a republish topic with name <compress_topic_name>/compressed which compress filtered images with compress method jpeg or png
        elif (choice_compression_method==1 or choice_compression_method==0):
            proc= Popen("rosrun image_transport republish raw in:="+raw_topic_name+" compressed out:="+compress_topic_name,shell=True, stdout=PIPE, stderr=PIPE)
            s=rospy.Subscriber(compress_topic_name+"/compressed", rospy.AnyMsg,  callback)

        #Create a republish topic which with name <compress_topic_name>/theora compress filtered images with method Theora
        else:
            proc= Popen("rosrun image_transport republish raw in:="+raw_topic_name+" theora out:="+compress_topic_name,shell=True, stdout=PIPE, stderr=PIPE)
            s=rospy.Subscriber(compress_topic_name+"/theora", rospy.AnyMsg,  callback)

        #measure process of compression's CPU usage on pepper
        p = psutil.Process(proc.pid)
        l = []
        while time.time()-start<monitoring_duration:
            cpu_use=p.cpu_percent(interval=2) #the pourcentage of cpu use (cpu time) for compression process on pepper robt
            now = datetime.now()
            current_time = now.strftime("%H:%M:%S")
            l.append(cpu_use)
            print("elapsed time="+str(time.time()-start)+" , cpu use= "+str(cpu_use))

        np.savetxt(compression_method[choice_compression_method]+"_cpu_use_compress.csv", l, delimiter=",")
        np.savetxt(compression_method[choice_compression_method]+"_rate_compress.csv", rate_list, delimiter=",")
        print("finished save===========================================")

    if onDistance:
        if(choice_compression_method not in [0,1,2]):
            print("please change choice_compression_method (0, 1, 2)");
            return

        #Create a republish topic with name <decompress_topic_name> which decompress compressed images with method jpeg or png
        elif (choice_compression_method==1 or choice_compression_method==0):
            proc= Popen("rosrun image_transport republish compressed in:="+compress_topic_name+" raw out:="+decompress_topic_name,shell=True, stdout=PIPE, stderr=PIPE)
            s = rospy.Subscriber(decompress_topic_name,rospy.AnyMsg,  callback)

        #Create a republish topic with name <decompress_topic_name> which decompress compressed images with method theora
        else:
            proc= Popen("rosrun image_transport republish theora in:="+compress_topic_name+" raw out:="+decompress_topic_name,shell=True, stdout=PIPE, stderr=PIPE)
            s = rospy.Subscriber(decompress_topic_name, rospy.AnyMsg,  callback)
            
        #measure process of compression's CPU usage on distant machine
        p = psutil.Process(proc.pid)
        l = []
        while time.time()-start<monitoring_duration:
            cpu_use=p.get_cpu_percent(interval=2) #the pourcentage of cpu use (cpu time) for decompression process on distant machine

            l.append(cpu_use)
            print("elapsed time="+str(time.time()-start)+" , cpu use= "+str(cpu_use))

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        np.savetxt(compression_method[choice_compression_method]+"_cpu_use_decompress.csv", l, delimiter=",")
        np.savetxt(compression_method[choice_compression_method]+"_rate_decompress.csv", rate_list, delimiter=",")
        print("finished save===========================================")



if __name__ == '__main__':
    
    rate_list=[]
    i = 0
    t1 = time.time()
    rospy.init_node("Monitor")
    if check_raw:
        monitorRaw()
    else:
        monitor()

    rospy.spin()
