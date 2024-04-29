# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path
import sys
from matplotlib import pyplot as plt
import math
from statistics import mean, median, mode, stdev, variance
import numpy as np 

from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import time
from typing import Callable

from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
import rosbag2_py


if os.environ.get('ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL', None) is not None:
    # This is needed on Linux when compiling with clang/libc++.
    # TL;DR This makes class_loader work when using a python extension compiled with libc++.
    #
    # For the fun RTTI ABI details, see https://whatofhow.wordpress.com/2015/03/17/odr-rtti-dso/.
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)
import rosbag2_py  # noqa



RESOURCES_PATH = '/home/cesar/dev/ae1932_build/bag'
showplots = False

def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def wait_for(
    condition: Callable[[], bool],
    timeout: Duration,
    sleep_time: float = 0.1,
):
    clock = Clock(clock_type=ClockType.STEADY_TIME)
    start = clock.now()
    while not condition():
        if clock.now() - start > timeout:
            return False
        time.sleep(sleep_time)
        return True

def localization_comparison():
    bag_path = RESOURCES_PATH + '/0327202412'
    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    
    #Cartographer Pose/Orientation info
    xpose = []
    ypose = []
    xorien = []
    yorien = []
    zorien = []
    worien = []
    
    #Gazebo Pose/Orientation info
    gzxpose = []
    gzypose = []
    gzxorien = []
    gzyorien = []
    gzzorien = []
    gzworien = []

    msg_counter = 0
    if showplots == True:
        plt.figure("Figure 1")
    
    #Get all info from topics into lists for comparisons
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic  == '/cartsynced_topic':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            xpose.append(msg.pose.position.x)
            ypose.append(msg.pose.position.y)
            xorien.append(msg.pose.orientation.x)
            yorien.append(msg.pose.orientation.y)
            zorien.append(msg.pose.orientation.z)
            worien.append(msg.pose.orientation.w)
                   
        if topic == '/gzsynced_topic':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            gzxpose.append(msg.pose.position.x)
            gzypose.append(msg.pose.position.y)
            gzxorien.append(msg.pose.orientation.x)
            gzyorien.append(msg.pose.orientation.y)
            gzzorien.append(msg.pose.orientation.z)
            gzworien.append(msg.pose.orientation.w)

        msg_counter += 1

    #Calculate X/Y Pose Error over a path 
    if showplots == True:
        plt.figure("X Pose Error")
    i= 0
    l = 0
    point = 0
    
    xerror = []
    yerror = []
    xorienerror = []
    yorienerror = []
    zorienerror = []
    worienerror = []
    
    points = []
    
    totalxerror = 0
    totalyerror = 0
    
    maxxerror = -1
    maxyerror = -1
    while i < len(gzxpose) and l < len(xpose):
        xerrorpoint = abs(xpose[i] - gzxpose[l])
        yerrorpoint = abs(ypose[i] - gzypose[l])
        xorienerrorpoint = abs(xorien[i] - gzxorien[l])
        yorienerrorpoint = abs(yorien[i] - gzyorien[l])
        zorienerrorpoint = abs(zorien[i] - gzzorien[l])
        worienerrorpoint = abs(worien[i] - gzworien[l])
        
        xerror.append(xerrorpoint)
        yerror.append(yerrorpoint)
        xorienerror.append(xorienerrorpoint)
        yorienerror.append(yorienerrorpoint)
        zorienerror.append(zorienerrorpoint)
        worienerror.append(worienerrorpoint)
        
        totalxerror = totalxerror + abs(xpose[i] - gzxpose[l])
        totalyerror = totalyerror + abs(ypose[i] - gzypose[l])
        points.append(point)
        
        if maxxerror < xerrorpoint:
            maxxerror = yerrorpoint
        if maxyerror < yerrorpoint:
            maxyerror = xerrorpoint
        
        
        i = i + 1
        l = l + 1
        point = point + 1
    if showplots == True: 
        plt.scatter(points, xerror, color='g')
    
        plt.figure("Y Pose Error")
        plt.scatter(points,yerror, color='b')
    
    averagexerror = totalxerror/point
    averageyerror = totalyerror/point
    
    # print("Average X pose error", averagexerror, "Average Y Pose error", averageyerror)
    # print("Average X pose max error", maxxerror, "Average Y Pose Max error", maxyerror)
    
    print("Average X pose error", mean(xerror), "Average Y Pose error", mean(yerror))
    print("Median X pose error", median(xerror), "Median Y Pose error", median(yerror))
    print("Mode X pose error", mode(xerror), "Mode Y Pose error", mode(yerror))
    print("MSE X pose",np.square(np.subtract(gzxpose,xpose)).mean() , "MSE Y Pose", np.square(np.subtract(gzypose, ypose)).mean())
    
    print("Average X Orientation error", mean(xorienerror), "Average Y Orientation error", mean(yorienerror), "Average Z Orientation error", mean(zorienerror), "Average W Orientation error", mean(worienerror))
    print("Median X Orientation error", median(xorienerror), "Median Y Orientation error", median(yorienerror), "Median Z Orientation error", median(zorienerror), "Median W Orientation error", median(worienerror))
    print("Mode X Orientation error", mode(xorienerror), "Mode Y Orientation error", mode(yorienerror), "Mode Z Orientation error", mode(zorienerror), "Mode W Orientation error", mode(worienerror))
    print("MSE X Orientation", np.square(np.subtract(gzxorien,xorien)).mean(), "MSE Y Orientation", np.square(np.subtract(gzyorien,yorien)).mean(), "MSE Z Orientation", np.square(np.subtract(gzzorien,zorien)).mean(), "MSE W Orientation", np.square(np.subtract(gzworien,worien)).mean())

    cart = list(zip(xpose, ypose)) # or just zip(X1, X2) in Python 2
    gz = list(zip(gzxpose, gzypose)) # or just zip(X1, X2) in Python 2
    
    i = 0
    l= 0
    totaldist = 0 
    count = 0

    #Calculate percentage of points that exceeded the 5 inch requirement
    while i < len(gz) and i < len(cart):
        if math.dist(gz[i], cart[i]) > .127:
            count = count + 1
        totaldist = totaldist + math.dist(gz[i], cart[i])
        i = i + 1

    while i < len(gz):
        while l < len(cart):
            #print(gz[i][0])
            if(gz[i][0] == cart[l][0]):
                #print(gz[i][0])
                totaldist = totaldist + math.dist((gz[i][1], gz[i][2]), (cart[l][1], cart[l][2]))
                count = count + 1
                break
            else:
                l = l + 1
        i = i + 1
    
    totaldist = totaldist/i #Average error distance in a path
    count = count/i
    print("Percentage of points that exceeded 5 inch requirement", count)
    print(totaldist)
    msg_counter = 0
    if showplots == True:
        plt.subplots_adjust(left=0.017, bottom=0.017,right=1.0, top=1.0, wspace=0.0 ,hspace=0.0)
        plt.show()
        
localization_comparison()
