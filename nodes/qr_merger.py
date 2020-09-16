#!/usr/bin/env python
from __future__ import print_function

import os
import time
import threading
import io
import traceback
import json
from math import sqrt, acos, pi, sin, cos

import numpy as np

import rospy
import std_srvs.srv
from std_srvs.srv import EmptyResponse
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Point

from ros_qr_tracker.msg import Percept
from ros_qr_tracker.srv import GetTargetPosition, GetTargetPositionResponse, LoadData, LoadDataResponse, SaveData, SaveDataResponse

class QRMerger():

    def __init__(self):

        rospy.init_node('qr_merger', log_level=rospy.DEBUG)

        self._lock = threading.RLock()

        self.matches1_topic = rospy.get_param("~match1topic", '/ros_qr_tracker1/matches')
        self.matches2_topic = rospy.get_param("~match2topic", '/ros_qr_tracker2/matches')

        self.last_msg = None

        self._t0 = None


        # Handle of the image subscriber.
        self._matches1_sub = rospy.Subscriber(self.matches1_topic, Percept, self._process_match)
        self._matches2_sub = rospy.Subscriber(self.matches2_topic, Percept, self._process_match)

        #ADDED VARIABLES
        self._tmpTargetReq = None
        self.markerDict = {}
        self.meanXDict = {}
        self.meanYDict = {}
        self.meanZDict = {}
        self.distMeanDict = {}

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        # getTarget pos
        rospy.Service('~get_target_pos', GetTargetPosition, self.get_target_pos)

        rospy.Service('~save_data', SaveData, self.save_data)
        rospy.Service('~load_data', LoadData, self.load_data)

        rospy.spin()
        # Start polling the sensors and base controller
        #self.rate = int(rospy.get_param("~rate", 30))
        #r = rospy.Rate(self.rate)
        #while not rospy.is_shutdown():
        #    # Publish all sensor values on a single topic for convenience
#       #      now = rospy.Time.now()
        #    r.sleep()


    def get_target_pos(self, msg):
        response = GetTargetPositionResponse()
        response.pos_x = 0
        response.pos_y = 0
        response.pos_z = 0
        with self._lock:
            self._tmpTargetReq = msg.data.replace('\t', ' ')
            if self._tmpTargetReq in self.markerDict:
                rospy.logwarn('Found target POSX %f, POSY %f', self.meanXDict[self._tmpTargetReq], self.meanYDict[self._tmpTargetReq])
                response.pos_x = self.meanXDict[self._tmpTargetReq]
                response.pos_y = self.meanYDict[self._tmpTargetReq]
                response.pos_z = self.meanZDict[self._tmpTargetReq]
            rospy.logwarn('Get position target "%s".', self._tmpTargetReq)
        
        return response

    def save_data(self, msg):
        with self._lock:
            filedata = json.dumps(self.markerDict, indent=2)
            with open(msg.filePath, 'w') as file:
                file.writelines([filedata])
                #file.writelines([])
            rospy.logwarn('Saved target data to "%s".', msg.filePath)
        return SaveDataResponse()

    def load_data(self, msg):
        with self._lock:
            filedata = ''
            with open(msg.filePath, 'r') as file:
                filedata = file.read(1024*1024)
            self.markerDict = json.loads(filedata)
            rospy.logwarn('Json loaded, calc means "%s".', msg.filePath)
            self.meanXDict = {}
            self.meanYDict = {}
            self.meanZDict = {}
            self.distMeanDict = {}
            for keydata in self.markerDict.iterkeys():
                pcount = 0
                sumx = 0
                sumy = 0
                sumz = 0
                for idx, val in self.markerDict[keydata].iteritems():
                    pcount+=1
                    sumx+=val[0]
                    sumy+=val[1]
                    sumz+=val[2]
                meanx = sumx/pcount
                meany = sumy/pcount
                meanz = sumz/pcount
                self.meanXDict[keydata] = meanx
                self.meanYDict[keydata] = meany
                self.meanZDict[keydata] = meanz
                self.distMeanDict[keydata] = {}
                for idx, val in self.markerDict[keydata].iteritems():
                    dx = val[0]-self.meanXDict[keydata]
                    dy = val[1]-self.meanYDict[keydata]
                    self.distMeanDict[keydata][idx] = dx*dx+dy*dy
            rospy.logwarn('Loaded target data to "%s".', msg.filePath)
        return LoadDataResponse()

    def _process_match(self, msg=None):
        try:
            qrString = msg.data

            if not qrString in self.markerDict:
                self.markerDict[qrString] = {}
                self.meanXDict[qrString] = 0
                self.meanYDict[qrString] = 0
                self.distMeanDict[qrString] = {}

            #calc marker pos
            posx = msg.posx
            posy = msg.posy
            posz = msg.posz

            rospy.logwarn("RECEIVED POSX: %f POSY: %f POSZ: %f", posx, posy, posz)

            if (len(self.markerDict[qrString])<10):
                self.markerDict[qrString][len(self.markerDict[qrString])] = [posx, posy, posz]
                pcount = 0
                sumx = 0
                sumy = 0
                sumz = 0
                for idx, val in self.markerDict[qrString].iteritems():
                    pcount+=1
                    sumx+=val[0]
                    sumy+=val[1]
                    sumz+=val[2]
                meanx = sumx/pcount
                meany = sumy/pcount
                meanz = sumz/pcount
                self.meanXDict[qrString] = meanx
                self.meanYDict[qrString] = meany
                self.meanZDict[qrString] = meanz
                for idx, val in self.markerDict[qrString].iteritems():
                    dx = val[0]-self.meanXDict[qrString]
                    dy = val[1]-self.meanYDict[qrString]
                    self.distMeanDict[qrString][idx] = dx*dx+dy*dy

            else:
                #confronto la distanza dalla media del nuovo punto con gli altri che ho inserito
                dx = posx-self.meanXDict[qrString]
                dy = posy-self.meanYDict[qrString]
                distMean = dx*dx+dy*dy

                maxdist = 0
                maxidx = 0
                for idx, val in self.distMeanDict[qrString].iteritems():
                    if val > maxdist:
                        maxidx = idx
                        maxdist = val
                if distMean< maxdist:
                    self.markerDict[qrString][idx] = [posx, posy, posz]
                    pcount = 0
                    sumx = 0
                    sumy = 0
                    sumz = 0
                    for idx, val in self.markerDict[qrString].iteritems():
                        pcount+=1
                        sumx+=val[0]
                        sumy+=val[1]
                        sumz+=val[2]

                    meanx = sumx/pcount
                    meany = sumy/pcount
                    meanz = sumz/pcount
                    self.meanXDict[qrString] = meanx
                    self.meanYDict[qrString] = meany
                    self.meanZDict[qrString] = meanz
                    for idx, val in self.markerDict[qrString].iteritems():
                        dx = val[0]-self.meanXDict[qrString]
                        dy = val[1]-self.meanYDict[qrString]
                        self.distMeanDict[qrString][idx] = dx*dx+dy*dy

            rospy.logwarn("DICT LEN: %d ", len(self.markerDict[qrString]))
            rospy.logwarn("POS MEAN X: %f - POS MEAN Y: %f - POS MEAN Z: %f", self.meanXDict[qrString], self.meanYDict[qrString], self.meanZDict[qrString])


        except rospy.exceptions.ROSInterruptException:
            pass
        finally:
            if not msg:
                self.running = False


    def shutdown(self):
        rospy.loginfo('Shutting down...')
        self.running = False
        rospy.loginfo('Done.')


if __name__ == '__main__':
    QRMerger()

