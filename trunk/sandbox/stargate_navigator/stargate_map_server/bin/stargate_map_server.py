#!/usr/bin/env python

""" Stargate Navigation System """

import roslib; roslib.load_manifest('stargate_map_server')
import rospy

import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from stargate_map_server.msg import *
from stargate_map_server.srv import *

###############################################################################
# Base classes
class map_data:
    """ Base class for all map data. """
    def __init__(self, name):
        self.name = name                # unique identifier

###############################################################################
# Data Storage
class sector_data(map_data):
    """ A sector is a location. Sectors store their gateways locally. """
    gateways = dict()

    def __init__(self, name, parent = None):
        map_data.__init__(self, name)
        self.parent = parent            # unique identifier of parent

    def addGateway(self, gateway):
        self.gateways.append(gateway)
        
class gateway_data(map_data):
    """ A gateway connects sectors. Most connect only two sectors, but some,
          like stairwells or elevators can connect many more. """
    def __init__(self, name, gateway_type, sector_pose):
        map_data.__init__(self, name)
        self.sector_pose = sector_pose  # a dictionary of poses within sectors
        self.gateway_type = gateway_type

    def connects(self):
        """ Returns a list of sectors that this gateway connects. """
        return self.sector_pose.keys()

    def getPose(self, sector):
        try:
            return self.sector_pose[sector]
        except:
            return None
    
    def getSectors(self, nolist):
        sectors = list()
        for s in self.sector_pose.keys():
            if not s in nolist:
                sectors.append(String(s))
        return sectors
    
class identifier_data(map_data):
    """ An identifier can be used to localize within a sector. """
    def __init__(self, name, sector, pose):
        map_data.__init__(self, name)
        self.parent = sector            # unique identifier of sector containing this
        self.pose = pose                # approximate pose within the sector

    def getPose(self, sector):
        if sector == self.parent:
            return self.pose
        else:
            return None
    
###############################################################################
# The whole map, full of shenanigans 
class stargate_map:
    """ The map contains a list of sectors, gateways and identifiers. """
    sectors = dict()            # key = sector name, value = object
    gateways = dict()           # key = gateway name, value = object
    identifiers = dict()        # key = identifer name, value = object

    def __init__(self, map_file):
        """ Load our map, run initialization. """
        self.current_sector = ""
        # load the map!
        for line in open(map_file).readlines():
            el = line.split(",")
            if el[0] == "S":
                # it's a sector! 
                self.sectors[el[1]] = sector_data(el[1], el[2])
            elif el[0] == "G":
                # it's a gateway!
                self.gateways[el[1]] = gateway_data(el[1], el[2], self.extractPoses(el[3:]))
                # add to relevant sectors!
                
            elif el[0] == "I":
                # it's an identifier
                sector, pose = self.extractPoses(el[2:]).items()[0]
                self.identifiers[el[1]] = identifier_data(el[1], sector, pose)
            elif el[0] == "B":
                # beginning point
                self.current_sector, self.sector_pose = self.extractPoses(el[1:]).items()[0]
                print "Currently at:", self.current_sector, self.sector_pose    
        # do broadcast of tf
        self.br = tf.TransformBroadcaster()

    def extractPoses(self, l):
        """ Extract names and poses from map file line. """
        poses = dict()
        while len(l) >= 7:
            p = Pose()
            p.position.x = float(l[1])
            p.position.y = float(l[2])
            p.position.z = float(l[3])
            q = quaternion_from_euler(float(l[4]),float(l[5]),float(l[6]))
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            poses[l[0]] = p
            l = l[7:]
        return poses

    def getSectorMap(self, sector):
        """ Find the map of identifiers and gateways in this sector. """
        sectorMap = dict()
        for x in self.gateways.values()+self.identifiers.values():
            p = x.getPose(sector)
            if p != None:
                sectorMap[x.name] = p
        return sectorMap

    def updateTF(self):
        """ Post TF updates for the current map sector. """
        # need to post that the current sector is the base of our map
        self.br.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),self.current_sector+"_map","map")
        # need to post the identifiers (for localization) and the gateways (mostly for visualization) 
        for n,p in self.getSectorMap(self.current_sector).items():
            self.br.sendTransform((p.position.x,p.position.y,p.position.z),(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w),rospy.Time.now(),n+"_map",self.current_sector+"_map")

    def setSectorCb(self, request):
        """ Service to set the current sector. """
        if request.sector.data in self.sectors.keys():
            self.current_sector = request.sector.data
            #self.current_pose = request.pose
        return SetSectorResponse(String(self.current_sector))

    def getSectorMapCb(self, req): 
        """ Service to get the map of gateways in this sector. """
        sectorMap = list()
        print req.sector.data
        for x in self.gateways.values():
            p = x.getPose(req.sector.data)
            if p != None:
                g = Gateway()
                g.sectors = x.getSectors([req.sector.data, ])
                g.pose = PoseStamped()
                g.pose.header.frame_id = req.sector.data
                g.pose.pose = p
                sectorMap.append(g)
        return GetSectorMapResponse(sectorMap)

if __name__ == "__main__":  
    rospy.init_node('stargate_map_server')
    s = stargate_map(rospy.get_param("~mapfile", "ILS.map"))

    # set the current sector - for the locater, when operating in kidnapped robot mode
    rospy.Service('stargate_map_server/SetSector',SetSector,s.setSectorCb)
    # get the gateway map for a sector
    rospy.Service('stargate_map_server/GetSectorMap',GetSectorMap,s.getSectorMapCb)
    
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        s.updateTF()
        r.sleep()

