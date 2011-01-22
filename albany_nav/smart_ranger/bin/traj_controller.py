#!/usr/bin/env python

import roslib; roslib.load_manifest('smart_ranger')
import rospy

import tf
from PIL import Image, ImageDraw
from math import sin, cos, radians, pi
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import GridCells

class CostMap:
    """ A 2d cost map for local planning, as defined in PML paper. """

    def __init__(self, width=8.0, height=8.0, resolution=0.1, robot_dia=1.0):
        """ Create a new cost map: 
            width - width of the map, in meters.
            height - height of the map, in meters.
            resolution - resolution of each cell, in meters.
            laser_offset - x offset of laser, in meters.  """
        self.last_update = rospy.Time.now()
        self.grid = Image.new("1",(int(width/resolution),int(height/resolution)), 50)
        #self.grid = np.ones( ((width/resolution), (height/resolution)), dtype='int' ) * 50
        self.width = width
        self.height = height
        self.resolution = resolution
        self.rad = robot_dia/2.0
        self.listener = tf.TransformListener()
        # robot pose
        self.pose = [0,0,0]
        self.origin_x = -width/2.0
        self.origin_y = -height/2.0
        # observations
        self.observations = list()       
        self.odom_frame = rospy.get_param('odom_frame', 'odom')
        self.base_frame = rospy.get_param('base_frame', 'base_link')
        rospy.Subscriber('base_scan',LaserScan,self.scanCb)
        self.pub = rospy.Publisher('obstacles',GridCells)

    def scanCb(self, scan):
        """ Callback that converts a laser scan into an update. """
        try:
            ((x,y,z), rot) = self.listener.lookupTransform(self.odom_frame, scan.header.frame_id, rospy.Time())
            (phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)
            self.observations.append( [scan, x, y, theta] )
            self.pose = [x,y,theta]
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Unable to transform scan from " + scan.header.frame_id + " to " + self.odom_frame)

    def amoritize(self, theta=0.75):
        """ Move all cells closer to median value of 50. """
        pix = self.grid.load()
        for i in range(self.width/self.resolution):
            for j in range(self.height/self.resolution):
                pix[i,j] = int((pix[i,j]-50)*theta) + 50

    def cost(self, trajectory, time=5.0, steps=10.0):
        """ Find the cost of cells covered by a trajectory. 
            trajectory - (dx, dth).
            time - time to forward simulate (seconds).
            steps - steps per second for forward simulation. """
        (dx, dth) = trajectory
        cost = 0
        pix = self.grid.load()
        # forward simulate
        x = self.pose[0]
        y = self.pose[1]
        th = self.pose[2]
        dx = dx/steps
        dth = dth/steps
        for i in range(int(steps*time)):
            # simulate in 0.1s steps
            x += dx*cos(th)
            y += dx*sin(th)
            th += dth
            # evaluate cost of cell[x][y]
            try:
                i = (x - self.origin_x)/self.resolution
                j = (y - self.origin_y)/self.resolution
                cost += pix[i,j] #self.grid[i][j]
                print pix[i,j],
            except IndexError:
                # out of range, assume obstacle
                cost += 100
        print cost
        return cost

    def world_to_map(self, x1, y1, x2, y2):
        """ Convert a ray in the world into map coordinates. """
        x1 = int((x1-self.origin_x)/self.resolution)
        y1 = int((y1-self.origin_y)/self.resolution)
        x2 = int((x2-self.origin_x)/self.resolution)
        y2 = int((y2-self.origin_y)/self.resolution)
        return [x1, y1, x2, y2]

    def map_to_world(self, x, y):
        """ Convert a point in the map into world coordinates. """
        #x1 = (x*self.resolution+self.origin_x)
        #y1 = (y*self.resolution+self.origin_y) 
        x1 = (x*self.resolution)+self.origin_x
        y1 = (y*self.resolution)+self.origin_y        
        th = self.pose[2]
        x = x1*cos(-th)+y1*sin(-th)
        y = x1*sin(-th)+y1*cos(-th)
        return [x,y]

    def update(self, max_scans=5):
        """ Update the map with the latest scan_observations. """
        self.grid = Image.new("1",((self.width/self.resolution),(self.height/self.resolution)), 50)
        draw = ImageDraw.Draw(self.grid)
        # update position of base
        try:
            ((x,y,z), rot) = self.listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time())
            (phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)
            self.pose = [x,y,theta]
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Unable to transform scan from " + self.base_frame + " to " + self.odom_frame)
        self.origin_x = self.pose[0] - self.width/2.0
        self.origin_y = self.pose[1] - self.height/2.0
        # current pose to relate scans to
        x = self.pose[0]
        y = self.pose[1]        
        th = self.pose[2]
        # raycast scans
        for scan in self.observations:
            (s,x1,y1,th1) = scan
            angle = s.angle_min+th1
            for r in s.ranges:
                if r > s.range_min:
                    x2 = x1 + cos(angle)*r
                    y2 = y1 + sin(angle)*r
                    l = self.world_to_map(x1,y1,x2,y2)
                    # clear free space
                    draw.line(l, fill=0)
                if r == 0.0: # max range
                    x2 = x1 + cos(angle)*3.0
                    y2 = y1 + sin(angle)*3.0
                    l = self.world_to_map(x1,y1,x2,y2)
                    # clear free space
                    draw.line(l, fill=0)
                angle += s.angle_increment
            angle = s.angle_min+th1
            for r in s.ranges:
                if r > s.range_min:
                    x2 = x1 + cos(angle)*r
                    y2 = y1 + sin(angle)*r
                    # draw enlarged obstacles
                    l = self.world_to_map(x2-self.rad,y2-self.rad,x2+self.rad,y2+self.rad)
                    draw.ellipse(l, outline=100, fill=100)
                angle += s.angle_increment
        rospy.loginfo("Updated map with " + str(len(self.observations)) + " scans")
        # drop older scans
        if len(self.observations) > max_scans:
            self.observations = self.observations[-max_scans:]  
        if rospy.Time.now() > self.last_update + rospy.Duration.from_sec(1.0):
            obstacles = GridCells()
            obstacles.header.frame_id = self.base_frame
            obstacles.header.stamp = rospy.Time.now()
            obstacles.cell_width = self.resolution
            obstacles.cell_height = self.resolution
            pix = self.grid.load()
            for i in range(self.width/self.resolution):
                for j in range(self.height/self.resolution):
                    if pix[i,j] > 50:
                        # obstacle
                        (x,y) = self.map_to_world(i,j)
                        obstacles.cells.append(Point(x,y,0))
            self.pub.publish(obstacles)     
            self.last_update = obstacles.header.stamp 

    def show(self):
        """ Show the map. """
        self.grid.show()

class TelepresenceController:
    """ An implementation of Trajectory Rollout, modified as per PML paper. """

    def __init__(self):
        self.map = CostMap()
        self.pub = rospy.Publisher('cmd_vel', Twist)
        rospy.Subscriber('desired_vel', Twist, self.twistCb)
        rospy.Subscriber('base_scan', LaserScan, self.map.scanCb)

        self.desired_x = 0
        self.desired_theta = 0

    def cost(self, t, td, sigma=2.0):
        """ Find cost of divergence between t and td. """
        (x1, th1) = t
        (x2, th2) = td
        return (x1-x2)*(x1-x2) + sigma*(th1-th2)*(th1-th2)

    def findBestTrajectory(self, x, theta, alpha=1.0, beta=10.0, sigma=0.0):
        """ Find the best trajectory, given a desired movement of (x,theta).
            x - desired forward trajectory
            th - desired angular trajectory
            alpha - scaling parameter for obstacle cost component
            beta - scaling parameter for cost of divergence component
            sigma - scaling parameter for overall speed """
        traj_list = list()
        # sample around x, theta
        for i in range(5):
            for j in range(10):
                traj_list.append( (x + (i-1)*0.025, theta + (j-2)*0.1) )                
        # evaluate trajectories
        best_cost = alpha*self.map.cost(traj_list[0]) + beta*self.cost(traj_list[0],(x,theta))
        best_traj = traj_list[0]
        for t in traj_list:
            c = alpha*self.map.cost(t) + beta*self.cost(t,(x,theta))
            print t,c
            if c < best_cost:
                best_cost = c
                best_traj = t
        #if best_traj != (x,theta):
        rospy.loginfo("Desired: "+str(x)+","+str(theta)+", best: "+str(best_traj))
        return best_traj

    def update(self):
        """ Update trajectory output. """
        x = self.desired_x
        th = self.desired_theta       
        # update map
        self.map.update()
        if x != 0 or th != 0:
            # evaluate trajectories
            x,th = self.findBestTrajectory(x,th)
        # send trajectory
        cmd = Twist()
        cmd.linear.x = x
        cmd.angular.z = th
        self.pub.publish(cmd)

    def twistCb(self, data):
        """ Callback to update internal velocity. """
        self.desired_x = data.linear.x
        self.desired_theta = data.angular.z

if __name__ == '__main__':
    rospy.init_node('traj_controller')
    if rospy.get_param('mode', 'telepresence') == 'telepresence':
        # do telepresence controller experiments
        controller = TelepresenceController()
        r = rospy.Rate(rospy.get_param('rate', 5.0))
        while not rospy.is_shutdown():
            controller.update()
            r.sleep()
    else:
        # autonomous controller experiments
        controller = AutonomousController()
        r = rospy.Rate(rospy.get_param('rate', 5.0))
        while not rospy.is_shutdown():
            controller.update()
            r.sleep()

