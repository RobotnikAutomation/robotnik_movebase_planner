#!/usr/bin/env python

"""
Copyright (c) 2014, Robotnik Automation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("interactive_markers")
import rospy, rospkg
import copy
import os
import tf

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl
from interactive_markers.menu_handler import *

import actionlib
from geometry_msgs.msg import Pose2D
from robotnik_mb_msgs.msg import goal, GoToGoal, GoToAction, CommandGoal, CommandAction
from std_srvs.srv import Empty, SetBool
from math import sqrt

server = None

# Client based on ActionServer to send Path goals to the robotnik_mb_planner node 
class MbPlannerClientPath():

    def __init__(self, planner_name):
        self.planner_name = planner_name
        # Creates the SimpleActionClient, passing the type of the action
        # (GoTo) to the constructor.
        self.client = actionlib.SimpleActionClient(planner_name, GoToAction)

    ## @brief Sends the goal to 
    ## @return 0 if OK, -1 if no server, -2 if it's tracking a goal at the moment
    def goTo(self, goal_list):
        # Waits until the action server has started up and started
        # listening for goals.
        if self.client.wait_for_server(timeout = rospy.Duration(3.0) ):
            #if self.getState() != GoalStatus.LOST:
            #   rospy.loginfo('MbPlannerClientPath: planner is tracking a goal')
            #   return -2

            g = GoToGoal(target = goal_list)
            rospy.loginfo('MbPlannerClientPath: Sending %d waypoints'%(len(goal_list)))
            self.client.send_goal(g)
            return 0
        else:
            rospy.logerr('MbPlannerClientPath: Error waiting for server')
            return -1

    ## @brief cancel the current goal
    def cancel(self):       
        rospy.loginfo('MbPlannerClientPath: cancelling the goal')
        self.client.cancel_goal()

    ## @brief Get the state information for this goal
    ##
    ## Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED,
    ## PREEMPTED, ABORTED, SUCCEEDED, LOST.
    ##
    ## @return The goal's state. Returns LOST if this
    ## SimpleActionClient isn't tracking a goal.
    def getState(self):
        return self.client.get_state()

    ## @brief Returns ret if OK, otherwise -1
    def getResult(self):
        ret = self.client.get_result()
        if not ret:
            return -1

        else:
            return ret


# Client based on ActionServer to send Command goals to the robotnik_mb_planner node 
class MbPlannerClientCommand():

    def __init__(self, planner_name):
        self.planner_name = planner_name
        # Creates the SimpleActionClient, passing the type of the action
        # (Command) to the constructor.
        self.client = actionlib.SimpleActionClient(planner_name, CommandAction)

    ## @brief Sends the goal to 
    ## @return 0 if OK, -1 if no server, -2 if it's tracking a goal at the moment
    def Command(self, cmd, param):
        # Waits until the action server has started up and started
        # listening for goals.
        if self.client.wait_for_server(timeout = rospy.Duration(3.0) ):             
            client_id = rospy.get_name()
            g = CommandGoal(cmd, client_id, param)                      
            rospy.loginfo('MbPlannerClientCommand: Sendig command %s'%cmd)
            self.client.send_goal(g)
            return 0
        else:
            rospy.logerr('MbPlannerClientCommand: Error waiting for server')
            return -1

    ## @brief cancel the current goal
    def cancel(self):       
        rospy.loginfo('MbPlannerClientCommand: cancelling the goal')
        self.client.cancel_goal()

    ## @brief Get the state information for this goal
    ##
    ## Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED,
    ## PREEMPTED, ABORTED, SUCCEEDED, LOST.
    ##
    ## @return The goal's state. Returns LOST if this
    ## SimpleActionClient isn't tracking a goal.
    def getState(self):
        return self.client.get_state()

    ## @brief Returns ret if OK, otherwise -1
    def getResult(self):
        ret = self.client.get_result()
        if not ret:
            return -1
        else:
            return ret


## @brief Class to manage  the creation of a Waypoint base on InteractiveMarker
class PointPath(InteractiveMarker):

    def __init__(self, frame_id, name, description, is_manager = False, speed = 0.2):
        InteractiveMarker.__init__(self)

        self.header.frame_id = frame_id
        self.name = name
        self.description = description
        self.speed = speed
        self.marker = Marker()
        ##self.marker.type = Marker.CYLINDER
        self.marker.type = Marker.ARROW     
        self.marker.scale.x = 0.75
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.pose.position.z = 0.0
        if is_manager:
            self.marker.color.r = 0.8
            self.marker.color.g = 0.0
            self.marker.color.b = 0.0
            self.marker.color.a = 0.5
        else:
            self.marker.color.r = 0.0
            self.marker.color.g = 0.8
            self.marker.color.b = 0.0
            self.marker.color.a = 0.5

        self.marker_control = InteractiveMarkerControl()
        self.marker_control.always_visible = True
        self.marker_control.orientation.w = 1
        self.marker_control.orientation.x = 0
        self.marker_control.orientation.y = 1
        self.marker_control.orientation.z = 0
        self.marker_control.markers.append( self.marker )       
        self.marker_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        self.controls.append( self.marker_control )
                    
    ## @brief method called every time that an interaction is received  
    def processFeedback(self, feedback):
                
        # Position Grid         
        if (gridsize_xy_ != 0.0):           
            self.pose.position.x = round(feedback.pose.position.x / gridsize_xy_, 0) * gridsize_xy_     
            self.pose.position.y = round(feedback.pose.position.y / gridsize_xy_, 0) * gridsize_xy_
        else: 
            self.pose.position = feedback.pose.position
        
        # Orientation Grid      
        if (gridsize_th_ != 0.0):
            quaternion = (
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z,
                self.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            th = euler[2]
            thg = round( th / gridsize_th_, 0) * gridsize_th_   
            q = tf.transformations.quaternion_from_euler(0, 0, thg)             
            self.pose.orientation.x = q[0]              
            self.pose.orientation.y = q[1]
            self.pose.orientation.z = q[2]
            self.pose.orientation.w = q[3]
        else:           
            self.pose.orientation = feedback.pose.orientation

        #print feedback.marker_name + " is now at " + str(self.pose.position.x) + ", " + str(self.pose.position.y) 
                        
## @brief Manages the creation of waypoints and how to send them to the planner
class PointPathManager(InteractiveMarkerServer):
    
    def __init__(self, name, frame_id, planner_path, planner_command, planner_toggle_continous_mode):
        InteractiveMarkerServer.__init__(self, name)
        self.list_of_points = []
        self.frame_id = frame_id
        self.counter_points = 0
        
        # Initialize global grid variables
        global gridsize_xy_ 
        global gridsize_th_
        gridsize_xy_ = 0.25
        gridsize_th_ = 0.31415926535
        
        # Menu handler to create a menu
        self.menu_handler = MenuHandler()
        h_first_entry = self.menu_handler.insert( "Waypoints" )     
        entry_new = self.menu_handler.insert( "Create New", parent=h_first_entry)
        entry = self.menu_handler.insert( "Delete last", parent=h_first_entry, callback=self.deletePointCB );
        entry = self.menu_handler.insert( "Delete all", parent=h_first_entry, callback=self.deleteAllPointsCB );
        entry = self.menu_handler.insert( "Save", parent=h_first_entry, callback=self.savePointsCB );
        entry = self.menu_handler.insert( "Load", parent=h_first_entry, callback=self.loadPointsCB );
        entry = self.menu_handler.insert( "0.1 m/s", parent=entry_new, callback=self.newPointCB_01)
        entry = self.menu_handler.insert( "0.2 m/s", parent=entry_new, callback=self.newPointCB_02)
        entry = self.menu_handler.insert( "0.3 m/s", parent=entry_new, callback=self.newPointCB_03)
        entry = self.menu_handler.insert( "0.6 m/s", parent=entry_new, callback=self.newPointCB_06)
        entry = self.menu_handler.insert( "1.0 m/s", parent=entry_new, callback=self.newPointCB_10)     
        h_second_entry = self.menu_handler.insert( "Actions" )
        entry = self.menu_handler.insert( "Go", parent=h_second_entry, callback=self.startRouteCB)  # Send the path from the first point to the last one
        entry = self.menu_handler.insert( "Cancel", parent=h_second_entry, callback=self.cancelRouteCB) # Stops the current path 
        #entry = self.menu_handler.insert( "Go back", parent=h_second_entry, callback=self.reverseRouteCB) # Sends the path from the last point to the first one
        entry = self.menu_handler.insert( "Start from this wp", parent=h_second_entry, callback=self.startFromThisWPCB) # Start from the closest WP
        entry = self.menu_handler.insert( "Pause", parent=h_second_entry, callback=self.sendCommandPauseCB)
        entry = self.menu_handler.insert( "Restart", parent=h_second_entry, callback=self.sendCommandRestartCB)
        entry = self.menu_handler.insert( "Twist", parent=h_second_entry, callback=self.sendCommandTwistCB)
        self.continuous_mode_entry = self.menu_handler.insert( "ContinousMode", parent=h_second_entry, callback=self.toggleContinuousModeCB)
        h_third_entry = self.menu_handler.insert( "Interface" )
        entry_pgrid = self.menu_handler.insert( "Position Grid", parent=h_third_entry)
        entry_agrid = self.menu_handler.insert( "Orientation Grid", parent=h_third_entry)
        entry = self.menu_handler.insert( "off", parent=entry_pgrid, callback=self.gridXY_0)
        entry = self.menu_handler.insert( "0.1", parent=entry_pgrid, callback=self.gridXY_1)
        entry = self.menu_handler.insert( "0.25", parent=entry_pgrid, callback=self.gridXY_2)
        entry = self.menu_handler.insert( "off", parent=entry_agrid, callback=self.gridTH_0)
        entry = self.menu_handler.insert( "pi/4", parent=entry_agrid, callback=self.gridTH_1)
        entry = self.menu_handler.insert( "pi/10", parent=entry_agrid, callback=self.gridTH_2)
            
        # Creates the first point
        #self.list_of_points.append(PointPath(frame_id, 'p1', 'p1'))
        self.initial_point = PointPath(frame_id, 'PointManager', 'PointManager', True)
        self.insert(self.initial_point, self.initial_point.processFeedback)

        # set different callback for POSE_UPDATE feedback
        self.setCallback(self.initial_point.name, self.alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )
        
        self.menu_handler.apply( self, self.initial_point.name )
        self.applyChanges()
        
        
        rospy.loginfo('Planner_path=%s  Planner_command=%s'%(planner_path, planner_command))
        self.planner_client = MbPlannerClientPath(planner_path)
        self.command_client = MbPlannerClientCommand(planner_command)

        self.current_continous_mode = False
        rospy.loginfo('Waiting for %s service' % planner_toggle_continous_mode)
        rospy.wait_for_service(planner_toggle_continous_mode)
        self.continuous_mode_service = rospy.ServiceProxy(planner_toggle_continous_mode, SetBool)
        self.continuous_mode_service(self.current_continous_mode)
        self.menu_handler.setCheckState(self.continuous_mode_entry, MenuHandler.UNCHECKED)

        # Locates and loads the UI file into the widget
        rp = rospkg.RosPack()       
        # loads a ui file for the dialog
        self.points_file_path = os.path.join(rp.get_path('robotnik_mb_planner'), 'config', 'waypoints.txt')
        
        #rospy.Timer(rospy.Duration(5), self.createNewPoint)
        self._go_service = rospy.Service('%s/go'%rospy.get_name(), Empty, self.goService)
        self._go_back_service = rospy.Service('%s/go_back'%rospy.get_name(), Empty, self.goBackService)
        self._cancel_service = rospy.Service('%s/cancel'%rospy.get_name(), Empty, self.cancelService)

        

    def alignMarker( self, feedback ):
        pose = feedback.pose
        
        # Position Grid
        if (gridsize_xy_ != 0.0):
            pose.position.x = round(pose.position.x / gridsize_xy_, 0) * gridsize_xy_
            pose.position.y = round(pose.position.y / gridsize_xy_, 0) * gridsize_xy_
        
        # Orientation Grid 
        if (gridsize_th_ != 0.0):
            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            th = euler[2]
            thg = round( th / gridsize_th_, 0) * gridsize_th_   
            q = tf.transformations.quaternion_from_euler(0, 0, thg)             
            pose.orientation.x = q[0]               
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

        self.setPose( feedback.marker_name, pose )
        self.applyChanges()

    ## @brief Creates a new PointPath and save it a list
    def createNewPoint(self, speed = 0.2):
                    
        ##print 'Creating new point %d'%(self.counter_points)
        new_point = PointPath(self.frame_id, 'p%d'%(self.counter_points), 'p%d->%.1f'%(self.counter_points, speed), speed = speed)
        #new_point = PointPath(self.frame_id, '1', '1')
        
        if len(self.list_of_points) > 1:
            new_point.pose.position.x = self.list_of_points[self.counter_points-1].pose.position.x
            new_point.pose.position.y = self.list_of_points[self.counter_points-1].pose.position.y
                        
        elif len(self.list_of_points) == 1:
            new_point.pose.position.x = self.list_of_points[0].pose.position.x
            new_point.pose.position.y = self.list_of_points[0].pose.position.y
    
        new_point.pose.position.x = new_point.pose.position.x  + 1.0
                    
        #print 'Creating new point at position %.2lf, %.2lf, %.2lf'%(new_point.pose.position.x, new_point.pose.position.y, new_point.pose.position.z)
        
        self.list_of_points.append(new_point)
        self.insert(new_point, new_point.processFeedback)       
        # set different callback for POSE_UPDATE feedback
        # self.setCallback(new_point.name, new_point.alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )       
        self.setCallback(new_point.name, self.alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )      
        self.menu_handler.apply( self, 'p%d'%(self.counter_points) )
        self.applyChanges()
        self.counter_points = self.counter_points + 1
        
        #for i in self.list_of_points:
        #   print 'Point %s: %.2lf, %.2lf, %.2lf'%(i.name, i.pose.position.x, i.pose.position.y, i.pose.position.z)
            
        return
    
    ## @brief Callback called to turn position grid off
    def gridXY_0(self, feedback ):
        global gridsize_xy_
        gridsize_xy_ = 0.0
        print 'Position Grid Off'
        
    ## @brief Callback called to define position grid
    def gridXY_1(self, feedback ):
        global gridsize_xy_
        gridsize_xy_ = 0.1
        print 'Position Grid set to ' + str(gridsize_xy_)
    
    ## @brief Callback called to define position grid   
    def gridXY_2(self, feedback ):
        global gridsize_xy_
        gridsize_xy_ = 0.25
        print 'Position Grid set to ' + str(gridsize_xy_)
        
    ## @brief Callback called to turn orientation grid off
    def gridTH_0(self, feedback ):
        global gridsize_th_
        gridsize_th_ = 0.0
        print 'Orientation Grid Off'
        
    ## @brief Callback called to define orientation grid
    def gridTH_1(self, feedback ):
        global gridsize_th_
        gridsize_th_ = 3.1415926535 / 4.0
        print 'Orientation Grid set to ' + str(gridsize_th_)

    ## @brief Callback called to define orientation grid
    def gridTH_2(self, feedback ):
        global gridsize_th_
        gridsize_th_ = 3.1415926535 / 10.0
        print 'Orientation Grid set to ' + str(gridsize_th_)
    
    ## @brief Callback called to create a new point 
    def newPointCB(self, feedback):
        #print 'newPointCB'
        self.createNewPoint()
    ## @brief Callback called to create a new point 
    def newPointCB_01(self, feedback):
        #print 'newPointCB'
        self.createNewPoint(0.1)
    ## @brief Callback called to create a new point 
    def newPointCB_02(self, feedback):
        #print 'newPointCB'
        self.createNewPoint(0.2)
    ## @brief Callback called to create a new point 
    def newPointCB_03(self, feedback):
        self.createNewPoint(0.3)
    def newPointCB_06(self, feedback):      
        self.createNewPoint(0.6)
    def newPointCB_10(self, feedback):      
        self.createNewPoint(1.0)

    ## @brief Callback called to create a new point 
    def deletePointCB(self, feedback):
        if self.counter_points > 0:
             p = self.list_of_points.pop()
             self.counter_points = self.counter_points - 1
             self.erase(p.name)
             self.applyChanges()
             
         #print 'deletePointCB' 
    
    ## @brief Function called to delete all the waypoints
    def deleteAllPoints(self):
        for i in range(len(self.list_of_points)):
            p = self.list_of_points.pop()
            self.counter_points = self.counter_points - 1
            self.erase(p.name)
             
        self.applyChanges()
        
    ## @brief Callback called to delete all the waypoints
    def deleteAllPointsCB(self, feedback):
        self.deleteAllPoints()
    
    ## @brief Callback called to save the current points
    def savePointsCB(self, feedback):
        
        try:
            file_points = open(self.points_file_path, 'w')
        except IOError, e:
            rospy.logerr( 'path_marker::savePointsCB: File %s not found: %s'%(self.points_file_path, e))
            return
        
        for i in self.list_of_points:
            # line format = 'p1;p1->0.4;0.5;5.2;0.0;0.4' (ID, description, x, y, th, speed)
            quaternion = (
                i.pose.orientation.x,
                i.pose.orientation.y,
                i.pose.orientation.z,
                i.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]                                                                  
            line = '%s;%s;%.3f;%.3f;%.5f;%.3f\n'%(i.name, i.description, i.pose.position.x, i.pose.position.y, yaw, i.speed)
            file_points.write(line) 
        
        rospy.loginfo('path_marker:savePointsCB: saved %d points'%(len(self.list_of_points)))
        file_points.close()
        
        
    ## @brief Callback called to load previous saved points
    def loadPointsCB(self, feedback):
        # Deletes all the loaded points
        if len(self.list_of_points) > 0:
            self.deleteAllPoints()
        
        try:
            file_points = open(self.points_file_path, 'r')
        except IOError, e:
            rospy.logerr( 'path_marker::loadPointsCB: File %s not found: %s'%(self.points_file_path, e))
            return
        
        num_of_loaded_points = 0
        # Reads and extracts the information of every line
        line = file_points.readline().replace('\n', '')
        while line != '':
            # line format = 'p1;p1->0.4;0.5;5.2;0.0;0.4'
            a = line.split(';')
            # a = ['p1', 'p1->0.4', '0.5', '0.4', '3.1416, '0.4'] // [ ID, DESCRIPTION, X, Y, TH, SPEED]
            if len(a) == 6:
                new_point = PointPath(self.frame_id, a[0], a[1], speed = float(a[5]))
                new_point.pose.position.x = float(a[2])
                new_point.pose.position.y = float(a[3])             
                th = float(a[4])
                
                q = tf.transformations.quaternion_from_euler(0, 0, th)              
                new_point.pose.orientation.x = q[0]             
                new_point.pose.orientation.y = q[1]
                new_point.pose.orientation.z = q[2]
                new_point.pose.orientation.w = q[3]
                                            
                rospy.loginfo('path_marker::loadPointsCB: Loading point %s at location %.2lf, %.2lf, %.2lf, speed =  %.2lf'%(a[0], new_point.pose.position.x, new_point.pose.position.y, th, new_point.speed))
                
                self.list_of_points.append(new_point)
                self.insert(new_point, new_point.processFeedback)
                self.setCallback(new_point.name, self.alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )      
                self.menu_handler.apply( self, a[0])
                self.applyChanges()
                self.counter_points = self.counter_points + 1
                num_of_loaded_points = num_of_loaded_points + 1             
            else:
                rospy.logerr('path_marker::loadPointsCB: Error processing line %s'%(line))      
                
            
            line = file_points.readline().replace('\n', '')
        
        file_points.close()
        
        rospy.loginfo('path_marker::loadPointsCB: Loaded %d points'%(num_of_loaded_points)) 
        
        
    ## @brief Starts the route
    def startRouteCB(self, feedback):
        goals = self.convertListOfPointPathIntoGoal()
        print 'goals: %s'%(goals)
        self.planner_client.goTo(goals)
        return
    
    ## @brief Starts the route on the inverse direction
    def reverseRouteCB(self, feedback):
        goals = self.convertListOfPointPathIntoGoal(inverse = True)
        #print 'goals: %s'%(goals)
        self.planner_client.goTo(goals)
        return
        
    ## @brief Stops the current route if it's started
    def cancelRouteCB(self, feedback):
        self.planner_client.cancel()
        return
    
    ## @brief Starts the route (inverse order of waypoints)
    def convertListOfPointPathIntoGoal(self, inverse = False):
        converted_list = []
        if inverse:
            for i in reversed(self.list_of_points):     
                #type(pose) = geometry_msgs.msg.Pose
                quaternion = (
                    i.pose.orientation.x,
                    i.pose.orientation.y,
                    i.pose.orientation.z,
                    i.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                ##roll = euler[0]
                ##pitch = euler[1]
                yaw = euler[2]                                                      
                converted_list.append(goal(pose = Pose2D(i.pose.position.x, i.pose.position.y, yaw), speed = i.speed)) # For now speed constant
        else:
            for i in self.list_of_points:
                quaternion = (
                    i.pose.orientation.x,
                    i.pose.orientation.y,
                    i.pose.orientation.z,
                    i.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                ##roll = euler[0]
                ##pitch = euler[1]
                yaw = euler[2]                                                                      
                #print 'convertListOfPointPathIntoGoal: %.2lf, %.2lf'%(i.pose.position.x, i.pose.position.y)
                converted_list.append(goal(pose = Pose2D(i.pose.position.x, i.pose.position.y, yaw), speed = i.speed)) # For now speed constant
        
        return converted_list
    
    ## @brief Fake service to emulate the event start route
    def goService(self, param):
        rospy.loginfo('%s::goService'%(rospy.get_name()))
        
        self.startRouteCB(None)
        
        return []
    
    ## @brief Fake service to emulate the event reverse route
    def goBackService(self, param):
        rospy.loginfo('%s::goBackService'%(rospy.get_name()))
        
        self.reverseRouteCB(None)
        
        return []
        
    ## @brief Fake service to emulate the event cancel route
    def cancelService(self, param):
        rospy.loginfo('%s::cancelService'%(rospy.get_name()))
        self.cancelRouteCB(None)
        
        return []

    ## @brief Send command PAUSE
    def sendCommandPauseCB(self, feedback):
        self.command_client.Command(cmd="PAUSE", param=[0,0])
        return

    ## @brief Send command RESTART
    def sendCommandRestartCB(self, feedback):
        self.command_client.Command(cmd="RESTART", param=[0,0])
        return

    ## @brief Send command TWIST
    def sendCommandTwistCB(self, feedback):
        self.command_client.Command(cmd="TWIST", param=[0.3,0.15])
        return
    ## @brief Toogle ContinuousMode
    def toggleContinuousModeCB(self, feedback):
        self.current_continous_mode = not(self.current_continous_mode)
        check_state = MenuHandler.NO_CHECKBOX
        check_state = MenuHandler.UNCHECKED
        if self.current_continous_mode == True:
            check_state = MenuHandler.CHECKED
        else:
            check_state = MenuHandler.UNCHECKED
        self.menu_handler.setCheckState(self.continuous_mode_entry, check_state)
        self.continuous_mode_service(self.current_continous_mode)
        self.menu_handler.reApply(server)
        self.applyChanges()
        return

    def startFromThisWPCB(self, feedback):
        pose = feedback.pose
        min_dist = 100.0;
        index = -1;     
        for i in self.list_of_points:
            distance = self.Dist( x1=pose.position.x, y1=pose.position.y, x2=i.pose.position.x, y2=i.pose.position.y )
            if distance < min_dist:
                min_dist = distance;            
                index = self.list_of_points.index(i)
        print 'Closest point is : %d'%(index)
        if index != -1:
            self.command_client.Command(cmd="STARTFROM", param=[index, 0.0])
        return 

    def Dist( self, x1,  y1,  x2,  y2):
        diff_x = (x2 - x1)
        diff_y = (y2 - y1)
        return sqrt( diff_x*diff_x + diff_y*diff_y )
            
# To insert intermediate waypoints
#list.insert(i, x)
#Insert an item at a given position

        
if __name__=="__main__":
    rospy.init_node("movebase_path_marker")
    
    _name = rospy.get_name().replace('/','')
    
    arg_defaults = {
      'frame_id': '/map',
      'planner_path': 'robotnik_mb_planner/path',
      'planner_command': 'robotnik_mb_planner/command',
      'planner_toggle_continous_mode': 'robotnik_mb_planner/continuous_mode'
    }
    
    args = {}
    
    for name in arg_defaults:
        try:
            if rospy.search_param(name): 
                args[name] = rospy.get_param('%s/%s'%(_name, name)) # Adding the name of the node, because the para has the namespace of the node
            else:
                args[name] = arg_defaults[name]
            #print name
        except rospy.ROSException, e:
            rospy.logerror('%s: %s'%(e, _name))
    
    server = PointPathManager(_name, frame_id = args['frame_id'], planner_path = args['planner_path'], planner_command = args['planner_command'], planner_toggle_continous_mode= args['planner_toggle_continous_mode'])
    
    rospy.spin()

