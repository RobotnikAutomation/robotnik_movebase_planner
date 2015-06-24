#!/bin/sh
rosrun actionlib_msgs genaction.py -o msg/ action/GoTo.action 
rosrun actionlib_msgs genaction.py -o msg/ action/Command.action
