#!/bin/bash

# streams on https://www.twitch.tv/autonomousdronedevelopers
# but also records and saves to /home/dvic/

## TWITCH STREAMING
#obs --startrecording --startstreaming

## ONE CAMERA FOR CHOREOGRAPHIES
obs --startrecording --profile lab_table_camera --scene corner_table

## FRONT CAMERA FOR HAND CONTROL 
#obs --startrecording --profile lab_table_camera --scene gesture_demo

## VIRTUAL CAMERAS FOR XREALITY
#obs --startrecording --profile lab_table_camera --scene xreality_demo

#ROSBAG RECORDING FOR FLIGHT ANALYSIS
#rosbag record -a

