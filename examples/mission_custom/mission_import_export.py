"""
mission_import_export.py: 

This example demonstrates how to import and export files in the Waypoint file format 
(http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format). The commands are imported
into a list, and can be modified before saving and/or uploading.

Documentation is provided at http://python.dronekit.io/examples/mission_import_export.html
"""


from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import datetime
import math

import socket
import sys
from thread import *

from pymavlink import mavutil


#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Example that demonstrates mission import/export from a file. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                   help="vehicle connection target. Default '127.0.0.1:14550'")
parser.add_argument('--mission', default='mpmission.txt',
                    help="choose a mission")
args = parser.parse_args()


#Connect to the Vehicle
print 'Connecting to vehicle on: %s. Loading mission: %s.' % (args.connect, args.mission)
vehicle = connect(args.connect, wait_ready=True)    
vehicle.heading_at_connect = vehicle.heading




# HOST = ''   # Symbolic name, meaning all available interfaces
# PORT = 8888 # Arbitrary non-privileged port
 
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# print 'Socket created'
 
# #Bind socket to local host and port
# try:
#     s.bind((HOST, PORT))
# except socket.error as msg:
#     print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
#     sys.exit()
     
# print 'Socket bind complete'
 
# #Start listening on socket
# s.listen(10)
# print 'Socket now listening'
 
# #now keep talking with the client
# while 1:
#     #wait to accept a connection - blocking call
#     conn, addr = s.accept()
#     print 'Connected with ' + addr[0] + ':' + str(addr[1])
     
# s.close()

















def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint=vehicle.commands.next
    if nextwaypoint ==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat=missionitem.x
    lon=missionitem.y
    alt=missionitem.z
    targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint



def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print "\nReading mission from file: %s" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                print 'command: %s' % ln_command
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                if ln_command==21:
                    cmd = Command( 0, 0, 0, ln_frame, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 1, vehicle.heading_at_connect, 0, 0, 0, 0, 0, 0)
                    print 'Created WP to reset heading to same as time of connection.'
                    missionlist.append(cmd)
                    print 'Loading Land WP.'
                    cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
                    print 'Loading disarm WP'
                    cmd = Command( 0, 0, 0, ln_frame, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                    missionlist.append(cmd)
                else:
                    cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
    return missionlist


def upload_mission(aFileName):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(aFileName)
    
    print "\nUpload mission from a file: %s" % import_mission_filename
    #Clear existing mission from vehicle
    print ' Clear mission'
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print ' Upload mission'
    vehicle.commands.upload()


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print " Download mission from vehicle"
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)


def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print "\nSave mission from Vehicle to file: %s" % export_mission_filename    
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print " Write mission to file"
        file_.write(output)
        
        
def printfile(aFileName):
    """
    Print a mission file to demonstrate "round trip"
    """
    print "\nMission file: %s" % aFileName
    with open(aFileName) as f:
        for line in f:
            print ' %s' % line.strip()        


#import_mission_filename = 'mpmission.txt'
# import_mission_filename = 'Hope_all_60m.txt'
import_mission_filename = 'missions/%s' % args.mission

datetime = '{:%Y-%m-%d_%H-%M-%S}'.format(datetime.datetime.now())
filename = args.mission.split('.',1)

export_mission_filename = 'missions_completed/%s-%s.%s' % (filename[0],datetime,filename[1])

#Upload mission from file
upload_mission(import_mission_filename)

# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)

print "Starting mission"
# Reset mission set to first (0) waypoint
vehicle.commands.next=1

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")





HOST = ''   # Symbolic name meaning all available interfaces
PORT = 8888 # Arbitrary non-privileged port
 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'Socket created'
 
#Bind socket to local host and port
try:
    s.bind((HOST, PORT))
except socket.error as msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
     
print 'Socket bind complete'
 
#Start listening on socket
s.listen(10)
print 'Socket now listening'
 
#Function for handling connections. This will be used to create threads
def clientthread(conn):
    #Sending message to connected client
    conn.send('Welcome to the server. Type something and hit enter\n') #send only takes string
     
    #infinite loop so that function do not terminate and thread do not end.
    while True:
         
        #Receiving from client
        data = conn.recv(1024)

        reply = 'OK...' + data

        # *** New
        # data will be the new land location and heading
        # Should validate data here - should be (lat,lng,heading)
        # Pass data to a helper function that will update the mission
        # newLandingLocation(data)
            # **** Ideally if we can create multiple sockets...
            # **** then we could be "holding" the new location data
            # **** and just before heading to the land wp check to see 
            # **** if there is a new location in the holding socket

        if not data: 
            break
     
        conn.sendall(reply)
     
    #came out of loop
    conn.close()
 
#now keep talking with the client
while 1:
    #wait to accept a connection - blocking call
    conn, addr = s.accept()
    print 'Connected with ' + addr[0] + ':' + str(addr[1])
     
    #start new thread takes 1st argument as a function name to be run, second is the tuple of arguments to the function.
    start_new_thread(clientthread ,(conn,))
 
s.close()

"""
def newLandingLocation(location):
    location = location.split(',', 3)
    nextwaypoint = vehicle.commands.next
"""











# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

while True:
    nextwaypoint=vehicle.commands.next
    print 'Command %s of %s.' % (nextwaypoint, vehicle.commands.count)
    print 'Vehicle armed: %s' % vehicle.armed
    if nextwaypoint == vehicle.commands.count & vehicle.armed == False:
       print 'Vehicle has landed and disarmed.'
       break
    print 'Battery voltage: %s' % vehicle.battery.voltage
    print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
    print 'Vehicle mode: %s' % vehicle.mode
    print 'System status: %s' % vehicle.system_status.state
    time.sleep(1)

#Download mission we just uploaded and save to a file
save_mission(export_mission_filename)

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

print "\nShow original and uploaded/downloaded files:"
#Print original file (for demo purposes only)
printfile(import_mission_filename)
#Print exported file (for demo purposes only)
printfile(export_mission_filename)
