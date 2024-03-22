#!/usr/bin/python3
# ROS python API
import json
import rospy
import numpy as np
#import time
import threading
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from detection_msgs.msg import *
from coordinate_msgs.msg import *
from std_msgs.msg import *

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self, arg_alt):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = arg_alt, latitude = float('nan'), longitude = float('nan'), min_pitch=0.0, yaw=0.0)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" %e)

    def setLand(self, arg_alt):
        rospy.wait_for_service('mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landService(altitude = arg_alt, latitude = float('nan'), longitude = float('nan'), min_pitch=0.0, yaw=0.0)
            print('Land Succeeded')
        except rospy.ServiceException as e:
            print("Service land call failed: %s" %e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
            print('Arming Succeeded')
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" %e)
        rospy.sleep(5)
        return

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
            print('Disarming Succeeded')
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" %e)

    def autoSetMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="AUTO.MISSION")
            print('Changed to Auto Mission Mode')
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" %e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
            print('Changed to Stabilized Mode')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Stabilized Mode could not be set." %e)

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
            print('Changed to Offboard Mode')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." %e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
            print('Changed to ALTCTL Mode')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Altitude Mode could not be set." %e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
            print('Changed to POSCTL Mode')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Position Mode could not be set." %e)

    def setAutoTakeoffMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.TAKEOFF')
            print('Changed to Auto Takeoff Mode')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autotakeoff Mode could not be set." %e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
            print('Changed to Auto Land Mode')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Autoland Mode could not be set." %e)

    def setGuidedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='GUIDED')
            print('Changed to Guided Mode')
        except rospy.ServiceException as e:
               print("service set_mode call failed: %s. Guided Mode could not be set." %e)

    def change_mav_frame(self):
        rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
        try:
            frame = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', mavros_msgs.srv.SetMavFrame)
            resp = frame(8)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed")

    def clear_pull():
        print("\n----------clear_pull----------")
        # Clearing waypoints
        rospy.wait_for_service("/mavros/mission/clear")
        waypoint_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
        resp = waypoint_clear()
        rospy.sleep(5)
        # Call waypoints_pull
        rospy.wait_for_service("/mavros/mission/pull")
        waypoint_pull = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
        resp = waypoint_pull()
        rospy.sleep(5)
        return
    
    def switch_modes(current_mode, next_mode, delay):
        print("\n----------switch_modes----------")
        rospy.wait_for_service("/mavros/set_mode")
        modes = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        resp = modes(current_mode, next_mode)
        rospy.sleep(delay)
        return

    def wpPush(self,index,wps):
        print("\n----------pushingWaypoints----------")
        rospy.wait_for_service('mavros/mission/push')
        try:
            wpPushService = rospy.ServiceProxy('mavros/mission/push', WaypointPush,persistent=True)
            wpPushService(start_index=0,waypoints=wps)#start_index = the index at which we want the mission to start
            print("Waypoint Pushed")
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" %e)
        rospy.sleep(5)
        return

    def wpPull(self,wps):
        rospy.wait_for_service('mavros/mission/pull')
        try:
            wpPullService = rospy.ServiceProxy('mavros/mission/pull', WaypointPull,persistent=True)
            print(wpPullService().wp_received)
            print("Waypoint Pulled")
        except rospy.ServiceException as e:
            print("Service Puling call failed: %s" %e)

class Generator:
    def __init__(self):
        pass

    def coordinates_generator(self, arg_lat, arg_lon, arg_dist, arg_coors):
        lat = arg_lat
        lon = arg_lon 
        dist = arg_dist 
        coors = arg_coors #num coordinates in each direction

        #Creating the offset grid
        mini, maxi = -dist*coors, dist*coors
        n_coord = coors*2+1
        axis = np.linspace(mini, maxi, n_coord)
        X, Y = np.meshgrid(axis, axis)

        #avation formulate for offsetting the latlong by offset matrices
        R = 6378137 #earth's radius
        dLat = X/R
        dLon = Y/(R*np.cos(np.pi*lat/180))
        latO = lat + dLat * 180/np.pi
        lonO = lon + dLon * 180/np.pi

        #stack x and y latlongs and get (lat,long) format
        output = np.stack([latO, lonO]).transpose(1,2,0)
        output.shape
        points = output.reshape(-1,2)

        np.savetxt('/home/gerardomartino/catkin_ws/src/avoidance/local_planner/scripts/coordinate.txt', points, delimiter=',', fmt='%.7f')

        return points
    def circle_path(self, arg_lat, arg_lon, arg_dist, arg_coors): #To be Tested

        # Convert latitude and longitude to radians
        lat_rad = np.math.radians(arg_lat)
        lon_rad = np.math.radians(arg_lon)
        # Earth's radius in meters
        R = 6378137
        circle_path = []
        for angle in range(0, 360):
            angle_rad = np.math.radians(angle)
            # Calculate new latitude and longitude for each point on the circle path
            lat_point = np.math.asin(np.math.sin(lat_rad) * np.math.cos(np.radius/R) + 
                                np.math.cos(lat_rad) * np.math.sin(np.radius/R) * np.math.cos(angle_rad))
            lon_point = lon_rad + np.math.atan2(np.math.sin(angle_rad) * np.math.sin(np.radius/R) * np.math.cos(lat_rad), 
                                            np.math.cos(np.radius/R) - np.math.sin(lat_rad) * np.math.sin(lat_point))
            # Convert back to degrees
            lat_point = np.math.degrees(lat_point)
            lon_point = np.math.degrees(lon_point)
            circle_path.append((lat_point, lon_point))

        np.savetxt('/home/gerardomartino/catkin_ws/src/avoidance/local_planner/scripts/coordinate.txt', points, delimiter=',', fmt='%.7f')
        return circle_path

    def print_coordinates(self, path_file):

        input = np.loadtxt(path_file, delimiter=",")    
        print('Shape of output grid:', input.shape)
        print('')
        print('Output coordinates')
        print(input)

    def load_coordinates(self, path_file):

        input = np.loadtxt(path_file, delimiter=",")    
        return input

class wpMissionCnt:

    def __init__(self):
        self.wp = Waypoint()
        
    def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
        self.wp.frame = frame 
        self.wp.command = command 
        self.wp.is_current= is_current
        self.wp.autocontinue = autocontinue  
        self.wp.param1=param1 
        self.wp.param2=param2
        self.wp.param3=param3
        self.wp.param4=param4
        self.wp.x_lat = x_lat 
        self.wp.y_long=y_long
        self.wp.z_alt= z_alt 

        return self.wp
    
    def loadWaypoints(self):
        path_survey = np.loadtxt('/home/gerardomartino/catkin_ws/src/avoidance/local_planner/scripts/coordinate.txt', delimiter=",") 
        wps = [] #List to story waypoints
        i = 0
        while i < len(path_survey):
            wayp = wpMissionCnt()
            w = wayp.setWaypoints(3,16,True,True,0.0,0.0,0.0,float('nan'),path_survey[i][0],path_survey[i][1],5)
            wps.append(w)
            i += 1
        return wps

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        self.vel = TwistStamped()
        #self.global_var = Global()

        self.vel.twist.linear.y = 0
        self.vel.twist.linear.z = 0
        self.vel.twist.linear.x = 0
        self.vel.twist.angular.z = 0
        self.vel.twist.angular.x = 0
        self.vel.twist.angular.y = 0
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        self.g = GlobalPositionTarget()
        self.g.altitude = 0
        self.g.latitude = 0
        self.g.longitude = 0
        self.g.type_mask=4088
        self.g.coordinate_frame=6
        # # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)
    # Callbacks
    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y

    def x_dir(self):
        self.sp.position.x = self.local_pos.x + 5
        self.sp.position.y = self.local_pos.y

    def neg_x_dir(self):
        self.sp.position.x = self.local_pos.x - 5
        self.sp.position.y = self.local_pos.y

    def y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y - 5


    def fire(self,msg):
        msg_boundingBoxes = BoundingBoxes()
        msg_boundingBox = BoundingBox()
        msg_boundingBoxes = msg
        msg_boundingBox = msg_boundingBoxes.bounding_boxes

        for val in msg_boundingBox:
            if val.Class == "fire":
                global_var.fire = True
                self.rover_latlon = rospy.Publisher('/gps_goal_fix', NavSatFix, queue_size=10)
                #self.rover_latlon = rospy.Publisher('/fire_target', NavSatFix, queue_size=10)
                self.coord_msg = NavSatFix()
                self.coord_msg.latitude = float(49.900000060515765) #float(global_var.latitude)
                self.coord_msg.longitude = float(8.900002267821108) #float(global_var.longitude)
                self.rover_latlon.publish(self.coord_msg)
                #self.rover_latlon_pub()
                self.forest_pub = rospy.Publisher('/fire/status', String, queue_size=10)
                # Convert the JSON data to a string
                self.forest_json =  json.dumps({"sensor":"FIRE_OFF", "latitude":self.coord_msg.latitude, "longitude":self.coord_msg.longitude})
                # Create a std_msgs/String message
                self.forest_msg = String()
                self.forest_msg.data = self.forest_json
                # Publish the message
                self.forest_pub.publish(self.forest_msg)
                print("Release Extinguisher Ball")

    def forest(self, msg):
        gen = Generator()
        msg_coordinates = NavSatFix()
        msg_coordinates = msg
    
        gen.coordinates_generator(float(msg_coordinates.latitude), float(msg_coordinates.longitude),5,1)
        if global_var.mission == True:
            self.fire_survey(msg_coordinates.latitude, msg_coordinates.longitude)
            global_var.mission == False
        else:
            print("Drone in Mission Mode")

    def move(self, msg):
        msg_coordinates = NavSatFix()
        msg_coordinates = msg
        global_var.latitude = float(msg_coordinates.latitude)
        global_var.longitude = float(msg_coordinates.longitude)
        print("move to latitude: ", global_var.latitude)
        print("move to longitude: ", global_var.longitude)
        return
    
    def fire_survey(self, arg_lat, arg_lon):
        path_survey = np.loadtxt('/home/gerardomartino/catkin_ws/src/avoidance/local_planner/scripts/coordinate.txt', delimiter=",") 
        
        if cnt.state.mode != "OFFBOARD" or cnt.state.mode == "POSCTL" or cnt.state.mode == "AUTO.LAND":
            while not cnt.state.armed:
                modes.setArm()
                #rospy.sleep()
            modes.setTakeoff(3)

            k=0
            while k<10:
                g_pub.publish(cnt.g)
                k = k + 1
                #rospy.sleep()

            while cnt.state.mode != "OFFBOARD":
                if cnt.local_pos.z > 2.2:
                    print("Change in OFFBOARD")
                    modes.setOffboardMode()  
                    break
            #rospy.sleep(5)
        if global_var.mission == True:
            delay = 0
            count = 0
            while count <= len(path_survey):
                if global_var.fire == False and count < len(path_survey):
                    global_var.setLatLon(path_survey[count][0], path_survey[count][1], global_var.altitude)
                    count += 1
                    print("WP:", count)
                    print("Fire Detection at coordinates:", str(global_var.latitude), "/", str(global_var.longitude))
                    #rospy.sleep(5)
                elif global_var.fire == False and count == len(path_survey):
                    print("Fire Detection Increase Range")
                    count = 0
                    global_var.altitude = 6
                    delay += 1
                    gen = Generator()
                    gen.coordinates_generator(float(arg_lat), float(arg_lon),10,1)
                    path_survey = np.loadtxt('/home/gerardomartino/catkin_ws/src/avoidance/local_planner/scripts/coordinate.txt', delimiter=",") 
                elif global_var.fire == True:
                    global_var.setLatLon(global_var.HOME_LAT, global_var.HOME_LON, 3)
                    rospy.sleep(10)
                    modes.setAutoLandMode()
                    rospy.sleep(5)
                    modes.setDisarm()
                    rospy.sleep(5)
                    modes.setPositionMode
                    print("Drone Landed in Home Position")
                    global_var.reset() #global_var.fire = False, #global_var.mission = True
                    break
                rospy.sleep((5 + delay))


    
    def drone_startup(self):
        while not cnt.state.armed:
            modes.setArm()
            rate.sleep()
            modes.setTakeoff(3)

            k=0
        while k<10:
            g_pub.publish(cnt.g)
            rate.sleep()
            k = k + 1

        while cnt.state.mode != "OFFBOARD":
            if cnt.local_pos.z > 2.2:
                print("Change in OFFBOARD")
                modes.setOffboardMode()  
                break

class Global:
    # initialization method
    def __init__(self):
        self.HOME_LAT = 40.8255861
        self.HOME_LON = 15.2745381
        self.latitude = 40.8255861
        self.longitude = 15.2745381
        self.altitude = 3
        #self.home = False
        self.fire = False
        self.mission = True
    
    def setLatLon(self, arg_lat, arg_lon, arg_alt):
        self.latitude = arg_lat
        self.longitude = arg_lon
        self.altitude = arg_alt

    def reset(self):
        self.fire = False
        self.mission = True

    

# Main function
def main():
    global cnt
    global rate
    global g_pub
    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)
    # controller object
    cnt = Controller()
    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    # Subscribe to yolov5 realtime detection
    rospy.Subscriber('yolov5/detections', BoundingBoxes, cnt.fire)
    # Subscribe to forest detector realtime detection
    rospy.Subscriber("drone/fire/gps", NavSatFix, cnt.forest)
    # Subscribe to forest detector realtime detection
    rospy.Subscriber("drone/coordinates", NavSatFix, cnt.move)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    g_pub = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)


    # ROS main loop
    while not rospy.is_shutdown():
            cnt.g.altitude = global_var.altitude
            cnt.g.latitude = global_var.latitude
            cnt.g.longitude = global_var.longitude
            g_pub.publish(cnt.g)
            #print("sono qui")
            #rate.sleep()
            

if __name__ == '__main__':
    try:
        # flight mode object
        modes = fcuModes()
        global_var = Global()
        main()
    except rospy.ROSInterruptException:
        pass
