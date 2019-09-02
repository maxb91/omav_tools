#!/usr/bin/env python

from Tkinter import *
import rospy
import roslaunch
import rospkg
from std_srvs.srv import Trigger, SetBool, Empty
from sensor_fusion_comm.srv import InitScale
from mav_state_machine_msgs.srv import RunTaskService 
from mavros_msgs.srv import CommandBool
class App:
    def __init__(self, master):

        frame = Frame(master)
        frame.pack()
        self.frame = frame

        self.recording_active = "Recording not active."
        self.recording_color = "black"
        
        self.button_init = Button(frame, text="INIT MSF", bg="green", command=self.init, width=6, height=6)
        self.button_init.grid(row=0,column=0)
        
        self.button_takeoff = Button(frame, text="TAKE OFF", bg="cyan", command=self.takeoff, width=12, height=6)
        self.button_takeoff.grid(row=0,column=1)

        self.button_plan = Button(frame, text="PLAN", bg="yellow", command=self.plan, width=6, height=6)
        self.button_plan.grid(row=0,column=2)
        
        self.button_execute = Button(frame, text="EXECUTE", bg="cyan", command=self.execute, width=6, height=6)
        self.button_execute.grid(row=0,column=3)
        
        self.button_stop = Button(frame, text="STOP", bg="red", command=self.stop, width=6, height=6)
        self.button_stop.grid(row=0,column=4)
        
        self.button_land = Button(frame, text="LAND", bg="orange", command=self.land, width=12, height=6)
        self.button_land.grid(row=0,column=5)
        
        self.button_home = Button(frame, text="HOME", bg="orange", command=self.home, width=12, height=6)
        self.button_home.grid(row=0,column=6)
        
        self.button_home = Button(frame, text="HOME", bg="orange", command=self.home, width=12, height=6)
        self.button_home.grid(row=0,column=7)

        self.button_record_bag = Button(frame, text="Record bag", bg="cyan", command=self.recordBag, width=6, height=4)
        self.button_record_bag.grid(row=1,column=1)

        self.label_rosbag_info = Label(frame, text="Recording inactive.", fg="black")
        self.label_rosbag_info.grid(row=1,column=3)

        self.button_stop_bag = Button(frame, text="Stop bag", bg="cyan", command=self.stopBag, width=6, height=4)
        self.button_stop_bag.grid(row=1,column=2)
    
    def init(self):
        init_service = rospy.ServiceProxy('pose_sensor_vicon/pose_sensor/initialize_msf_scale', InitScale)
        init_service(1.0)
        print "initializing msf"

    def takeoff(self):
        takeoff_service = rospy.ServiceProxy('take_off', Empty)
        takeoff_service()
        print "Taking off"

    def plan(self):
        plan_service = rospy.ServiceProxy('load_file', Trigger)
        plan_service()
        print "Planning trajectory"
    
    def execute(self):
        execute_service = rospy.ServiceProxy('publish_path', Trigger)
        execute_service()
        print "Executing trajectory"
    
    def stop(self):
        stop_service = rospy.ServiceProxy('stop', Trigger)
        stop_service()
        print "Stopping trajectory"
    
    def land(self):
        land_service = rospy.ServiceProxy('mission/start_stop_task', RunTaskService)
        land_service("Landing",True)
        print "landing"
    
    def home(self):
        home_service = rospy.ServiceProxy('mission/start_stop_task', RunTaskService)
        home_service("Homing",True)
        print "Going home"
    
    def recordBag(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        rospack = rospkg.RosPack()
        record_launch_path = rospack.get_path('omav_gui') + "/launch/record_bag.launch"
        print "starting ", record_launch_path
        self.recordBagLaunch = roslaunch.parent.ROSLaunchParent(uuid, [record_launch_path])
        self.recordBagLaunch.start()
        print "Recording bag."
        self.label_rosbag_info.config(text="Recording active!", fg="red")
    
    def stopBag(self):
        self.recordBagLaunch.shutdown()
        self.label_rosbag_info.config(text="Recording inactive.", fg="black")
        print "Stopped recording."

root = Tk()
root.lift()
root.attributes('-topmost', True)
root.update()

app = App(root)

root.mainloop()