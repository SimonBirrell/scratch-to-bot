#! /usr/bin/python

# Connect to Scratch, read any broadcasts and handle any that are intended for the Camera bot

import time
import py_websockets_bot
import scratch
import time
import socket
import platform
import subprocess
import psutil
import threading
import logging

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-10s) %(message)s',
                    )

DEBUG = False

ROBOT_HOSTNAME = "robot.local"
VLC_ATTRIBUTES = 'http://robot.local:8080/?action=stream'
VLC_PROCESS_NAME = "VLC"

TIME_BETWEEN_SCRATCH_RECONNECTION_ATTEMPTS = 1.0
ROBOT_UPDATE_SLEEP = 0.1
SPEED_FACTOR = 10.0             # Converts cm/s to motor speed
DEFAULT_SPEED = 4.0             # cm/s
DEFAULT_DISTANCE = 4.0
MOVEMENT_SLEEP = 0.1            # Seconds to sleep between setting motors
ANGULAR_VELOCITY = 180.0         # Degrees per second on turn
ANGULAR_SPEED_FACTOR = 8.0      # Converts degrees per second to motor speed
ANGULAR_SLEEP = 0.05            # Seconds to sleep between setting motors

NECK_CENTRAL_ANGLE = 90.0        # Pan offset from centre in degrees
NECK_LIMIT = 80                 # Maximum angle neck can pan from adjusted centre
HEAD_CENTRAL_ANGLE = 0.0        # Tilt offset from centre in degrees
HEAD_LIMIT = 80                 # Maximum angle head can tilt from adjusted centre

Input_buffer = []
Stop_motors_immediately = False
MAX_LENGTH_INPUT_BUFFER = 100
Exit_program = False

############################# ScratchReader ########################################
#
# This class runs in a separate thread and reads in commands from Scratch, adding
# them to Input_buffer, a global list that is shared with the main thread.
# When a "robot stop" command is received from Scratch, this is placed at the 
# the front of the list, and any subsequent motor control commands are cancelled.
#   
# If the buffer gets full, then subsequent commands are dropped.
#         
            
class ScratchReader(threading.Thread):
    def __init__(self, buffer_lock, stop_motors_lock):
        self.scratch = scratch.Scratch()
        self.buffer_lock = buffer_lock
        self.stop_motors_lock = stop_motors_lock
        self.exit_reader = False
        print "ScratchReader connected to Scratch"
        threading.Thread.__init__(self, name="ScratchReader")
        self.daemon = True
        
    def run(self):
        while not self.exit_reader:
            raw_message = self.scratch.receive() 
            print "ScratchReader read message", raw_message
            self.add_message_to_buffer(raw_message)
        print "Scratch Reader finished"    

    def add_message_to_buffer(self, raw_message):
        global Stop_motors_immediately
        
        stop_motors = False
        tokens = split_into_tokens(raw_message)
        print "Tokenized message", raw_message
        with self.buffer_lock:
            if self.message_requires_immediate_execution(tokens):
                self.schedule_for_immediate_execution(tokens)
                stop_motors = True
            else:
                self.schedule_at_end_of_queue(tokens)
        if stop_motors:
            with self.stop_motors_lock:
                Stop_motors_immediately = True
                        
        
    def message_requires_immediate_execution(self, tokens):
        if len(tokens) < 3:
            return False
        return ((tokens[1]=='robot') and (tokens[2]=='stop'))
        
    def message_is_a_movement_command(self, tokens):
        return ((tokens[1]=='robot') and ((tokens[2]=='forward') or (tokens[2]=='backward') or (tokens[2]=='turn') or (tokens[2]=='neck') or (tokens[2]=='head')))

    def schedule_at_end_of_queue(self, tokens):
        global Input_buffer
        if len(Input_buffer) < MAX_LENGTH_INPUT_BUFFER:
            Input_buffer.append(tokens)            

    def schedule_for_immediate_execution(self, tokens):
        global Input_buffer
        
        Input_buffer.insert(0, tokens)
        i = 0
        print "Purging movement commands from buffer length", len(Input_buffer)
        while (i < len(Input_buffer)):
            print "Checking", Input_buffer[i]
            if self.message_is_a_movement_command(Input_buffer[i]):
                print "deleting"
                del Input_buffer[i]  
            else:    
                i = i + 1          
                
    def shutdown(self):
        print "Shutting down Scratch Reader" 
        self.exit_reader = True             
            
############################# End of ScratchReader #################################
   
############################# BufferReader #########################################
# 
# An object created with this class runs on the main thread and reads in buffered
# Scratch commands one at a time. It uses a lock on the globally scoped input buffer
# to coordinate with ScratchReader which is operating on a separate thread.
#
   
class BufferReader():
        def __init__(self, buffer_lock):
            self.lock = buffer_lock
    
        def get_next_command(self):
            command_tokens = None
            with self.lock:
                if len(Input_buffer) > 0:
                    command_tokens = Input_buffer[0]
                    del Input_buffer[0]                   
            return command_tokens

############################# End of BufferReader ##################################
           
############################# Robot Class ##########################################            
        
class Robot:
    def __init__(self, stop_motors_lock ):
        print "Initializing robot class"
        hostname = ROBOT_HOSTNAME
        self.speed = DEFAULT_SPEED
        self.neck_angle = 0
        self.head_angle = 0
        self.stop_motors_lock = stop_motors_lock
        if not DEBUG: 
            self.bot = py_websockets_bot.WebsocketsBot( hostname )
            print "Connected to robot"
            self.set_neck_and_head()
            self.set_sensor_config()
        self.exit_update = False            
        self.update_thread = threading.Thread(target=update_robot_loop, args=(self,))  
        self.update_thread.start() 
    def __enter__(self):
        print "__enter__"  
    def __exit__(self):
        if not DEBUG: self.bot.disconnect()    
        
    def set_sensor_config(self):
        print "Setting sensors"
        if not DEBUG:         
            # Configure the sensors on the robot
            sensorConfiguration = py_websockets_bot.mini_driver.SensorConfiguration(
                configD12=py_websockets_bot.mini_driver.PIN_FUNC_ULTRASONIC_READ)

            # We set the sensor configuration by getting the current robot configuration and modifying it.
            # In this way we don't trample on any other configuration settings
            robot_config = self.bot.get_robot_config()
            robot_config.miniDriverSensorConfiguration = sensorConfiguration
            self.bot.set_robot_config( robot_config )    
        
    def extract_distance(self, tokens):
        distance = to_integer(tokens[3]) if len(tokens)>=4 else DEFAULT_DISTANCE
        return distance     
        
    def extract_angle(self, tokens):
        angle = 0.0
        print tokens
        if len(tokens)>=3:
            parameter = tokens[2]
            print "parameter", parameter
            if parameter=='left':
                angle = -90.0
            elif parameter=='right': 
                angle = 90.0
            elif (parameter=='centre') or (parameter=='center'): 
                angle = 0.0
            else:
                angle = to_integer(parameter)
        return angle
        
    def extract_on_or_off(self, tokens, position=2):
        result = None
        if (len(tokens)>=position+1):
            on_or_off = tokens[position]
            if (on_or_off=='on') or (on_or_off=='off'):
                result = on_or_off
        return result   

    def robot_forward(self, distance):
        global Stop_motors_immediately
        
        print "Move robot forward ", distance, " cm"
        if self.speed != 0:
            print "Speed", self.speed
            motor_time = abs(distance / float(self.speed))
            print "Loop for ", motor_time, " seconds"
            timeout = time.time() + motor_time
            while True:
                if time.time() > timeout:
                    break
                motor_speed = self.speed * SPEED_FACTOR 
                stop_motors = False
                with self.stop_motors_lock:  
                    if Stop_motors_immediately:
                        Stop_motors_immediately = False    
                        stop_motors = True
                if stop_motors:
                    print "Stopping motors immediately"
                    self.robot_set_motor_speeds(0.0, 0.0)
                    break
                else:
                    self.robot_set_motor_speeds(motor_speed, motor_speed)
                    time.sleep(MOVEMENT_SLEEP)    
            print "Finished robot_forward ", distance  
            self.robot_set_motor_speeds( 0.0, 0.0 )   
                
    def robot_set_motor_speeds(self, left_motor_speed, right_motor_speed):   
        print "robot_set_motor_speeds", left_motor_speed, right_motor_speed
        if not DEBUG: 
            self.bot.set_motor_speeds( left_motor_speed, right_motor_speed )           
            
    def robot_stop(self):
        print "Stop robot motors"   
        self.robot_set_motor_speeds( 0.0, 0.0)     
            
    def robot_turn(self, angle):
        print "Robot turn ", angle 
        motor_time = abs(angle / ANGULAR_VELOCITY)
        print "Loop for ", motor_time, " seconds"
        timeout = time.time() + motor_time
        print "timeout ", timeout
        motor_speed = self.speed * ANGULAR_SPEED_FACTOR     
        if angle <= 0:
            left_motor_speed = -motor_speed
            right_motor_speed = motor_speed
        else:
            left_motor_speed = motor_speed
            right_motor_speed = -motor_speed
        while True:
            print "loop ", time.time()
            if time.time() > timeout:
                print "BREAK ************"
                break
            print "set_motor_speeds to ", left_motor_speed, right_motor_speed
            if not DEBUG:
                self.bot.set_motor_speeds( left_motor_speed, right_motor_speed ) 
            time.sleep(ANGULAR_SLEEP)  
        if not DEBUG:
            self.bot.set_motor_speeds( 0, 0 ) 
            
    def adjust_angle(self, angle, centre_adjust, limit):        
        angle = angle
        if angle < -limit:
            angle = -limit
        elif angle > limit:
            angle = limit  
        angle = angle + centre_adjust    
        return angle               
        
    def robot_neck(self, angle):
        self.neck_angle = angle
        print "Robot neck ", angle 
        self.set_neck_and_head()
            
    def robot_head(self, angle):
        self.head_angle = angle
        print "Robot head ", angle 
        self.set_neck_and_head()

    def set_neck_and_head(self):
        adjusted_neck_angle = self.adjust_angle(self.neck_angle, NECK_CENTRAL_ANGLE, NECK_LIMIT)
        print "Neck angle ", self.neck_angle, " Adjusted ", adjusted_neck_angle
        adjusted_head_angle = self.adjust_angle(self.head_angle, HEAD_CENTRAL_ANGLE, HEAD_LIMIT)
        print "Pan / tilt set to ", adjusted_neck_angle, adjusted_head_angle
        if not DEBUG:
            self.bot.set_neck_angles( pan_angle_degrees=adjusted_neck_angle, tilt_angle_degrees=adjusted_head_angle)                

    def handle_robot_command(self, tokens=[]):
        print tokens
        if len(tokens) > 1:
            robot_command = tokens[1]
            if robot_command=='forward':      
                self.robot_forward(self.extract_distance(tokens))
            elif robot_command=='backward':      
                self.robot_forward(-self.extract_distance(tokens))     
            elif robot_command=='turn':      
                self.robot_turn(self.extract_angle(tokens))     
            elif robot_command=='neck':
                self.robot_neck(self.extract_angle(tokens))
            elif robot_command=='head':
                self.robot_head(self.extract_angle(tokens))
            elif robot_command=='stop':
                self.robot_stop()    
            elif robot_command=='camera':
                self.robot_camera(self.extract_on_or_off(tokens))
            else:
                print "Unknown command for robot"    
        else:
            print "Empty command sent to robot"        
            
    def robot_camera(self, on_or_off):
        print "on_or_off", on_or_off
        if on_or_off=='on':
            self.open_camera_window()
            self.start_streaming_images()
        elif on_or_off=='off':
            self.stop_streaming_images()
            self.close_camera_window()
            
    def open_camera_window(self):
        print "Open camera window"
        open_vlc_instance()
        
    def close_camera_window(self):
        print "Close camera window"
        kill_all_vlc_instances()            

    def start_streaming_images(self):
        print "Start streaming images"
        if not DEBUG:
            self.bot.start_streaming_camera_images()

    def stop_streaming_images(self):
        print "Stop streaming images"
        if not DEBUG:
            self.bot.stop_streaming_camera_images()
            
    def update(self):
        print "update"
        if not DEBUG:
            #self.bot.update()  
            #bot.update() is not compatible with Mac OSX
            self.bot._update_camera_keep_alive()    
        self.update_sensors()
            
    def update_sensors(self):
        if not DEBUG:
            # Read sensors
            status_dict, read_time = self.bot.get_robot_status_dict()
            sensor_dict = status_dict[ "sensors" ]
            ultrasonic = sensor_dict["ultrasonic"]
            ultrasonic_distance = ultrasonic["data"] 
            print "Ultrasonic Distance ", ultrasonic_distance
            s.sensorupdate({'distance' : ultrasonic_distance})
            
    def shutdown(self):
        print "Shutting down robot" 
        self.exit_update = True
        self.update_thread.join() 
        print "Robot shut down"
        
    def exit_update_now(self):
        return self.exit_update    

############################# End of Robot Class ###################################                        

############################# Utility robot functions ##############################                        

def update_robot_loop(robot):
    while (robot.exit_update_now()== False):
        try:
            robot.update()
            time.sleep(ROBOT_UPDATE_SLEEP)  
        except KeyboardInterrupt:
            print "Update thread received keyboard interrupt"      
            break
        except:
            print "update failed"
            break    
    print "Exiting robot update thread"        
            
############################# End of Utility robot functions #######################                        

def to_integer(string):
    try:
        return int(string)
    except ValueError:
        return 0

def split_into_tokens(message):
    command_type = message[0].lower()
    scratch_command = message[1].lower()
    tokens = [command_type] + scratch_command.split()
    return tokens       
    
############################# Platform utility functions ###########################                                                  
                    
def open_vlc_instance():
     global VLC
     kill_all_vlc_instances()
     subprocess.Popen([VLC, VLC_ATTRIBUTES], stdout=subprocess.PIPE)           
            
def kill_all_vlc_instances():
    print "Preparing to kill open VLC windows"
    for proc in psutil.process_iter():
        if proc.name() == VLC_PROCESS_NAME:
            print "Killing!"
            proc.kill()   
  
def adjust_to_platform():
    global VLC
    if platform.system()=='Darwin':
        print "Running on Mac"
        VLC = "/Applications/VLC.app/Contents/MacOS/VLC"
    else:
        print "Running on ", platform.system()
        VLC = "vlc"               

############################# Legacy functions ###################################                                     

############################# End of Legacy functions ############################                                     
        
############################# The Main Loop ######################################                                     

if __name__ == "__main__":
    adjust_to_platform()
    while True:
        try: 
            buffer_lock = threading.Lock()
            stop_motors_lock = threading.Lock()
            # Read Scratch commands on a separate thread and insert them into the Input_buffer
            scratch_reader = ScratchReader(buffer_lock, stop_motors_lock)
            scratch_reader.start() 
            # BufferProcessor pulls commands off the queue
            buffer_reader = BufferReader(buffer_lock)
            # This is the Robot
            robot = Robot(stop_motors_lock)
            while True:
                # Interpret Scratch commands on main thread
                next_command = buffer_reader.get_next_command()
                if next_command != None:
                    if next_command[0] == 'broadcast':
                        if (len(next_command) > 1) and (next_command[1] == 'robot'):
                            robot_tokens = next_command
                            del robot_tokens[0]
                            robot.handle_robot_command(robot_tokens)
                    elif next_command[0] == 'sensor-update':
                        print "TO DO: sensor update code" 
                else:        
                    time.sleep(0.05)                                              
        except scratch.ScratchError:
            print "No connection to Scratch." 
            time.sleep(TIME_BETWEEN_SCRATCH_RECONNECTION_ATTEMPTS)  
        except socket.error:
            print "Socket error connecting to robot"
        except KeyboardInterrupt:
            print "User requested break"
            break  
    print "Shutting down"
    scratch_reader.shutdown()
    robot.shutdown()
              
    
                  