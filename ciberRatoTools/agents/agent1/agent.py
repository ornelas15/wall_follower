
import math
import random
import xml.etree.ElementTree as ET
import numpy as np
from croblink import CRobLinkAngs
import sys


CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        # Sensor
        self.halfwall_size = 0.1
        self.robot_radius = 0.5
        # Coordinates
        self.pos = None
        self.rob_coords = []  
        self.cnum = 1 
        self.prev_cnum = 0   
        # Mov model
        self.Vl = 0
        self.Vr = 0
        self.out_Vl = 0
        self.out_Vr = 0 
        self.ori = 0
        self.target_ori = 0
        #
        self.cell_type = "rw"
        
        
        self.last_coord = None
        self.next_coord = None  
        self.prev_ori = None
        self.prev_action = None
        self.vector = 0
        self.angle = 0
        self.target_ori = 0

    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))
     
    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()        
                    
        state = 'stop'
        stopped_state = 'return'
        
        while True:
            if state != 'return':
                self.prev_ground = 0
            else:
                self.prev_ground = self.measures.ground    
            
            #read sensors
            self.readSensors()   

            # Load initial position and orientation
            if self.pos == None:
                self.pos = init_pos = (0,0)

            # Add initial position
            if init_pos not in self.rob_coords:
                self.rob_coords.append(init_pos)
            
            # call following line method
            if state == 'stop' and self.measures.start:
                state = stopped_state
            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'
            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)

                # Load sensor data
                sensor = self.sensor_info()  

                # Estimate position with mov.model
                self.mov_model()

                # Get explored coordinates
                self.retrieve_coordinates()  

                # Control actions based on sensor data
                action = self.control(sensor)  

                # Follow wall according to right sensor     
                self.navigate(action)

                if self.measures.time == self.simTime:
                    exit()
                                           
    def sensor_info(self):
        """ Load sensor data """
        # Avoid float division by 0:
        for i in range(4):
            if self.measures.irSensor[i] == 0:
                self.measures.irSensor[i] = 0.1
        # Get distance to walls
        front = round(((1/self.measures.irSensor[0])-self.halfwall_size),2) 
        left = round(((1/self.measures.irSensor[1])-self.halfwall_size),2) 
        right = round(((1/self.measures.irSensor[2])-self.halfwall_size),2) 
        rear = round(((1/self.measures.irSensor[3])-self.halfwall_size),2) 
        sensor = [front,left,right,rear]  
        # Fix -10 reading -> seems to be from not detecting any wall
        for i in range(4):
            if sensor[i] <= -2:
                sensor[i] = 10
        return sensor
    
    def mov_model(self):
        """ Movement model with IIR filter """        
        # Limit max speed
        max_speed = 0.05
        
        # Add noise
        motors_noise = random.gauss(1,max_speed**2)
        
        # Load previous position
        x_t0, y_t0 = self.pos
        
        # Calculate power component
        self.out_Vl_t = ((self.Vl + self.out_Vl) / 2.0) * motors_noise
        self.out_Vr_t = ((self.Vr + self.out_Vr) / 2.0) * motors_noise 
        
        # Limit the wheel speeds to the maximum speed
        self.out_Vl_t = min(self.out_Vl_t, max_speed)
        self.out_Vr_t = min(self.out_Vr_t, max_speed)
        
        # Linear vel
        lin = (self.out_Vr_t + self.out_Vl_t) / 2.0
        rot = self.out_Vr_t - self.out_Vl_t
        
        # Calculate linear component:
        x_t1 = x_t0+lin * np.cos(self.ori)
        y_t1 = y_t0+lin * np.sin(self.ori)

        # Calculate rotation component:
        teta_t1 = self.ori + rot
        # Ensure -pi to pi range
        teta_t1 = ((teta_t1 + np.pi) % (2 * np.pi)) - np.pi
        
        # Return the estimated next pose
        self.pos = (round(x_t1,2), round(y_t1,2)) 
        
        # Update position
        self.out_Vl = self.out_Vl_t
        self.out_Vr = self.out_Vr_t
        self.ori = round(teta_t1,2)
   
    def retrieve_coordinates(self):
        """ Retrieve coordinates based on estimated positions """  
        # Initialize readings
        distance_x = 0
        distance_y = 0
        x = round(self.pos[0],1)
        y = round(self.pos[1],1)   
        # Get distance absolute value 
        distance_x = abs(x - self.rob_coords[-1][0])
        distance_y = abs(y - self.rob_coords[-1][1])        
        # Measure distance compared to last coord
        if distance_x >= 1.90 or distance_y >= 1.90:
            prev_point = self.rob_coords[-1]
            curr_point = x,y
            new_point = adjust_position(prev_point, curr_point)

            # Add coords to list
            if new_point != self.rob_coords[-1]:
                self.rob_coords.append(new_point)
            self.cnum += 1
            self.pos = new_point
    
    def calculate_turn(self, direction, ori_deg):
        """ Calculate a new target orientation given a left or right turn """
        # Normalizing initial orientation to -180 to 180 range
        if ori_deg > 180:
            ori_deg -= 360
        elif ori_deg < -180:
            ori_deg += 360

        if direction == "left":
            target_ori = ori_deg + 90
        else:
            target_ori = ori_deg - 90
        
        # Normalize target orientation to -180 to 180 range
        if target_ori > 180:
            target_ori -= 360
        elif target_ori < -180:
            target_ori += 360
        
        return target_ori
    
    def angle_difference(self, angle1, angle2):
        """ Calculate the smallest difference between two angles """
        diff = (angle2 - angle1 + 180) % 360 - 180
        return diff + 360 if diff < -180 else diff

    def control(self, sensor):
        """ Choose action based on sensor data """
        # Convert orientation to degrees and normalize to -180 to 180
        ori_deg = (self.ori * (180 / np.pi))
        if ori_deg > 180:
            ori_deg -= 360
        elif ori_deg < -180:
            ori_deg += 360
        action = None

        # Check sensor readings on new cell
        if self.cnum > self.prev_cnum:
            rounded_ori = round(ori_deg / 90) * 90 % 360
            # Check cell types (fw - front wall, rw - right wall, nw - no wall)
            if sensor[0] <= 0.8:
                self.cell_type = "fw"
                self.target_ori = self.calculate_turn("left", rounded_ori)
            elif sensor[0] >= 1.5 and sensor[2] >= 1.5:
                self.cell_type = "nw"
                self.target_ori = self.calculate_turn("right", rounded_ori)
            else:
                self.cell_type = "rw"

            self.prev_cnum += 1
            action = "move"

        # Types of cells
        elif self.cell_type == "fw":
            # Forward wall -> rotate left
            if self.angle_difference(ori_deg, self.target_ori) > 10:
                action = "rotate left"
            else:
                action = "move"

        elif self.cell_type == "nw":
            # No wall -> rotate right
            if self.angle_difference(ori_deg, self.target_ori) < -10:
                action = "rotate right"
            else:
                action = "move"

        elif self.cell_type == "rw":
                # Perform noise adjustments if needed
                if sensor[2] <= 0.4:
                    action = "turn left"
                elif sensor[2] > 0.6:
                    action = "turn right"
                else:
                    action = "move"

        else:
            action = "move"  

        return action

          
    def navigate(self, action):
        """ Navigate based on actions"""   
        
        # Based on distances to wall, perform actions
        if action == "rotate right":
            self.driveMotors(0.05, -0.05)
            self.Vl = 0.05
            self.Vr = -0.05
        elif action == "rotate left":
            self.driveMotors(-0.05, 0.05)
            self.Vl = -0.05
            self.Vr = 0.05
        elif action == "turn right":
            self.driveMotors(0.05,0.03)
            self.Vl = 0.05
            self.Vr = 0.03
        elif action == "turn left":
            self.Vl = 0.03
            self.Vr = 0.05
            self.driveMotors(0.03,0.05)
        elif action == "move":
            self.driveMotors(0.05,0.05)
            self.Vl = 0.05
            self.Vr = 0.05
        elif action == "stop":
            self.driveMotors(0.0,0.0)
            self.Vl = 0.00
            self.Vr = 0.00
        else:
            self.Vl = -0.05
            self.Vr = 0.05
            self.driveMotors(-0.05,0.05)  


# Aux functions:          
def adjust_position(previous_point, current_point):
    prev_x, prev_y = previous_point
    curr_x, curr_y = current_point

    # Determine the axis with the larger movement
    delta_x = curr_x - prev_x
    delta_y = curr_y - prev_y

    if abs(delta_x) > abs(delta_y):
        # Adjust the x coordinate
        new_x = int(round(curr_x))
        new_y = prev_y 
    else:
        # Adjust the y coordinate
        new_x = prev_x  
        new_y = int(round(curr_y))

    return new_x, new_y


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None

           i=i+1

rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()