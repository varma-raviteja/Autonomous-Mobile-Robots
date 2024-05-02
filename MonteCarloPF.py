# You may need to import some classes of the controller module. Ex:

#  from controller import Robot, Motor, DistanceSensor

from controller import Robot

# import numpy as it may be used in future labs

import numpy as np

import math

#######################################################

# Creates Robot

#######################################################

robot = Robot()

#######################################################

# Sets the time step of the current world

#######################################################

timestep = int(robot.getBasicTimeStep())

#######################################################

# Gets Robots Distance Sensors

# Documentation:

#  https://cyberbotics.com/doc/reference/distancesensor

#######################################################

frontDistanceSensor = robot.getDevice('front distance sensor')

leftDistanceSensor = robot.getDevice('left distance sensor')

rightDistanceSensor = robot.getDevice('right distance sensor')

rearDistanceSensor = robot.getDevice('rear distance sensor')

frontDistanceSensor.enable(timestep)

leftDistanceSensor.enable(timestep)

rightDistanceSensor.enable(timestep)

rearDistanceSensor.enable(timestep)

#######################################################

# Gets Robots Lidar Distance Sensors

# Documentation:

#  https://cyberbotics.com/doc/reference/lidar

#######################################################

lidar = robot.getDevice('lidar')

lidar.enable(timestep)

lidar_horizontal_res = lidar.getHorizontalResolution()

lidar_num_layers = lidar.getNumberOfLayers()

lidar_min_dist = lidar.getMinRange()

lidar_max_dist = lidar.getMaxRange()

print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res,
      "\nLidar Image Number of Layers: ", lidar_num_layers)

print("Lidar Range: [", lidar_min_dist, " ,", lidar_max_dist, '] in meters')

#######################################################

# Gets Robots Camera

# Documentation:

#  https://cyberbotics.com/doc/reference/camera

#######################################################

camera_front = robot.getDevice('cameraFront')

camera_front.enable(timestep)

camera_front.recognitionEnable(timestep)

camera_right = robot.getDevice('cameraRight')

camera_right.enable(timestep)

camera_right.recognitionEnable(timestep)

camera_rear = robot.getDevice('cameraRear')

camera_rear.enable(timestep)

camera_rear.recognitionEnable(timestep)

camera_left = robot.getDevice('cameraLeft')

camera_left.enable(timestep)

camera_left.recognitionEnable(timestep)

#######################################################

# Gets Robots Motors

# Documentation:

#  https://cyberbotics.com/doc/reference/motor

#######################################################

leftMotor = robot.getDevice('left wheel motor')

rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))

rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0)

rightMotor.setVelocity(0)

#######################################################

# Gets Robot's the position sensors

# Documentation:

#  https://cyberbotics.com/doc/reference/positionsensor

#######################################################

leftposition_sensor = robot.getDevice('left wheel sensor')

rightposition_sensor = robot.getDevice('right wheel sensor')

leftposition_sensor.enable(timestep)

rightposition_sensor.enable(timestep)

#######################################################

# Gets Robot's IMU sensors

# Documentation:

#  https://cyberbotics.com/doc/reference/inertialunit

#######################################################

imu = robot.getDevice('inertial unit')

imu.enable(timestep)


def turn(angle, positions):
    while robot.step(timestep) != -1:
        pos = round((imu.getRollPitchYaw()[2] * (180 / math.pi)))
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(-2)
        if (pos == 0):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    print("searching")
    while robot.step(timestep) != -1:
        pos = round((imu.getRollPitchYaw()[2] * (180 / math.pi)))
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(-2)
        obj_in_view = len(camera_front.getRecognitionObjects())
        if (obj_in_view > 0):
            pos_dist = camera_front.getRecognitionObjects()[0].getPosition()[0]
            a = camera_front.getRecognitionObjects()[0].colors[0]
            b = camera_front.getRecognitionObjects()[0].colors[1]
            c = camera_front.getRecognitionObjects()[0].colors[2]
            p = camera_front.getRecognitionObjects()[0].getPositionOnImage()[0]
            if (a == 1 and b == 0 and c == 0) and (p >= 40 and p <= 50):
                print("Red found")
                positions[0] = pos_dist
            elif (a == 0 and b == 1 and c == 0) and (p >= 40 and p <= 50):
                print("Green found")
                positions[1] = pos_dist
            elif (a == 0 and b == 0 and c == 1) and (p >= 40 and p <= 50):
                print("Blue found")
                positions[2] = pos_dist
            elif (a == 1 and b == 1 and c == 0) and (p >= 40 and p <= 50):
                print("Yellow found")
                positions[3] = pos_dist
        if (angle == pos):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    return positions


def setposition():
    while robot.step(timestep) != -1:
        pos=round((imu.getRollPitchYaw()[2]*(180/math.pi)))
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(-2)
        if(pos==90):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break



            
def determine_xy(positions):
    r1=positions[0]*39.3701
    r2=positions[1]*39.3701
    r3=positions[2]*39.3701
    
    x1=20
    y1=20
    x2=-20
    y2=-20
    x3=20
    y3=-20
    
    A=-2*x1+2*x2
    B=-2*y1+2*y2
    C=r1*r1 - r2*r2 -x1*x1 + x2*x2 -y1*y1 + y2*y2
    D=-2*x2+2*x3
    E=-2*y2+2*y3
    F=r2*r2 - r3*r3 -x2*x2 + x3*x3 -y2*y2 + y3*y3
    
    x=(C*E-F*B)/(E*A-B*D) 
    y=(C*D-A*F)/(B*D-A*E)
    res=[]
    res.append(x)
    res.append(y)
    return res


def determine_grid(xy):
    x=xy[0]
    y=xy[1]
    if(x<=-10 and x>=-20 and y>=10 and y<=20):
        return 1
    elif(x<=0 and x>=-10 and y>=10 and y<=20):
        return 2
    elif(x>=0 and x<=10 and y>=10 and y<=20):
        return 3
    elif(x>=10 and x<=20 and y>=10 and y<=20):
        return 4
    elif(x<=-10 and x>=-20 and y>=0 and y<=10):
        return 5
    elif(x<=0 and x>=-10 and y>=0 and y<=10):
        return 6
    elif(x>=0 and x<=10 and y>=0 and y<=10):
        return 7
    elif(x>=10 and x<=20 and y>=0 and y<=10):
        return 8
    elif(x<=-10 and x>=-20 and y<=0 and y>=-10):
        return 9
    elif(x<=0 and x>=-10 and y<=0 and y>=-10):
        return 10
    elif(x>=0 and x<=10 and y<=0 and y>=-10):
        return 11
    elif(x>=10 and x<=20 and y<=0 and y>=-10):
        return 12
    elif(x<=-10 and x>=-20 and y<=-10 and y>=-20):
        return 13
    elif(x<=0 and x>=-10 and y<=-10 and y>=-20):
        return 14
    elif(x>=0 and x<=10 and y<=-10 and y>=-20):
        return 15
    elif(x>=10 and x<=20 and y<=-10 and y>=-20):
        return 16    


direction = -1


def perform_motion():
    global direction
    direction = -1
    while robot.step(timestep) != -1:
        if (checker == 0):
            p = imu.getRollPitchYaw()[2]
            if (p < 0):
                p = p + 6.28
            dif = traverse[0] - p
            traverse.pop(0)
            set_time_rotation(dif)
            checker = 1
        if (direction != -1):
            if (rotation_time > 0):
                print(time)
                if (direction == 'a'):
                    set_velocity(-2, 2)
                if (direction == 'c'):
                    set_velocity(2, -2)
                rotation_time = rotation_time - 0.032
                if (rotation_time <= 0):
                    rotation_time = -1
                    continue
                else:
                    continue
            elif (time > 0):
                set_velocity(5, 5)
                time = time - 0.032
                continue
            else:
                direction = -1
                continue
        if (len(traverse) == 0):
            set_velocity(0, 0)
            break
        i = traverse.pop(0)
        p = imu.getRollPitchYaw()[2]
        if (p < 0):
            p = p + 6.28
        dif = i - p
        set_time_rotation(dif)


def getNeighbours(node):
    if (node == [0, 0]):
        return [[0, 1]]
    if (node == [0, 1]):
        return [[0, 0], [0, 2]]
    if (node == [0, 2]):
        return [[0, 3], [1, 2], [0, 1]]
    if (node == [0, 3]):
        return [[0, 2], [1, 3]]
    if (node == [1, 0]):
        return [[1, 1], [2, 0]]
    if (node == [1, 1]):
        return [[1, 0], [2, 1]]
    if (node == [1, 2]):
        return [[0, 2], [2, 2]]
    if (node == [1, 3]):
        return [[0, 3], [2, 3]]
    if (node == [2, 0]):
        return [[1, 0], [3, 0]]
    if (node == [2, 1]):
        return [[1, 1], [2, 2]]
    if (node == [2, 2]):
        return [[1, 2], [2, 1]]
    if (node == [2, 3]):
        return [[1, 3], [3, 3]]
    if (node == [3, 0]):
        return [[2, 0], [3, 1]]
    if (node == [3, 1]):
        return [[3, 0], [3, 2]]
    if (node == [3, 2]):
        return [[3, 1], [3, 3]]
    if (node == [3, 3]):
        return [[2, 3], [3, 2]]


def dfs(matrix, visited, current, path):
    if (len(visited) == 16):
        return True
    if current not in visited:
        visited.append(current)
    path.append(current)
    neighbours = getNeighbours(current)
    for i in neighbours:
        if (i not in visited):
            dfs(matrix, visited, i, path)
            path.append(current)

matrix = [[1, 1, 1, 1],
          [1, 1, 1, 1],
          [1, 1, 1, 1],
          [1, 1, 1, 1]]

visited = []
path = []
backTrackPath = []

# perform simulation steps until Webots is stopping the controller
checker = 0
time = -1
motion = -1
rotation_time = -1


def set_velocity(l, r):
    leftMotor.setVelocity(l / 0.8)
    rightMotor.setVelocity(r / 0.8)


# Main loop:
# perform simulation steps until Webots is stopping the controller


def set_time_rotation(v):
    global direction
    global rotation_time
    global time
    if (v < 0):
        direction = 'c'
    if (v > 0):
        direction = 'a'
    rotation_time = (abs(v)) / (4 / 2.28)
    time = 2
    print(rotation_time, time, direction)
    return


def perform_dfs_motion():
    global checker
    global direction
    global rotation_time
    global time
    while (robot.step(timestep) != -1):
        f = frontDistanceSensor.getValue()
        if checker == 0:
            p = imu.getRollPitchYaw()[2]
            if (p < 0):
                p = p + 6.28
            dif = traverse[0] - p
            traverse.pop(0)
            set_time_rotation(dif)
            checker = 1
        if (direction != -1):
            if (rotation_time > 0):
                print(time)
                if (direction == 'a'):
                    set_velocity(-2, 2)
                if (direction == 'c'):
                    set_velocity(2, -2)
                rotation_time = rotation_time - 0.032
                if (rotation_time <= 0):
                    rotation_time = -1
                    continue
                else:
                    continue
            elif (time > 0):
                print("here")
                if(f<0.14):
                    time=-1
                    continue
                set_velocity(5, 5)
                time = time - 0.032
                continue
            else:
                direction = -1
                continue
        if (len(traverse) == 0):
            set_velocity(0, 0)
            break
        i = traverse.pop(0)
        p = imu.getRollPitchYaw()[2]
        if (p < 0):
            p = p + 6.28
        dif = i - p
        set_time_rotation(dif)


dist_array=[[0.8, 0, 0, 0.1],
[0.6, 0, 0, 0.3],
[0.3, 0, 0, 0.6],
[0.1, 0, 0.8, 0.8],
[0, 0.6, 0, 0],
[0, 0, 0, 0],
[0, 0, 0, 0],
[0.3, 0, 0.5, 0],
[0, 0.3, 0, 0],
[0, 0, 0, 0],
[0, 0, 0, 0],
[0.6, 0, 0.3, 0],
[0, 0.2, 0.9, 0],
[0, 0.3, 0.5, 0],
[0, 0.6, 0.3, 0],
[0.8, 0.8, 0.1, 0]]


grid_layout=[[0,1,0,0],[0,1,0,1],[0,1,1,1],[0,0,1,1],[0,1,1,0],[0,0,1,1],
[1,0,1,0],[1,0,1,0],[1,0,1,0],[1,1,0,0],[1,0,0,1],[1,0,1,0],[1,1,0,0],
[0,1,0,1],[0,1,0,1],[1,0,0,1]]

def rotate(angle):
    while robot.step(timestep) != -1:
        pos=round((imu.getRollPitchYaw()[2]*(180/math.pi)))
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(-2)
        if(pos==angle):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break   

def moveforward():
    old_pos_left=leftposition_sensor.getValue()
    old_pos_right=rightposition_sensor.getValue()
    radius_in_meters = .8/39.37
    circumference_in_meters = 2 * 3.141 * radius_in_meters
    encoder_unit=circumference_in_meters / 6.28
    l_distance=0
    r_distance=0
    distance=10
           
    distance=distance*0.05/2
    while robot.step(timestep) != -1:
       left_2p_sensor=leftposition_sensor.getValue() -old_pos_left
       right_2p_sensor=rightposition_sensor.getValue()-old_pos_right
       l_2distance= left_2p_sensor * encoder_unit
       r_2distance= right_2p_sensor * encoder_unit
    
       if l_2distance >= distance or r_2distance >= distance:
           leftMotor.setVelocity(0)
           rightMotor.setVelocity(0)
           break
       leftMotor.setVelocity(6.28)
       rightMotor.setVelocity(6.28)   

class Particles:
    def __init__(self,id, x, y, orientation,existence,weight):
        self.id=id
        self.x = x
        self.y = y
        self.orientation=orientation
        self.existence=existence
        self.weight=weight

def distaf(w_positions,p_list):
    print("INSIDE DIST")
    global dist_array
    x=[]
    for obj in  w_positions:
            x.append(round(obj,1))
            
    c=1
    for i in range(len(dist_array)):
        if(dist_array[i]==x):
            return c 
        c=c+1
    print(x)
    


def similarity_function(w_positions):
    a=w_positions[0]
    b=w_positions[1]
    c=w_positions[2]
    d=w_positions[3]
    
    if(a==0 and b==0 and c==0 and d==0):
        return [6,7,10,11]
    elif(a==0 and b!=0 and c==0 and d==0):
        return [5,9]
    elif(a!=0 and b==0 and c==0 and d!=0):
        return [1,2,3]
    elif((a!=0 and b==0 and c!=0 and d!=0) or(a!=0 and b!=0 and c!=0 and d==0)):
        return [4,16]
    elif(a!=0 and b==0 and c!=0 and d==0):
        return [8,12]
    elif(a==0 and b!=0 and c!=0 and d==0):
        return [13,14,15]

def move_particles(angle):
    x=0
    y=0
    index=-1
    print("Inside MOVEEEEEEEEEEEEEE")
    global p_list
    global grid_layout
    if (angle==90):
        y=y+10
        index=0
    elif (angle==0):
        x=x+10
        index=1
    elif (angle==180):
        x=x-10
        index=3
    elif (angle==-90):
        y=y-10
        index=2
    i=0
    m_list=[]
    for list in p_list:
        if(grid_layout[i][index]==1):
            print("YES",i+1)
            list.x=list.x+x
            list.y=list.y+y
            orientation=angle
            m_list.append(Particles(list.id,list.x,list.y,angle,1,list.weight))
        else:
            print("No",i+1)
        i=i+1
    return m_list
    
counts = dict()        

def update(p_list,landmarks_similarity_list):
    u_list=[]
    u_p_list=[]
    for i in p_list:
        u_list.append(i.id)
    for i in landmarks_similarity_list:
        if(i in u_list):
            u_list.append(i)
    global counts
    for i in u_list:
        counts[i] = counts.get(i, 0) + 1
    print(u_list)
    print(counts)   
    weight=1/len(u_list)
    for i in p_list:
        i.weight=counts[i.id]*weight
        u_p_list.append(Particles(i.id,i.x,i.y,i.orientation,i.existence,i.weight))
    return u_p_list   

def resample(p_list):
    global counts
    print(counts)
    r_list=[]
    c=0
    
    for i in counts:
        if counts[i]==2:
            c=c+1
    for i in p_list:
        if counts[i.id]==2:
            i.weight=1/c
            r_list.append(Particles(i.id,i.x,i.y,i.orientation,i.existence,i.weight))
    return r_list            


def reintialize(p_list):
    global comat
    l= len(p_list)
    if(l!=0):
        nor=int(16/l)
    r_list=[]
    weight=1/16
    for i in p_list:
        for j in range(nor):
            r_list.append(Particles(i.id,comat[i.id-1][0],comat[i.id-1][1],i.orientation,i.existence,weight))    
    return r_list
      
def monte_carlo_loc():
    global p_list
    maxDist=0
    count=0
    list=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
    while(count<10):
        front_dist = frontDistanceSensor.getValue()
        right_dist = rightDistanceSensor.getValue()
        rear_dist = rearDistanceSensor.getValue()
        left_dist = leftDistanceSensor.getValue()
        maxDist=max(front_dist,right_dist,rear_dist,left_dist)
        print("MAXXXXXXXXXXXX:\t",maxDist)
        if(maxDist==front_dist):
            rotate(90)
            angle=90  
        elif(maxDist==right_dist):
            rotate(0)
            angle=0 
        elif(maxDist==rear_dist):
            rotate(-90)
            angle=-90
        elif(maxDist==left_dist):
            rotate(180)
            angle=180
        moveforward()
        p_list=move_particles(angle)
        print("**************MOVE*************************")
        for obj in  p_list:
            print(obj.id,obj.x, obj.y,obj.orientation, obj.existence,obj.weight)
        w_positions = [0, 0, 0, 0]
        w_positions = turn(89, w_positions)
        print("W POSSSS:\t",w_positions)
        landmarks_similarity_list=similarity_function(w_positions)
        print(landmarks_similarity_list)
        
        
        p_list=update(p_list,landmarks_similarity_list)
        print("**************UPDATE*************************")
        for obj in  p_list:
            print(obj.id,obj.x, obj.y,obj.orientation, obj.existence,obj.weight)
        
        p_list=resample(p_list)
        print("**************RESAMPLE*************************")
        for obj in  p_list:
            print(obj.id,obj.x, obj.y,obj.orientation, obj.existence,obj.weight)
        count=count+1
        if(count==2):
            rotate(angle)
            moveforward()
            w_positions = [0, 0, 0, 0]
            w_positions = turn(89, w_positions)
            print("W POSSSS:\t",w_positions)
            landmarks_similarity_list=similarity_function(w_positions)
            print(landmarks_similarity_list)
            v=distaf(w_positions,p_list)
            print(v)
            return v
            break          
      
        p_list=reintialize(p_list)
        print("**************REINTIALIZE*************************")
        for obj in  p_list:
            print(obj.id,obj.x, obj.y,obj.orientation, obj.existence,obj.weight)
        
        print("*******************************************", count)
        

comat=[[-15,15],[-5,15],[5,15],[15,15],[-15,5],[-5,5],[5,5],[15,5],[-15,-5],[-5,-5],[5,-5],[15,-5],[-15,-15],[-5,-15],[5,-15],[15,-15]]

N=16
p_list=[]
weight=16/N
weight=int(weight)


while robot.step(timestep) != -1:

    # Read the sensors:

    # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances

    full_range_image = lidar.getRangeImage()

    # print size of Range Image

    print('#################################################################')

    print("Lidar's Full Range Image Size: ", len(full_range_image))

    # Compare Distance Sensors to Lidar Ranges

    front_dist = frontDistanceSensor.getValue()

    right_dist = rightDistanceSensor.getValue()

    rear_dist = rearDistanceSensor.getValue()

    left_dist = leftDistanceSensor.getValue()

    print("Distance Sensor vs Lidar")

    print("\tFront:\t", front_dist, "\t|", full_range_image[0])

    print("\tRight:\t", right_dist, "\t|", full_range_image[90])

    print("\tRear:\t", rear_dist, "\t|", full_range_image[180])

    print("\tLeft:\t", left_dist, "\t|", full_range_image[270])
    
    
    ids=1
    for x in comat:
        for i in range(weight):
            x_c=x[0]
            y_c=x[1]
            o=90
            e=1
            w=1/N
            p_list.append(Particles(ids,x_c,y_c,o,e,w))
            ids=ids+1     
            
    for obj in p_list:
        print(obj.id, obj.x, obj.y,obj.orientation, obj.existence, obj.weight)
    
      
    cell_pf=monte_carlo_loc()
    print(cell_pf)
    break
    """
    positions = [0, 0, 0, 0]

    positions = turn(89, positions)

    co_ord = determine_xy(positions)

    cell = determine_grid(co_ord)

    start = [(cell - 1) // 4, (cell - 1) % 4]
    print(cell)
    print(co_ord)
    print(positions)

    dfs(matrix, visited, start, path)
    print(path)
   
    traverse = []
    for i in range(len(path) - 1):
        p2 = path[i]
        p1 = path[i + 1]
        x = p1[0] - p2[0]
        y = p1[1] - p2[1]
        for i in range(abs(x)):
            if (x < 0):
                traverse.append(1.57)
            else:
                traverse.append(4.712)
        for i in range(abs(y)):
            if (y < 0):
                traverse.append(3.14159)
            else:
                traverse.append(0)
    print(traverse, path)
    perform_dfs_motion()
    break
    """