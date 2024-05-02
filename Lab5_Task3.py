from controller import Robot

# import numpy as it may be used in future labs

import numpy as np

import math


robot = Robot()

timestep = int(robot.getBasicTimeStep())

frontDistanceSensor = robot.getDevice('front distance sensor')
leftDistanceSensor = robot.getDevice('left distance sensor')
rightDistanceSensor = robot.getDevice('right distance sensor')
rearDistanceSensor = robot.getDevice('rear distance sensor')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)
rearDistanceSensor.enable(timestep)

lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar_horizontal_res = lidar.getHorizontalResolution()
lidar_num_layers = lidar.getNumberOfLayers()
lidar_min_dist = lidar.getMinRange()
lidar_max_dist = lidar.getMaxRange()

print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res,
      "\nLidar Image Number of Layers: ", lidar_num_layers)

print("Lidar Range: [", lidar_min_dist, " ,", lidar_max_dist, '] in meters')


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



leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)



leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')
leftposition_sensor.enable(timestep)
rightposition_sensor.enable(timestep)


imu = robot.getDevice('inertial unit')
imu.enable(timestep)


def rotateandfind(angle, positions):
    while robot.step(timestep) != -1:
        pos = round((imu.getRollPitchYaw()[2] * (180 / math.pi)))
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(-2)
        if (pos == 0):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
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
                positions[0] = pos_dist
            elif (a == 0 and b == 1 and c == 0) and (p >= 40 and p <= 50):
                positions[1] = pos_dist
            elif (a == 0 and b == 0 and c == 1) and (p >= 40 and p <= 50):
                positions[2] = pos_dist
            elif (a == 1 and b == 1 and c == 0) and (p >= 40 and p <= 50):
                positions[3] = pos_dist
        if (angle == pos):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            break
    return positions


            
def get_xy(positions):
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


def get_grid(xy):
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


dir = -1


def adjlist(node):
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


def dfs(matrix, v, curr, travel):
    if (len(v) == 16):
        return True
    if curr not in v:
        v.append(curr)
    travel.append(curr)
    neighbours = adjlist(curr)
    for i in neighbours:
        if (i not in v):
            dfs(matrix, v, i, travel)
            travel.append(curr)

matrix = [[1, 1, 1, 1],
          [1, 1, 1, 1],
          [1, 1, 1, 1],
          [1, 1, 1, 1]]

v = []
travel = []

# perform simulation steps until Webots is stopping the controller
checker = 0
t = -1
motion = -1
rt = -1


def set_velocity(l, r):
    leftMotor.setVelocity(l / 0.8)
    rightMotor.setVelocity(r / 0.8)


# Main loop:
# perform simulation steps until Webots is stopping the controller


def time_to_rotate(v):
    global dir
    global rt
    global t
    if (v < 0):
        dir = 'c'
    if (v > 0):
        dir = 'a'
    rt = (abs(v)) / (4 / 2.28)
    t = 2
    print(rt, t, dir)
    return


def makepath():
    global checker
    global dir
    global rt
    global t
    while (robot.step(timestep) != -1):
        f = frontDistanceSensor.getValue()
        if checker == 0:
            p = imu.getRollPitchYaw()[2]
            if (p < 0):
                p = p + 6.28
            dif =  move[0] - p
            move.pop(0)
            time_to_rotate(dif)
            checker = 1
        if (dir != -1):
            if (rt > 0):
                print(t)
                if (dir == 'a'):
                    set_velocity(-2, 2)
                if (dir == 'c'):
                    set_velocity(2, -2)
                rt = rt - 0.032
                if (rt <= 0):
                    rt = -1
                    continue
                else:
                    continue
            elif (t > 0):
                if(f<0.14):
                    t=-1
                    continue
                set_velocity(5, 5)
                t = t - 0.032
                continue
            else:
                dir = -1
                continue
        if (len( move) == 0):
            set_velocity(0, 0)
            break
        i =  move.pop(0)
        p = imu.getRollPitchYaw()[2]
        if (p < 0):
            p = p + 6.28
        dif = i - p
        time_to_rotate(dif)


while robot.step(timestep) != -1:


    full_range_image = lidar.getRangeImage()

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

    pillars_dist = [0, 0, 0, 0]
    pillars_dist = rotateandfind(89, pillars_dist)
    get_coordinates = get_xy( pillars_dist)
    cell_number = get_grid(get_coordinates)

    begin = [(cell_number - 1) // 4, (cell_number - 1) % 4]
    print(cell_number)
    print(get_coordinates)
    print(pillars_dist)

    dfs(matrix, v, begin, travel)
    print(travel)
   
    move = []
    for i in range(len(travel) - 1):
        second = travel[i]
        first = travel[i + 1]
        m = first[0] - second[0]
        n = first[1] - second[1]
        for i in range(abs(m)):
            if (m < 0):
                move.append(1.5)
            else:
                move.append(4.7)
        for i in range(abs(n)):
            if (n < 0):
                move.append(3.1)
            else:
                move.append(0)
    makepath()
    break

# Enter here exit cleanup code
