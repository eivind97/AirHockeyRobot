from globfile import *

def velocity(puck_pos):
    vec = [Vec2d(pos[0], pos[1]) for pos,t in puck_pos] # Gives 5 latest x and y coordinates from puck position input
    t = [t for pos,t in puck_pos] # makes at timestamp for each puck position

    velocityVec = [] # empty table

    if len(vec) > 4: # If more then 4 positions are found
        new_dir = vec[4] - vec[3] # make posible puck direction based on the two last coordinates

        if new_dir.length > 2.0: # if direction is longer then 2 (unit unknown, maybe pixel or mm)
            td = t[4] - t[3] # time difference between the 2 positions
            velocityVec = new_dir / (td.total_seconds()) # calculate velocity and add it to velocityVec tabel
            return velocityVec # return the velocityVec table when method is run and if statement is true

        elif new_dir.length <= 2.0: # if direction is between 1 and 2 mm / pixels
            new_dir = vec[4] - vec[2] # new_dir is changed to be between the last and third last position found by vec
            td = t[4] - t[2] # time difference is changed to the time between last and third last timestamp
            velocityVec = new_dir / (td.total_seconds()) # calculate and add velocity to velocityVec table
            return velocityVec # returns the velocityVec table when method is run and if statement is true
        else:
            return Vec2d.zero() # returns that x and y position is zero
    else:
        return Vec2d.zero() # returns that x and y position is zero

def rotate_vel(velocity): # misleading, means return velocity after a wall is hit
    angle_inn = velocity.angle # calculate the angle based on velocity input
    angle_out = -angle_inn # makes output angle equal input angle

    rotated_velocity = velocity # assume velocity after wall hit equals velocity before wall hit
    if angle_inn < 0:
        rotated_velocity = velocity.rotated(-angle_inn + angle_out) # velocity out is velocity in with respect to angle change

    if angle_inn > 0:
        rotated_velocity = velocity.rotated(-angle_inn + angle_out)

    return rotated_velocity

def path_points2(puck_velocity, puck_position):
    puck_pos = puck_position
    last_velocity = puck_velocity
    totalTime = 0 #time
    t = 0
    points = [] # empty tabel
    times = [] # empty tabel
    calculate = False

    i = 0 # Variable to get out of stuck while loop

    #The system should not calulate puck path if it is moving in
    #positive x-direction
    puck_vel_threshold = -5

    while puck_velocity[0] < puck_vel_threshold and puck_pos[0] > pusher_x_minPos: # pusher_x_minPos = 70 and puck_velocity[0] is start velocity
        i += 1
        totalTime += t
        points.append(puck_pos) # add puck position to end og points table
        times.append(totalTime) # add current total time to times table

        if puck_velocity[0] < 0 and puck_velocity[1] < 0:
            t = (puck_y_minPos - puck_pos[1]) / puck_velocity[1] # time it takes from current position to bottom wall
            px = puck_pos[0] + puck_velocity[0]*t # new x coordinate after puck reach min y coordinate

            puck_pos = [px, puck_y_minPos] # add a new puck position
            last_velocity = puck_velocity
            puck_velocity = rotate_vel(puck_velocity) # puck velocity after wall hit
            calculate = True # system should now calculate puck path

        elif puck_velocity[0] < 0 and puck_velocity[1] > 0:
            t = (puck_y_maxPos - puck_pos[1])/puck_velocity[1]
            px = puck_pos[0] + puck_velocity[0]*t

            puck_pos = [px, puck_y_maxPos]
            last_velocity = puck_velocity
            puck_velocity = rotate_vel(puck_velocity)
            calculate = True

        elif puck_velocity[0] < 0 and puck_velocity[1] == 0:
            calculate = True

        if i > 30:
            calculate = False
            times = []
            break

    if calculate:
        t = (pusher_x_minPos + 25 - points[-1][0]) / last_velocity[0] # time it takes to reach min x position
        totalTime += t # add current time to total time

        py = points[-1][1] + last_velocity[1]*t # find new y coordinate at the time the puck reach x min pos

        # change bellow statement such that left most puck position and robot interception position is the same
        # puck_min = 35 and bot_min = 70 gives incorrect pusher position, especially at puck angles close to 90deg
        puck_pos =[pusher_x_minPos, py] # add new puck position
        points.append(puck_pos) # add position to points table
        times.append(totalTime) # add time to times table

    else:
        points = [puck_position]
        times = [0]

    #print(points)
    return points, times, last_velocity