from uio import StringIO
from coordinate import TEXT
from dead_reckoning import move_to_goal

#I mixed up my x and y coordinates in some places in the earlier functions, which is why in some of the later function I have to swap them -Dylan

#We used 3 functions written by chatgpt: print_map, copy_2d_array, and copy_1d_array
#These functions only assist in visualizing the 2D array for debugging puproses and do not affect the path planning in any way

#Define gridsize of the course
GRIDSIZE_LENGTH = 16
GRIDSIZE_HEIGHT = 10

#Define what symbols for blank space, obstacle space, start and goal
BLANK_SYMBOL = '.'
OBSTACLE_SYMBOL = 'X'
START_SYMBOL = 'S'
GOAL_SYMBOL = 'G'
PATH_SYMBOL = "P"

DEBUG = 0

#add obstacles, goal and start coordinates via a file
def read_in_coordinates_from_file(course):
    xStart = 0
    yStart = 0
    xGoal = 0
    yGoal = 0
    file = StringIO(TEXT)
    obs_num = 1 #keeps track of how many obstacles we add
    for l in file:
        split_fl = [num.strip() for num in l.split(' ')]
        coordinates = []
        for s in split_fl:
            if(s == 'O'):
                corner1 = []
                corner2 = []
                corner3 = []
                
                if(len(coordinates) < 6):                                                    # if there are less than 3 coordinates it will throw an Exception
                    raise Exception("Too Few Coordinates Given for obstacle")
                
                corner1.append(coordinates[0])                                               
                corner1.append(coordinates[1])
                #verify_coordinates(corner1[0],corner1[1])                                   # point 1 of ann obstacle
                corner2.append(coordinates[2])
                corner2.append(coordinates[3])
                #verify_coordinates(corner2[0],corner2[1])                                   # point 2 of an obstacle
                corner3.append(coordinates[4])
                corner3.append(coordinates[5])
                #verify_coordinates(corner3[0],corner3[1])                                 # point 3 of an obstacle
                create_obstacle(course, corner1, corner2, corner3)                          # add obstacle
                obs_num += 1
            elif(s == 'G'):
                #verify_coordinates(coordinates[1], coordinates[0])                            # Goal location addition and verification
                course[coordinates[1]][coordinates[0]] = GOAL_SYMBOL
                xGoal = coordinates[0]
                yGoal = coordinates[1]
            elif(s == 'S'):
                #verify_coordinates(coordinates[1], coordinates[0])                            # Start location addition and verification
                course[coordinates[1]][coordinates[0]] = START_SYMBOL
                xStart = coordinates[0]
                yStart = coordinates[1]
            else:
                coordinates.append(int(s,10))                                                #if its not one of thoes 3 charaters or an integer it will throw an error
    
    if DEBUG:
        print(f"Course Updated from File\n")
        print_map(course)
    
    return (xStart, yStart, xGoal, yGoal)

#creates 2d array grid map full of Os to represent blank
def create_map():
    course = [[BLANK_SYMBOL for x in range(GRIDSIZE_LENGTH)] for y in range (GRIDSIZE_HEIGHT)]
    if DEBUG:
        print_map(course)
    return course

#verifies given coordinate is in the course area or else it will through an exception
def verify_coordinates(height, length):
    if length >= GRIDSIZE_HEIGHT or height >= GRIDSIZE_LENGTH:
        raise Exception(f"Coordinate ({length} {height}) is invaild!")

#chatgpt code to easily print 2d array
def print_map(course):
    for row in course:
        print(' '.join(row))

#takes input of 3 corners and creates the obstacle on the course
def create_obstacle(course, corner1, corner2, corner3):
    for row in range(int(corner1[1]), int(corner3[1]) + 1):
        for col in range(int(corner1[0]), int(corner2[0]) + 1):
            course[row][col] = (OBSTACLE_SYMBOL)
        
    if DEBUG:
        print("Obstacle added, new map is")
        print_map(course)

#utilizes a method similar to brushfire
#It marks the goal coordinate with a 0, then adds all 4 coordinates around the point to the queue with 1 higher distance
#if its a valid square, it places the distance number on that square that will represent the distance from that square to the goal
#it will go until the entire grid is filled with their distance to the goal
def goal_fire(course, xGoal, yGoal):
    dist = 0
    queue = []
    
    queue.append((xGoal, yGoal, dist))
    while (queue):
        col, row, dist = queue.pop(0)
        if ((row >= 0 and row < GRIDSIZE_HEIGHT) and (col >= 0 and col < GRIDSIZE_LENGTH)):
            if course[row][col] == BLANK_SYMBOL or course[row][col] == GOAL_SYMBOL or course[row][col] == START_SYMBOL:
                course[row][col] = str(dist)
                queue.append((col-1, row, dist+1))
                queue.append((col+1, row, dist+1))
                queue.append((col, row+1, dist+1))
                queue.append((col, row-1, dist+1))
                
    if DEBUG:
        print("Q is now empty, map is ")
        print_map(course)

    return course

#This function grows all obstacles by 1ft in all directions. Ex: a 1x1 obstacle would turn into a 3x3 obstacle
#Using this function before calculating the path ensures that there will be more space between the obstacles and the robot. This gives the robot more clearance in its path
#It was not used in the final demo as we were confident in the accuracy of the robots pathing
def expand_obstacles(course):
    queue = []
    
    for row in range(GRIDSIZE_HEIGHT):
        for col in range(GRIDSIZE_LENGTH):
            if course[row][col] == OBSTACLE_SYMBOL:
                queue.append((row,col))
                
    while (queue):
        row, col = queue.pop(0)
        
        #expand obstacles left, and if possible diagonally up left and down left
        if row > 0:
            course[row-1][col] = OBSTACLE_SYMBOL
            if col > 0:
                course[row-1][col-1] = OBSTACLE_SYMBOL
            if col < GRIDSIZE_LENGTH:
                course[row-1][col+1] = OBSTACLE_SYMBOL
        
        #expand obstacles right, and if possible diagonally up right and down right
        if row < GRIDSIZE_HEIGHT:
            course[row+1][col] = OBSTACLE_SYMBOL
            if col > 0:
                course[row+1][col-1] = OBSTACLE_SYMBOL
            if col < GRIDSIZE_LENGTH:
                course[row+1][col+1] = OBSTACLE_SYMBOL
        
        #expand obstacles up
        if col > 0:
            course[row][col-1] = OBSTACLE_SYMBOL
        
        #expand obstacles down
        if col < GRIDSIZE_HEIGHT:
            course[row][col+1] = OBSTACLE_SYMBOL
    
    if DEBUG:
        print("Obstacles expanded, map is now")    
        print_map(course)
        
    return course

#this function takes the path from goal_fire and the starting coordinates
#it sees what the number is at the starting square, then continues looking for the next lowest number until it finally reaches the goal with a value of 0
#it stores these coordinates in an array called Path
def find_path(course, xStart, yStart, xGoal, yGoal):
    #I had to fix the x and y coordinates here which is why I am assigning x to y and y to x at the start here
    #current x and y coordinate
    xCurr = yStart
    yCurr = xStart
    curDist = int(course[xCurr][yCurr])
    path = []
    path.append((yCurr,xCurr))
    
    while (course[xCurr][yCurr] != '0'):
        if (course[xCurr][yCurr+1] != OBSTACLE_SYMBOL and int(course[xCurr][yCurr+1]) < curDist):
            yCurr += 1
        elif (course[xCurr+1][yCurr] != OBSTACLE_SYMBOL and int(course[xCurr+1][yCurr]) < curDist):
            xCurr += 1
        elif (course[xCurr-1][yCurr] != OBSTACLE_SYMBOL and int(course[xCurr-1][yCurr]) < curDist):
            xCurr -= 1
        elif (course[xCurr][yCurr-1] != OBSTACLE_SYMBOL and int(course[xCurr][yCurr-1]) < curDist):
            yCurr -= 1
        
        curDist = int(course[xCurr][yCurr])
        path.append((yCurr,xCurr))
    
    if DEBUG:
        print("Path array finished, current path is ")
        print(path)
    
    return path

#this takes the 'clean_course' which is a copy of the course when it only had start, goal, and obstacles
#It also takes path, and then overlays the coordinates with the Path symbol onto the clean map showing the robots start, goal, obstacles, and path it will take to get there
def overlay_path(course, path, xGoal, yGoal ):
    
    y, x = path.pop(0)
    course[x][y] = START_SYMBOL
    
    while (path):
        y, x = path.pop(0)
        course[x][y] = PATH_SYMBOL
    
    course[yGoal][xGoal] = GOAL_SYMBOL
    
    if DEBUG:
        print("The clean path is ")
        print_map(course)
    
    return course
        
#chatgpt code to copy a 2darray because we cant use copy or deepcopy
def copy_2d_array(original):
    # Get the number of rows and columns
    rows = len(original)
    cols = len(original[0]) if rows > 0 else 0

    # Create a new 2D array with the same dimensions
    copied_array = [[0] * cols for _ in range(rows)]

    # Copy each element from the original array to the new array
    for i in range(rows):
        for j in range(cols):
            copied_array[i][j] = original[i][j]

    return copied_array

#chatgpt code to copy a 1darray because we cant use copy
def copy_1d_array(original):
    # Create a new array with the same length
    copied_array = [0] * len(original)
    
    # Copy each element from the original array to the new array
    for i in range(len(original)):
        copied_array[i] = original[i]
    
    return copied_array
    
def main(): 
    try:
        #create base course
        course = create_map()
        
        #add_obstacles(course)
        xStart, yStart, xGoal, yGoal = read_in_coordinates_from_file(course)

        #create a copy of the course with only start, goal, and obstacles marked. Will use later with overlay_path
        empty_course = copy_2d_array(course)
        
        #Add numbers to all grid cells representing their distance to goal
        course = goal_fire(course, xGoal, yGoal)
        
        #calculate path coordinates from start to goal
        path = find_path(course, xStart, yStart, xGoal, yGoal)
        
        #print a clean path from start to goal for visual purposes
        clean_course = overlay_path(empty_course, copy_1d_array(path), xGoal, yGoal)
        print_map(clean_course)

        #move robot along given path
        move_to_goal(path)    
    except Exception as e:
        print(f"Error: {e}")
    
main()
