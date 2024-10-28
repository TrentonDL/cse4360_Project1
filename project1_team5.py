from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait
#import numpy as np
import os
import string
import dead_reckoning

hub = PrimeHub(top_side=Axis.Z, front_side=Axis.X)

#Define Motors and their port connections
l_Motor = Motor(Port.A,Direction.COUNTERCLOCKWISE)
r_Motor = Motor(Port.B, Direction.CLOCKWISE)

#Define gridsize of the course
GRIDSIZE_LENGTH = 16
GRIDSIZE_HEIGHT = 10
#Expanded gridsize dimensions
EXP_GRIDSIZE_LENGTH = GRIDSIZE_LENGTH * 2
EXP_GRIDSIZE_HEIGHT = GRIDSIZE_HEIGHT * 2

#Define what symbols for blank space, obstacle space, start and goal
BLANK_SYMBOL = '.'
OBSTACLE_SYMBOL = 'X'
START_SYMBOL = 'S'
GOAL_SYMBOL = 'G'
PATH_SYMBOL = "P"

DEBUG = 1

def getcwd(fname):
    cwd = os.path.dirname(__file__)                                                 # get the current working directory
    path = os.path.abspath(os.path.join(cwd,fname))                                 # get the real path of the filename and join with the cwd
    return path                                                                     # return the path

#add obstacles, goal and start coordinates via a file
def read_in_coordinates_from_file(course):
    xStart = 0
    yStart = 0
    xGoal = 0
    yGoal = 0
    fname = "coordinate.txt"                                                        
    file = open(getcwd(fname))
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
                verify_coordinates(corner1[0],corner1[1])                                   # point 1 of ann obstacle
                corner2.append(coordinates[2])
                corner2.append(coordinates[3])
                verify_coordinates(corner2[0],corner2[1])                                   # point 2 of an obstacle
                corner3.append(coordinates[4])
                corner3.append(coordinates[5])
                verify_coordinates(corner3[0],corner3[1])                                 # point 3 of an obstacle
                create_obstacle(course, corner1, corner2, corner3)                          # add obstacle
                obs_num += 1
            elif(s == 'G'):
                verify_coordinates(coordinates[1], coordinates[0])                            # Goal location addition and verification
                course[coordinates[1]][coordinates[0]] = GOAL_SYMBOL
                xGoal = coordinates[0]
                yGoal = coordinates[1]
            elif(s == 'S'):
                verify_coordinates(coordinates[1], coordinates[0])                            # Start location addition and verification
                course[coordinates[1]][coordinates[0]] = START_SYMBOL
                xStart = coordinates[0]
                yStart = coordinates[1]
            else:
                coordinates.append(int(s,10))                                                #if its not one of thoes 3 charaters or an integer it will throw an error

    file.close()

    if DEBUG:
        print(f"Course Updated from File {fname}\n")
        print_map(course)
    
    return (xStart, yStart, xGoal, yGoal)

#creates 2d array grid map full of Os to represent blank
def create_map():
    course = [[BLANK_SYMBOL for x in range(GRIDSIZE_LENGTH)] for y in range (GRIDSIZE_HEIGHT)]
    #course = np.full((GRIDSIZE_LENGTH, GRIDSIZE_HEIGHT), BLANK_SYMBOL)
    if DEBUG:
        print_map(course)
    return course

#verifies given coordinate is in the course area or else it will through an exception
def verify_coordinates(height, length):
    if length >= GRIDSIZE_LENGTH or height >= GRIDSIZE_HEIGHT:
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

def expand_map(course):
    expCourse = [[BLANK_SYMBOL for x in range(GRIDSIZE_LENGTH*2)] for y in range(GRIDSIZE_HEIGHT*2)]
    
    #for every symbol in original course, create a copy in the x+1, y+1, and x+1 y+1 directions 
    for row in range(GRIDSIZE_HEIGHT):
        for col in range(GRIDSIZE_LENGTH):
            symbol = course[row][col]
            expCourse[2*row][2*col] = symbol
            expCourse[2*row+1][2*col] = symbol
            expCourse[2*row][2*col+1] = symbol
            expCourse[2*row+1][2*col+1] = symbol
            
    if DEBUG:
        print("\nCourse expanded, expCourse is ")
        print_map(expCourse)
        
    return expCourse

def goal_fire(course):
    xGoal = 0
    yGoal = 0
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

def find_path(course, xStart, yStart, xGoal, yGoal):
    
    #current coordinate
    xCurr = xStart
    yCurr = yStart
    curDist = course[xStart][yStart]
    path = []
    path.append((xCurr,yCurr))
    
    while (course[xCurr][yCurr] != '0'):
        if (course[xCurr][yCurr+1] < curDist):
            yCurr += 1
        elif (course[xCurr+1][yCurr] < curDist):
            xCurr += 1
        elif (course[xCurr-1][yCurr] < curDist):
            xCurr -= 1
        elif (course[xCurr][yCurr-1] < curDist):
            yCurr -= 1
        
        curDist = course[xCurr][yCurr]
        path.append((xCurr,yCurr))
    
    if DEBUG:
        print("Path array finished, current path is ")
        print(path)
    
    

def main():
    try:
        course = create_map()
        #add_obstacles(course)
        xStart, yStart, xGoal, yGoal = read_in_coordinates_from_file(course)
    except Exception as e:
        print(f"Error: {e}")
    
    #Course created, convert from 1ft squares to 6in squares
    #course = expand_map(course)
    
    #Course expanded, now apply brushfire to paint a path
    #course = brushfire(course) #couldnt get to fully work
    
    #expand obstacles for padding
    course = expand_obstacles(course)
    
    #next attempt to path course
    course = goal_fire(course, xGoal, yGoal)
    
    #path = find_path(course, xStart, yStart, xGoal, yGoal)
    dead_reckoning.move_to_goal(path)

main()