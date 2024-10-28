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
            elif(s == 'S'):
                verify_coordinates(coordinates[1], coordinates[0])                            # Start location addition and verification
                course[coordinates[1]][coordinates[0]] = START_SYMBOL
            else:
                coordinates.append(int(s,10))                                                #if its not one of thoes 3 charaters or an integer it will throw an error

    file.close()

    if DEBUG:
        print(f"Course Updated from File {fname}\n")
        print_map(course)

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

#uses a while loop to add all obstacles to the course
def add_obstacles(course):
    print("Adding Obstacles")
    #place start and goal symbols
    user_inp = input("Enter Start coordinate (Ex: 2 6): ").split()
    verify_coordinates(int(user_inp[1]),int(user_inp[0]))
    course[int(user_inp[1])][int(user_inp[0])] = START_SYMBOL
    user_inp = input("Enter Goal coordinate (Ex: 2 6): ").split()
    verify_coordinates(int(user_inp[1]),int(user_inp[0]))
    course[int(user_inp[1])][int(user_inp[0])] = GOAL_SYMBOL
    
    if DEBUG:
        print("Start and Goal added")
        print_map(course)
    
    corner1 = corner2 = corner3 = 0
    while (user_inp != 'n' and user_inp != 'N'):
        #gets 4 corners, numbers stored as an array. 2 6 is stored ['2', '6']
        #since all shapes are rectangles, we only need 3 corners
        corner1 = input("Enter top left corner coordinate (Ex: 2 6): ").split()
        verify_coordinates(int(corner1[1]),int(corner1[0]))
        corner2 = input("Enter top right corner coordinate (Ex: 2 6): ").split()
        verify_coordinates(int(corner2[1]),int(corner2[0]))
        corner3 = input("Enter bottom left corner coordinate (Ex: 2 6): ").split()
        verify_coordinates(int(corner3[1]),int(corner3[0]))
        
        create_obstacle(course, corner1, corner2, corner3)
        user_inp = input("Do you want to add another obstacle? Type n to end. ")

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

def brushfire(course):
    #triple nested for loop yayyyy, whats time complexity anyway
    #1st loop will iterate waves, fill in numbers 1-Length
    #we use legnth to ensure the wave travels the entire course, should never need to use all 16, but could need up to it
    #2nd and 3rd loops are the 2d array
    for wave in range (1, GRIDSIZE_LENGTH):
        for row in range(GRIDSIZE_HEIGHT):
            for col in range(GRIDSIZE_LENGTH):
                
                #if we find an obstacle or the prev wave number, we want to mark every square around it with the next number
                if (course[row][col] == OBSTACLE_SYMBOL or course[row][col] == str(wave - 1)):
                    
                    #place above if not on first row; dont replace obstacle symbols
                    if (row > 0):
                        if course[row-1][col] == str(wave):
                           pass#course[row-1][col] = PATH_SYMBOL
                        elif course[row-1][col] == BLANK_SYMBOL:
                            course[row-1][col] = str(wave)
                        
                        #if this is the top right part of the obstacle, then add a top right corner
                        if (col < (GRIDSIZE_LENGTH-1)) and ((course[row][col+1] == BLANK_SYMBOL) or (course[row][col+1] == str(wave))):
                            course[row-1][col+1] = str(wave)
                        
                        #if this is the top left part of obstacle, then add a top left corner
                        if (col > 0) and ((course[row][col-1] == BLANK_SYMBOL) or (course[row][col-1] == str(wave))):
                            course[row-1][col-1] = str(wave)
                            
                    #place to the left if not on first column
                    if (col > 0):
                        if course[row][col-1] == str(wave):
                           pass#course[row][col-1] = PATH_SYMBOL
                        elif course[row][col-1] == BLANK_SYMBOL:
                            course[row][col-1] = str(wave)
                    
                    #place below if not on last row
                    if (row < (GRIDSIZE_HEIGHT-1)):
                        if course[row+1][col] == str(wave):
                           pass#course[row+1][col] = PATH_SYMBOL
                        elif course[row+1][col] == BLANK_SYMBOL:
                            course[row+1][col] = str(wave)
                        
                        #if this is the bottom right part of the obstacle, then add a bottom right corner 
                        if (col < (GRIDSIZE_LENGTH-1)) and ((course[row][col+1] == BLANK_SYMBOL) or (course[row][col+1] == str(wave))):
                            course[row+1][col+1] = str(wave)
                            
                        #if this is the bottom left part of obstacle, then add a bottom left corner
                        if (col > 0) and ((course[row][col-1] == BLANK_SYMBOL) or (course[row][col-1] == str(wave))):
                            course[row+1][col-1] = str(wave)
                    
                    #place to the right if not on last column
                    if (col < (GRIDSIZE_LENGTH-1)):
                        if course[row][col+1] == str(wave):
                            pass#course[row][col+1] = PATH_SYMBOL
                        elif course[row][col+1] == BLANK_SYMBOL:
                            course[row][col+1] = str(wave)
        
        if DEBUG:
            print(f"Wave {wave} now completed, map is")
            print_map(course)        
       
    
    if DEBUG:
        print("\nbrushfire done, course is now")
        print_map(course)


def goalfire(course):
    xGoal = 0
    yGoal = 0
    dist = 0
    queue = []
    
    for row in range(GRIDSIZE_HEIGHT):
        for col in range(GRIDSIZE_LENGTH):
            if course[row][col] == GOAL_SYMBOL:
                xGoal = col
                yGoal = row
                break
    
    queue.append((xGoal, yGoal, 0))
    while (queue):
        col, row, dist = queue.pop(0)
        if ((row > 0 and row < GRIDSIZE_HEIGHT) and (col > 0 and col < GRIDSIZE_LENGTH)):
            if course[row][col] == BLANK_SYMBOL or course[row][col] == GOAL_SYMBOL:
                course[row][col] = str(dist)
                queue.append((col-1, row, dist+1))
                queue.append((col+1, row, dist+1))
                queue.append((col, row+1, dist+1))
                queue.append((col, row-1, dist+1))
                
    if DEBUG:
        print("Q is now empty, map is ")
        print_map(course)
    

def main():
    try:
        course = create_map()
        #add_obstacles(course)
        read_in_coordinates_from_file(course) #testing
    except Exception as e:
        print(f"Error: {e}")
    
    #Course created, convert from 1ft squares to 6in squares
    #course = expand_map(course)
    
    #Course expanded, now apply brushfire to paint a path
    #course = brushfire(course) #couldnt get to fully work
    
    #next attempt to path course
    course = goalfire(course)

main()