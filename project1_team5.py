from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait
#import numpy as np
import os
import string

#Define Motors and their port connections
l_Motor = Motor(Port.A,Direction.COUNTERCLOCKWISE)
r_Motor = Motor(Port.B, Direction.CLOCKWISE)

#Define gridsize of the course
GRIDSIZE_LENGTH = 16
GRIDSIZE_HEIGHT = 10

#Define what symbols for blank space, obstacle space, start and goal
BLANK_SYMBOL = '.'
OBSTACLE_SYMBOL = 'X'
START_SYMBOL = 'S'
GOAL_SYMBOL = 'G'

DEBUG = 1

def getcwd(fname):
    cwd = os.path.dirname(__file__)                                                 # get the current working directory
    path = os.path.abspath(os.path.join(cwd,fname))                                 # get the real path of the filename and join with the cwd
    return path                                                                     # return the path

#add obstacles, goal and start coordinates via a file
def read_in_coordinates_from_file(course):
    fname = "coordinate.txt"                                                        
    file = open(getcwd(fname))
    for l in file:
        split_fl = [num.strip() for num in l.split(' ')]
        coordinats = []
        for s in split_fl:
            if(s == 'O'):
                corner1 = corner2 = corner3 = []
                
                if(len(coordinats) < 6):                                                    # if there are less than 3 coordinates it will throw an Exception
                    raise Exception("Too Few Coordinates Given for obstacle")
                
                corner1.append(coordinats[1])                                               
                corner1.append(coordinats[0])
                verify_coordinates(corner1[0],corner1[1])                                   # point 1 of ann obstacle
                corner1.append(coordinats[3])
                corner1.append(coordinats[2])
                verify_coordinates(corner2[0],corner2[1])                                   # point 2 of an obstacle
                corner3.append(coordinats[5])
                corner3.append(coordinats[4])
                verify_coordinates(corner3[0],corner3[1])                                   # point 3 of an obstacle
                create_obstacle(course, corner1, corner2, corner3)                          # add obstacle
            elif(s == 'G'):
                verify_coordinates(coordinats[1], coordinats[0])                            # Goal location addition and verification
                course[coordinats[1]][coordinats[0]] = GOAL_SYMBOL
            elif(s == 'S'):
                verify_coordinates(coordinats[1], coordinats[0])                            # Start location addition and verification
                course[coordinats[1]][coordinats[0]] = START_SYMBOL
            else:
                coordinats.append(int(s,10))                                                #if its not one of thoes 3 charaters or an integer it will throw an error

    file.close()

    if DEBUG:
        print(f"Course Updated from File {fname}\n")
        print_map(course)

#creates 2d array grid map full of Os to represent blank
def create_map():
    course = [[BLANK_SYMBOL for x in range(GRIDSIZE_LENGTH)] for x in range (GRIDSIZE_HEIGHT)]
    #course = np.full((GRIDSIZE_LENGTH, GRIDSIZE_HEIGHT), BLANK_SYMBOL)
    if DEBUG:
        print_map(course)
    return course

#verifies given coordinate is in the course area or else it will through an exception
def verify_coordinates(length, height):
    if length > GRIDSIZE_LENGTH or height > GRIDSIZE_HEIGHT:
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
            course[row][col] = OBSTACLE_SYMBOL
        
    if DEBUG:
        print("Obstacle added, new map is")
        print_map(course)

def main():
    try:
        course = create_map()
        #add_obstacles(course)
        read_in_coordinates_from_file(course) #testing
    except Exception as e:
        print(f"Error: {e}")

main()