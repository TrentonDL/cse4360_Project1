from pybricks.pupdevices import Motor
from pybricks.parameters import Port,Direction
from pybricks.tools import wait
#import numpy as np

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

#creates 2d array grid map full of Os to represent blank
def create_map():
    course = [[BLANK_SYMBOL for x in range(GRIDSIZE_LENGTH)] for x in range (GRIDSIZE_HEIGHT)]
    #course = np.full((GRIDSIZE_LENGTH, GRIDSIZE_HEIGHT), BLANK_SYMBOL)
    if DEBUG:
        print_map(course)
    return course

#chatgpt code to easily print 2d array
def print_map(course):
    for row in course:
        print(' '.join(row))

#uses a while loop to add all obstacles to the course
def add_obstacles(course):
    print("Adding Obstacles")
    #place start and goal symbols
    user_inp = input("Enter Start coordinate (Ex: 2 6): ").split()
    course[int(user_inp[1])][int(user_inp[0])] = START_SYMBOL
    user_inp = input("Enter Goal coordinate (Ex: 2 6): ").split()
    course[int(user_inp[1])][int(user_inp[0])] = GOAL_SYMBOL
    
    if DEBUG:
        print("Start and Goal added")
        print_map(course)
    
    corner1 = corner2 = corner3 = 0
    while (user_inp != 'n' and user_inp != 'N'):
        #gets 4 corners, numbers stored as an array. 2 6 is stored ['2', '6']
        #since all shapes are rectangles, we only need 3 corners
        corner1 = input("Enter top left corner coordinate (Ex: 2 6): ").split()
        corner2 = input("Enter top right corner coordinate (Ex: 2 6): ").split()
        corner3 = input("Enter bottom left corner coordinate (Ex: 2 6): ").split()
        
        
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
    course = create_map()
    add_obstacles(course)

main()