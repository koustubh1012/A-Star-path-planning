# import libraries

import cv2
import numpy as np
import heapq as hq
import math
import time

start_time = time.time()  
canvas = np.ones((500,1200,3))   # creating a frame for video generation
obstacle_set = set()             # set to store the obstacle points
obstacle_list = []               # list to store the obstacle points in order for videp

c2c_node_grid = [[float('inf')] * 500 for _ in range(1200)]       # create a 2D array for storing cost to come
tc_node_grid = [[float('inf')] * 500 for _ in range(1200)]        # create a 2D array for storing cost to come
closed_set = set()               # set to store the value of visited and closed points
closed_list = []                 # list to store the closed nodes
hq.heapify(closed_list)
visited={}
'''
Loop to define the obstacle points in the map
'''
# C = input("Enter the clearance from the obstacle in pixel: ")
# R = input("Enter the radius of the robot in pixel: ")


x_goal = 0
y_goal = 0
x_start = 0
y_start = 0

def visited_node(node):
    visited.update({node[2]:node[4]})
def move_forward(node):
    new_heading = node[5]
    x = node[4][0] + L*np.cos(np.deg2rad(new_heading))
    y = node[4][1] + L*np.sin(np.deg2rad(new_heading))
    x = round(x)
    y = round(y)
    c2c = node[1]+L
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)
    tc = c2c + c2g
    return (x,y),new_heading,tc,c2c


def move_30(node):
    new_heading = (node[5] + 30) % 360
    x = node[4][0] + L*np.cos(np.deg2rad(new_heading))
    y = node[4][1] + L*np.sin(np.deg2rad(new_heading))
    x = round(x)
    y = round(y)
    c2c = node[1]+L
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)
    tc = c2c + c2g
    return (x,y),new_heading,tc,c2c


def move_minus_30(node):
    new_heading = (node[5] - 30) % 360
    x = node[4][0] + L*np.cos(np.deg2rad(new_heading))
    y = node[4][1] + L*np.sin(np.deg2rad(new_heading))
    x = round(x)
    y = round(y)
    c2c = node[1]+L
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)
    tc = c2c + c2g
    return (x,y),new_heading,tc,c2c

def move_60(node):
    new_heading = (node[5] + 60) % 360
    x = node[4][0] + L*np.cos(np.deg2rad(new_heading))
    y = node[4][1] + L*np.sin(np.deg2rad(new_heading))
    x = round(x)
    y = round(y)
    c2c = node[1]+L
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)
    tc = c2c + c2g
    return (x,y),new_heading,tc,c2c

def move_minus_60(node):
    new_heading = (node[5] - 60) % 360
    x = node[4][0] + L*np.cos(np.deg2rad(new_heading))
    y = node[4][1] + L*np.sin(np.deg2rad(new_heading))
    x = round(x)
    y = round(y)
    c2c = node[1]+L
    c2g = math.sqrt((y_goal-y)**2 + (x_goal-x)**2)
    tc = c2c + c2g
    return (x,y),new_heading,tc,c2c

C = 5
R = 5
T = C + R

for y in range(500):
    for x in range(1200):
        canvas[y,x] = [255,255,255]
        if (0<=y<=T):                      # points in the bottom boundary
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif (0<=x<=T):                   # points in the left boundary
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif (500-T<=y<500):                 # points in the top boundary
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif (1200-T<=x<1200):               # points in the right boundary
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif (100-T<=x<=175+T) and (100-T<=y<500):         # points in first rectangle
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif (275-T<=x<=350+T and 0<=y<=400+T):          # points in second rectangle
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        
        # Points int the C-shaped obstacle
        elif (900-T<=x<=1015+T) and (50-T<=y<=125+T):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif (1020-T<=x<1100+T and 50-T<=y<=450+T):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif (900-T<=x<=1015+T and 375-T<=y<=450+T):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1

        # # Points in the hexagon obstacle
        # elif(520-T<=x<=780+T) and (175-T<=y<=325+T):
        #     obstacle_set.add((x,y))
        #     obstacle_list.append((x,y))
        #     node_grid[x][y] = -1
        # elif(325+T<=y<=400+T) and (-y+(325+T)+(75/(130+T))*(x-520+T)>=0) and (-y+(325+T)+(75/(T-130))*(x-780+T)>=0):
        #     obstacle_set.add((x,y))
        #     obstacle_list.append((x,y))
        #     node_grid[x][y] = -1
        # elif(95<=y<=170) and (-75*x-135*y+61575<=0) and (-75*x+135*y+35925>=0):
        #     obstacle_set.add((x,y))
        #     obstacle_list.append((x,y))
        #     node_grid[x][y] = -1

        # Points in the hexagon obstacle
        elif(515<=x<=785) and (170<=y<=330):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif(330<=y<=405) and (-75*x+135*y-5925<=0) and (-75*x-135*y+103425>=0):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1
        elif(95<=y<=170) and (-75*x-135*y+61575<=0) and (-75*x+135*y+35925>=0):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            c2c_node_grid[x][y] = -1
            tc_node_grid[x][y] = -1

valid_start = False

while not valid_start:
        x_start = int(input("Enter the start x coordinate: "))
        y_start = int(input("Enter the start y coordinate: "))
        theta_start = int(input("Enter the initial heading: "))
        if (x_start, y_start) in obstacle_set:
            print("Invalid coordinates, Enter again: ")
        else:
            initial_node = (0, 0, 1, [], (x_start, y_start), theta_start)
            valid_start = True

valid_goal = False

while not valid_goal:
        x_goal = int(input("Enter the goal x coordinate: "))
        y_goal = int(input("Enter the goal y coordinate: "))
        theta_goal = int(input("Enter the goal heading: "))
        if (x_goal, y_goal) in obstacle_set:
            print("Invalid coordinates, Enter again: ")
        else:
            goal = (x_goal, y_goal)
            valid_goal = True

valid_step = False

while not valid_step:
        L = int(input("Enter step size: "))
        if not (1 <= L <= 10):
            print("Invalid Step Size, Enter Again: ")
        else:
            valid_step = True


# move_30(initial_node)
# (x,y),c2c,c2g = move_30(initial_node)
# print((x,y))

new_index = 1         
open_list = []
hq.heappush(open_list,initial_node)        # Push initial node to the list
hq.heapify(open_list)                      # covers list to heapq data type


while(open_list):
    # cost to come, index, parent node index and coordinate values (x,y)
    node = hq.heappop(open_list)       # pop the node with lowest cost to come
    closed_set.add(node[4])            # add the node coordinates to closed set
    hq.heappush(closed_list,node)
    # closed_list.append(node)           # add the node to the closed list
    visited_node(node)
    index = node[2]                    # store the index of the current node
    parent_index = node[3]             # store the parent index list of current node
    
    if node[4] == goal:                # if the node is goal position, exit the loop
        print("Goal reached")
        break

    point, new_heading, tc, c2c = move_30(node)                                     # get the new node's coordinates and cost to come
    if point not in obstacle_set and point not in closed_set:           # check if the new node is in the obstacle set or visited list
        x = point[0]
        y = point[1]
        if tc<tc_node_grid[x][y]:                                         # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()       
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1 
            tc_node_grid[x][y] = tc
            c2c_node_grid[x][y] = c2c                                       # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading)
            hq.heappush(open_list, new_node)                            # push the new node to the open list


    point, new_heading, tc, c2c = move_60(node)                                     # get the new node's coordinates and cost to come
    if point not in obstacle_set and point not in closed_set:           # check if the new node is in the obstacle set or visited list
        x = point[0]
        y = point[1]
        if tc<tc_node_grid[x][y]:                                         # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()       
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1 
            tc_node_grid[x][y] = tc
            c2c_node_grid[x][y] = c2c                                       # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading)
            hq.heappush(open_list, new_node)                            # push the new node to the open list


    point, new_heading, tc, c2c = move_forward(node)                                     # get the new node's coordinates and cost to come
    if point not in obstacle_set and point not in closed_set:           # check if the new node is in the obstacle set or visited list
        x = point[0]
        y = point[1]
        if tc<tc_node_grid[x][y]:                                         # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()       
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1 
            tc_node_grid[x][y] = tc
            c2c_node_grid[x][y] = c2c                                       # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading)
            hq.heappush(open_list, new_node)                            # push the new node to the open list


    point, new_heading, tc, c2c = move_minus_30(node)                                     # get the new node's coordinates and cost to come
    if point not in obstacle_set and point not in closed_set:           # check if the new node is in the obstacle set or visited list
        x = point[0]
        y = point[1]
        if tc<tc_node_grid[x][y]:                                         # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()       
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1 
            tc_node_grid[x][y] = tc
            c2c_node_grid[x][y] = c2c                                       # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading)
            hq.heappush(open_list, new_node)                            # push the new node to the open list


    point, new_heading, tc, c2c = move_minus_60(node)                                     # get the new node's coordinates and cost to come
    if point not in obstacle_set and point not in closed_set:           # check if the new node is in the obstacle set or visited list
        x = point[0]
        y = point[1]
        if tc<tc_node_grid[x][y]:                                         # check if the new cost to come is less than original cost to come
            new_parent_index = parent_index.copy()       
            new_parent_index.append(index)                              # Append the current node's index to the new node's parent index list
            new_index+=1 
            tc_node_grid[x][y] = tc
            c2c_node_grid[x][y] = c2c                                       # Update the new cost to come
            new_node = (tc, c2c, new_index, new_parent_index, point, new_heading)
            hq.heappush(open_list, new_node)                            # push the new node to the open list

print(node)








# Mark the obstacle points in the frame, including points after bloating
for point in obstacle_list:
    canvas[point[1],point[0]] = [255, 0, 0]

# Draw the obstacles in the frame, excluding the points after bloating
cv2.rectangle(canvas, (100, 499), (175, 100), (0 , 0, 255), -1)
cv2.rectangle(canvas, (275, 400), (350, 0), (0 , 0, 255), -1)
cv2.rectangle(canvas, (900, 125), (1100, 50), (0 , 0, 255), -1)
cv2.rectangle(canvas, (900, 450), (1100, 375), (0 , 0, 255), -1)
cv2.rectangle(canvas, (1020, 450), (1100, 50), (0, 0, 255), -1)
pts = np.array([[650, 400], [780, 325], 
                [780, 175], [650, 100], 
                [520, 175], [520, 325]],
                np.int32)
canvas = cv2.fillPoly(canvas, [np.array(pts)], color=(0, 0, 255))
cv2.circle(canvas,(x_start, y_start), 5, (0,0,255),-1)
cv2.circle(canvas,(x_goal, y_goal), 5, (0,0,255), -1)

   
print("Processing Video...")

path = node[3]            # Get the parent node list 
counter = 0               # counter to count the frames to write on video

fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4 format
video_writer = cv2.VideoWriter('output.mp4', fourcc, 60, (1200, 500))

'''
Loop to mark the explored nodes in order on the frame
'''
print("Exploring map")

while closed_list:
    node=hq.heappop(closed_list)
    canvas[node[4][1], node[4][0]] = [0, 255, 0]
    
    counter +=1
    if counter%500 == 0 or counter == 0:
        canvas_flipped = cv2.flip(canvas,0)
        canvas_flipped_uint8 = cv2.convertScaleAbs(canvas_flipped)
        # cv2.imshow('window',canvas_flipped_uint8)
        # cv2.waitKey(1)
        video_writer.write(canvas_flipped_uint8)

'''
Loop to mark the path created
'''
print("Backtracking")
# print(visited)
# for index in path:
#     for node in closed_list:
#         if node[2] == index:
#             canvas[node[4][1], node[4][0]] = [0,0,0]
#             canvas_flipped = cv2.flip(canvas,0)
#             canvas_flipped_uint8 = cv2.convertScaleAbs(canvas_flipped)

for index in path:
    coord=visited[index]
    cv2.circle(canvas, (coord[0],coord[1]), 1, [0,0,0], -1)

    canvas_flipped = cv2.flip(canvas,0)
    canvas_flipped_uint8 = cv2.convertScaleAbs(canvas_flipped)
    # cv2.imshow('window',canvas_flipped_uint8)
    # cv2.waitKey(1)

    video_writer.write(canvas_flipped_uint8)



'''
Loop to add some additional frames at the end of the video
'''
for i in range(150):
    video_writer.write(canvas_flipped_uint8)
    
print("Video Processed")
# cv2.waitKey(0)
video_writer.release()
# cv2.destroyAllWindows()
end_time = time.time()
print(f"The runtime of my program is {end_time - start_time} seconds.")

