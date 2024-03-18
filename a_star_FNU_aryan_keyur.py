# import libraries

import cv2
import numpy as np
import heapq as hq

canvas = np.ones((500,1200,3))   # creating a frame for video generation
obstacle_set = set()             # set to store the obstacle points
obstacle_list = []               # list to store the obstacle points in order for videp

node_grid = [[float('inf')] * 500 for _ in range(1200)]       # create a 2D array for storing cost to come
closed_set = set()               # set to store the value of visited and closed points
closed_list = []                 # list to store the closed nodes


'''
Loop to define the obstacle points oin the map
'''
for y in range(500):
    for x in range(1200):
        canvas[y,x] = [255,255,255]
        if (0<=y<=5):                      # points in the bottom boundary
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif (0<=x<=5):                   # points in the left boundary
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif (495<=y<500):                 # points in the top boundary
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif (1195<=x<1200):               # points in the right boundary
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif (95<=x<=180) and (95<=y<500):         # points in first rectangle
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif (270<=x<=355 and 0<=y<=405):          # points in second rectangle
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        
        # Points int the C-shaped obstacle
        elif (895<=x<=1020) and (45<=y<=130):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif (1015<=x<1105 and 45<=y<=455):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif (895<=x<=1020 and 370<=y<=455):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1

        # Points in the hexagon obstacle
        elif(515<=x<=785) and (170<=y<=330):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif(330<=y<=405) and (-75*x+135*y-5925<=0) and (-75*x-135*y+103425>=0):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1
        elif(95<=y<=170) and (-75*x-135*y+61575<=0) and (-75*x+135*y+35925>=0):
            obstacle_set.add((x,y))
            obstacle_list.append((x,y))
            node_grid[x][y] = -1


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

cv2.imshow("A star",canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()