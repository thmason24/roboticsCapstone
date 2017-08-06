#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import heapq
import yaml


def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    #print(occupancy_map)
    #create adjacency list
    y_len = occupancy_map.shape[0]
    x_len = occupancy_map.shape[1]
    #adjMap = [[[]]*y_len for x in range(x_len)]
    print(occupancy_map)
    
    #generate adjacency list
    X_list = []
    Y_list = []
    i_list = []
    j_list = []
    adj = []
    occupancy_list = []
    for (i,j), obstacle in np.ndenumerate(occupancy_map):
        print(str(i) + ' ' + str(j)) # + ' ' + str(value))
        x = (j+0.5)*x_spacing
        y = (i+0.5)*y_spacing
        #make adjacency list for this node
        adjEntry = []
        if not obstacle:
            #print(x)
            #print(y)
            if j > 0:
                if not occupancy_map[i,j-1]:
                    adjEntry.append([(j-1)*y_len+i,x_spacing])
            if j < x_len:
                if not occupancy_map[i,j+1]:
                    adjEntry.append([(j+1)*y_len+i,x_spacing])
            if i > 0:
                if not occupancy_map[i-1,j]:
                    adjEntry.append([j*y_len+i-1,x_spacing])
            if i < y_len:
                if not occupancy_map[i+1,j]:
                    adjEntry.append([j*y_len+i+1,y_spacing])
            #print(adjEntry)
        X_list.append(x)
        Y_list.append(y)
        i_list.append(i)
        j_list.append(j)
        occupancy_list.append(occupancy_map[i,j])
        adj.append(adjEntry)
    
    #execute dykstra
    #initialize to xlen + ylen * num grid points
    maxDist = (x_spacing+y_spacing)*len(adj)
    dist = [maxDist] * len(adj)

    print()
    print(x_spacing)
    print(y_spacing)
    print(start)
    print(goal)
    startInd = int((start[0]/y_spacing)-0.5)*x_len+int((start[1]/x_spacing)-0.5)
    goalInd  = int((goal[0]/y_spacing)-0.5)*x_len+int((goal[1]/x_spacing)-0.5)

    print()
    print(startInd)
    print(goalInd)
    print()
    dist[startInd] = 0
    prev = [-1]*len(adj)
    h = list(zip(dist,range(len(dist))))
    heapq.heapify(h)
    while len(h) > 0:
        u = heapq.heappop(h)
        for ind,i in enumerate(adj[u[1]]):
            if dist[i[0]] > u[0] + i[1]:
                dist[i[0]] = u[0] + i[1]
                prev[i[0]] = u[1]
                heapq.heappush(h,(dist[i[0]],i[0]))
    if dist[goalInd] == maxDist:
        return -1
    else:
    	#create path
    	print('distance = ' + str(dist[goalInd]))
    	path = []
    	curInd = goalInd
    	while curInd != startInd:
    		path.append(curInd)
    		curInd = prev[curInd]
    	xPath = [j_list[i] for i in path]
    	yPath = [i_list[i] for i in path]

    	plt.figure(1)
    	borderX = [0, x_len-1, 0, x_len-1]
    	borderY = [0, 0,y_len-1,  y_len-1]
    	obstaclesI = [i_list[i] for i in range(len(i_list)) if occupancy_list[i] == 1]
    	obstaclesJ = [j_list[i] for i in range(len(i_list)) if occupancy_list[i] == 1]

    	plt.scatter(borderX,borderY,color = 'black',marker = 's',s=100)
    	plt.scatter(obstaclesJ,obstaclesI,color = 'black')

    	plt.scatter(j_list[startInd],i_list[startInd],color = 'red', marker = 's', s=100)
    	plt.scatter(j_list[goalInd],i_list[goalInd],color = 'green', marker = 's', s=100)

    	plt.scatter(xPath,yPath)
    	return path
    

def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



def main():
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    main()

