import queue
from math import inf, sqrt
from heapq import heappop, heappush

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """

    path = []
    boxes = []
    
    #list of all boxes
    srcBoxes = mesh["boxes"]

    #iterate through boxes and find which box source_point and destination_point are contained in
    for box in srcBoxes:
        if (source_point[0] >= box[0] and source_point[0] <= box[1] and source_point[1] >= box[2] and source_point[1] <= box[3]):
            box1 = box
        if (destination_point[0] >= box[0] and destination_point[0] <= box[1] and destination_point[1] >= box[2] and destination_point[1] <= box[3]):
            box2 = box
    #breadth first search from box1 (source_point box) to box2 (destination_point box)
    #segments = breadth_first_search(box1, box2, mesh, source_point, destination_point, boxes)
    segments = aStar(box1, box2, mesh, source_point, destination_point, boxes)
    segments = dijkstras_shortest_path(box1, box2, mesh, source_point, destination_point, boxes)
    #segments = bdaStar(box1, box2, mesh, source_point, destination_point, boxes)

    #add source_point to path then iterate through dictionary and retrieve detail points

    path.append(source_point)
    for seg in segments:
        path.append(segments[seg])
    return path, boxes

def bdaStar(box1, box2, mesh, initial_position, destination, boxes):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    pathsForward = {box1: None}          # maps cells to previous cells on path
    pathsBackward = {box2: None}
    pathcostsForward= {box1: 0}       # maps cells to their pathcosts (found so far)
    pathcostsBackward= {box2: 0}  
    queue = []
    heappush(queue, (0, box1, 'destination'))  # maintain a priority queue of cells
    heappush(queue, (0, box2, 'beginning')) # maintain a priority queue of cells
    print("START:", box1, "END: ", box2)
    found_path = []
    
    while queue:
        priority, cell, curr_goal = heappop(queue)
        boxes.append(cell)

        if curr_goal == 'destination':
            if cell in pathsBackward:
                #print("FORWARD")
                #print("CELL: ", cell)
                pathCell = cell
                found_path.insert(0, cell)
                while(pathsForward[cell] != None):
                    found_path.insert(0, pathsForward[cell])
                    cell = pathsForward[cell]
                cell = pathCell
                while(pathsBackward[cell] != None):
                    found_path.append(pathsBackward[cell])
                    cell = pathsBackward[cell]
                #print(found_path)
                return detail_points(found_path, initial_position, destination)
            else:
                for child in mesh["adj"].get(cell):
                # calculate cost along this path to child
                    if cell == box1:
                        cost_to_child = priority + midPointDistance(cell, child) # transition cost will be distance cost from one box to neighbor 
                        if child not in pathcostsForward or cost_to_child < pathcostsForward[child]:
                            pathcostsForward[child] = cost_to_child            # update the cost
                            pathsForward[child] = cell                         # set the backpointer
                            heappush(queue, (cost_to_child + midPointDistance(child, box2), child, "destination"))     # put the child on the priority queue
                    else:
                        cost_to_child = priority + midPointDistance(cell, child) - midPointDistance(cell, box2) # transition cost will be distance cost from one box to neighbor 
                        if child not in pathcostsForward or cost_to_child < pathcostsForward[child]:
                            pathcostsForward[child] = cost_to_child            # update the cost
                            pathsForward[child] = cell                         # set the backpointer
                            heappush(queue, (cost_to_child + midPointDistance(child, box2), child, "destination"))     # put the child on the priority queue
        else: 
            if  cell in pathsForward:
                #print("BACKWARD")
                #print("CELL: ", cell)
                pathCell = cell
                found_path.insert(0, cell)
                while(pathsBackward[cell] != None):
                    print("CELL:" , cell, "PREVIOUS: ", pathsBackward[cell])
                    found_path.append(pathsBackward[cell])
                    cell = pathsBackward[cell]
                cell = pathCell
                while(pathsForward[cell] != None):
                    #print("FORWARD")
                    #print("CELL:" , cell, "PREVIOUS: ", pathsForward[cell])
                    found_path.insert(0, pathsForward[cell])
                    cell = pathsForward[cell]
                #print(found_path)
                return detail_points(found_path, initial_position, destination)
            else:
                for child in mesh["adj"].get(cell):
                # calculate cost along this path to child
                    if cell == box2:
                        cost_to_child = priority + midPointDistance(cell, child) # transition cost will be distance cost from one box to neighbor 
                        if child not in pathcostsBackward or cost_to_child < pathcostsBackward[child]:
                            pathcostsBackward[child] = cost_to_child            # update the cost
                            pathsBackward[child] = cell                         # set the backpointer
                            heappush(queue, (cost_to_child + midPointDistance(child, box1), child, "beginning"))     # put the child on the priority queue
                    else:
                        cost_to_child = priority + midPointDistance(cell, child) - midPointDistance(cell, box1) # transition cost will be distance cost from one box to neighbor 
                        if child not in pathcostsBackward or cost_to_child < pathcostsBackward[child]:
                            pathcostsBackward[child] = cost_to_child            # update the cost
                            pathsBackward[child] = cell                         # set the backpointer
                            heappush(queue, (cost_to_child + midPointDistance(child, box1), child, "beginning"))     # put the child on the priority queue
                    
            
        # investigate children
    print("No path!")
    return []

def aStar(box1, box2, mesh, initial_position, destination, boxes):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    paths = {box1: None}          # maps cells to previous cells on path
    pathcosts = {box1: 0}       # maps cells to their pathcosts (found so far)
    queue = []
    heappush(queue, (midPointDistance(box1, box2), box1))  # maintain a priority queue of cells
    found_path = []
    
    while queue:
        priority, cell = heappop(queue)
        boxes.append(cell)

        if cell == box2:
            found_path.insert(0, cell)
            while(paths[cell] != None):
                found_path.insert(0, paths[cell])
                cell = paths[cell]
            #print(found_path)
            return detail_points(found_path, initial_position, destination)
        
        # investigate children
        for child in mesh["adj"].get(cell):
            # calculate cost along this path to child
            cost_to_child = priority + midPointDistance(cell, child) - midPointDistance(cell, box2) # transition cost will be distance cost from one box to neighbor 
            #print("cost2child:", cost_to_child ,  "CHILD: " , child , "CELL: " , cell)
            if child not in pathcosts or cost_to_child < pathcosts[child]:
                pathcosts[child] = cost_to_child            # update the cost
                paths[child] = cell                         # set the backpointer
                heappush(queue, (cost_to_child + midPointDistance(child, box2), child))     # put the child on the priority queue
    #print("No path!")
    return []

def dijkstras_shortest_path(box1, box2, mesh, initial_position, destination, boxes):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    paths = {box1: None}          # maps cells to previous cells on path
    pathcosts = {box1: 0}       # maps cells to their pathcosts (found so far)
    queue = []
    heappush(queue, (0, box1))  # maintain a priority queue of cells
    found_path = []
    
    while queue:
        priority, cell = heappop(queue)
        boxes.append(cell)

        if cell == box2:
            found_path.insert(0, cell)
            while(paths[cell] != None):
                found_path.insert(0, paths[cell])
                cell = paths[cell]
            #print(found_path)
            return detail_points(found_path, initial_position, destination)
        
        # investigate children
        for child in mesh["adj"].get(cell):
            # calculate cost along this path to child
            cost_to_child = priority + midPointDistance(cell, child) # transition cost will be distance cost from one box to neighbor 
            #print("cost2child:", cost_to_child ,  "CHILD: " , child , "CELL: " , cell)
            if child not in pathcosts or cost_to_child < pathcosts[child]:
                pathcosts[child] = cost_to_child            # update the cost
                paths[child] = cell                         # set the backpointer
                heappush(queue, (cost_to_child, child))     # put the child on the priority queue
            
    return []

def midPointDistance(box1, box2):
    point1 = midpoint(box1)
    point2 = midpoint(box2)
    return distance(point1, point2)

def midpoint(p1):
    return ((p1[0] + p1[1]) / 2, (p1[2] + p1[3]) / 2)
    
def distance(p1, p2):
    #Euclidean distance formula on two points
    return ((p2[1]-p1[1])**(2) + (p2[0]-p1[0])**(2))**(1/2)
    # detail of current cell to exact point of destination (detail point of cell, exact destination point)

def breadth_first_search(start_box, destination_box, mesh, source_point, destination_point, boxes):
    #breadth first search from https://www.redblobgames.com/pathfinding/a-star/introduction.html
    frontier = queue.Queue()
    frontier.put(start_box)
    came_from = dict()
    came_from[start_box] = None
    found_path = []


    while not frontier.empty():
        current = frontier.get()
        boxes.append(current)
        
        if current == destination_box: 
            print("found")
            #trace back through boxes that found_path travelled through
            found_path.insert(0, current)
            while(came_from[current] != None):
                found_path.insert(0, came_from[current])
                current = came_from[current]
            print(found_path)
            return detail_points(found_path, source_point, destination_point)

        for next in mesh["adj"].get(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
        # keep track of all boxes visited
    
    #if no path found
    print("No path!")
    return []

def detail_points(pathList, startPoint, endPoint):
    segments = {}
    #box points (y1 y2 x1 x2) : (x1, y1)
    segments[pathList[0]] = startPoint

    #find the detail points given the boxes traversed in pathList
    for x in range(0, len(pathList)- 1):
        xRange = [max(pathList[x][2], pathList[x+1][2]), min(pathList[x][3], pathList[x+1][3])]
        yRange = [max(pathList[x][0], pathList[x+1][0]), min(pathList[x][1], pathList[x+1][1])]
        
        #find if startPoint is closest to either corner or straight line to next box
        if (startPoint[1] < xRange[0]):
            xVal = xRange[0]
        elif (startPoint[1] > xRange[1]):
            xVal = xRange[1]
        else:
            xVal = startPoint[1]
        if (startPoint[0] < yRange[0]):
            yVal = yRange[0]
        elif (startPoint[0] > yRange[1]):
            yVal = yRange[1]
        else:
            yVal = startPoint[0]
        #add the (box, detail point) as (key, value) pair in segments dictionary
        segments[pathList[x]] = (yVal, xVal)
    segments[pathList[len(pathList) - 1]] = endPoint
    print("SEGMENTS: ", segments)

    return segments

def distance(p1, p2):
    #Euclidean distance formula on two points
    return ((p2[1]-p1[1])**(2) + (p2[0]-p1[0])**(2))**(1/2)