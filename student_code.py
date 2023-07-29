import math
import heapq


def estimated_distance(start_node, end_node):
    """
    Heuristic function to calculate distance between two nodes
    Use hypotenuse of a right triangle.
    
    Input: start_node position (x, y)
           end_node position (x, y)
    Output: Estimated distance between start and end node
    """
    
    return math.hypot(end_node[0] - start_node[0], end_node[1] - start_node[1])


def shortest_path(M, start, goal):
    """
    Main function to find the shortest path
    
    Input:  M = Map
            start = start node
            goad = destination node
    Output: List of nodes of shorted path
    """
    
    print("shortest path called")
    
    # Initialisation
    node_path = {}
    node_path[start] = 0
    
    current_cost = {}
    current_cost[start] = 0
    
    frontier = [(0, start)]
    
    # Exploring paths
    while len(frontier) > 0:
        
        current_node = heapq.heappop(frontier)[1] # heapq is implemented to provide priority queue.
                                                  # select priority node  
        #print('current_node: ', current_node)
        #print('current_cost: ', current_cost)
        
        # Stop if current node reaches destination
        if current_node == goal:
            break
        
        # Check for all neighbor nodes
        for neighbor_node in M.roads[current_node]:
            
            # Use Heuristic function to calculate path cost (h)
            # the distance between current node and neighbor node
            path_cost = estimated_distance(M.intersections[current_node], M.intersections[neighbor_node])
            
            # Calculate total cost (f = g + h)
            # f = total cost
            # g = path cost so far
            # h = estimated distance to destination
            new_cost = path_cost + current_cost[current_node]
            
            # Collect the possible nodes and their cost 
            # Set priority of nodes from their cost
            if neighbor_node not in current_cost or new_cost < current_cost[neighbor_node]:
                node_path[neighbor_node] = current_node
                current_cost[neighbor_node] = new_cost
                heapq.heappush(frontier, (new_cost, neighbor_node))
    
    #print('node_path: ', node_path)
    
    # If destination node is not obseved in the possible paths
    if goal not in node_path:
        return "Destination {} not found".format(goal)
    
    # Find the shortest routh
    current_node = goal
    path = list()
    
    # Collect node from destination back to start node
    while current_node != start:
        path.append(current_node)
        current_node = node_path[current_node]
    path.append(start)
    
    # Revert list to get shortest path from start node to destination node
    path.reverse()
    
    print('shortest path: ', path)
    
    return path