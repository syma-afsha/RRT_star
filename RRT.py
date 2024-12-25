from matplotlib import pyplot as plt
from PIL import Image
import numpy as np
import sys
import random
import math

class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
        self.parent = None
    
    def dist(self, p):
        return math.sqrt((self.x - p.x)**2 + (self.y - p.y)**2)
    
    def __str__(self):
        return "({}, {})".format(self.x, self.y)

# Function to load and process the image map for path finding
def process_image_map(image_path):
    image = Image.open(image_path).convert('L')
    grid_map = np.array(image.getdata()).reshape(image.size[0], image.size[1])/255
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0
    grid_map = (grid_map * -1) + 1
    return grid_map

# Function to create a random point in the space
def create_random_point(probability, xmax, ymax, goal_point):
    if random.random() > probability:
        random_point = Point(random.randint(0, xmax), random.randint(0, ymax))
    else: 
        random_point = goal_point
    return random_point

# Function to find the nearest node in the graph to a given point
def find_nearest_node(random_point, graph):
    distances = [vertex.dist(random_point) for vertex in graph]
    min_index = distances.index(min(distances))
    return graph[min_index]

# Function to move towards a point by a fixed distance
def move_towards_point(step_size, start_point, end_point):
    if start_point.x == end_point.x and start_point.y == end_point.y:
        return start_point
    direction = Point(end_point.x - start_point.x, end_point.y - start_point.y)
    direction_length = math.sqrt(direction.x**2 + direction.y**2)
    normalized = Point(direction.x/direction_length, direction.y/direction_length)
    return Point(math.floor(start_point.x + step_size * normalized.x), math.floor(start_point.y + step_size * normalized.y)) 

# Function to determine a new node's position considering the step size
def determine_new_node(closest_node, random_point, step_size):
    distance = random_point.dist(closest_node)
    if distance < step_size:
        return random_point
    return move_towards_point(step_size, closest_node, random_point)

# Function to check if a line segment is free from obstacles
def check_segment_free(start_node, end_node, grid_map):
    num_points = math.floor(start_node.dist(end_node))
    for i in range(1, num_points + 2):
        point = move_towards_point(i, start_node, end_node)
        if grid_map[point.y, point.x] == 1:
            return False
    return True

# Function to construct the path from start to goal
def construct_path(graph, current_node):
    path = [current_node]
    while current_node in graph:
        current_node = current_node.parent
        if current_node is not None:
            path.insert(0, current_node)
    return path

# Main RRT algorithm function
#RRT Algorithm
def rrt(grid_map, K, delta_q, p, qstart, qgoal):
    G = [qstart]
    plt.figure(figsize=(10,10))
    for k in range(K):
        qrand = create_random_point(p, len(grid_map[0])-2, len(grid_map)-2, qgoal)
        qnear = find_nearest_node(qrand, G)
        qnew = determine_new_node(qnear, qrand, delta_q)
        
        #Plotting the random paths
        if check_segment_free(qnear, qnew, grid_map):
            plt.subplot(1,2,1)
            plt.scatter(qnew.x,qnew.y, c='black', marker="+")
            plt.plot([qnear.x,qnew.x],[qnear.y,qnew.y],'g--')
            G.append(qnew)
            qnew.parent = qnear
            if qnew.x == qgoal.x and qnew.y == qgoal.y:
                path = construct_path(G, qnew)
                return path, k
    return None

#Smoothing the resulting path
def rrt_smoothing(grid_map, path):
    
    start_counter = 0
    goal_counter = len(path) - 1

    start = path[start_counter]
    goal = path[goal_counter]
    smooth_path = [goal]
    while goal.x != start.x or goal.y != start.y:
        if check_segment_free(start, goal, grid_map):
            smooth_path.insert(0, start)
            goal = path[start_counter]
            start_counter = 0
            start = path[start_counter]
        else:
            start_counter = start_counter + 1
            start = path[start_counter]
    return smooth_path

def path_distance(path):
    total_distance = 0
    for vertex in range(len(path)-1):
        dist = math.sqrt((path[vertex].x - path[vertex+1].x)**2 + (path[vertex].y - path[vertex+1].y)**2)
        total_distance = total_distance + dist
    
    return total_distance
        



def main():
    image_path = str(sys.argv[1])
    K = int(sys.argv[2])
    delta_q = float(sys.argv[3])
    p  = float(sys.argv[4])
    qstart_x  = int(sys.argv[5])
    qstart_y  = int(sys.argv[6])
    qgoal_x  = int(sys.argv[7])
    qgoal_y  = int(sys.argv[8])

    qstart = Point(qstart_y, qstart_x)
    qgoal = Point(qgoal_y, qgoal_x)
    grid_map = process_image_map(image_path)
    
    path, r = rrt(grid_map, K, delta_q, p, qstart, qgoal)
    smooth_path = rrt_smoothing(grid_map, path)
    
    rrt_distance = path_distance(path)
    smooth_distance = path_distance(smooth_path)
    
    #Plotting the resulting path and smoothing path

    if path:
        print('Path found in ', r, ' iterations')
        print('Path Distance: ', rrt_distance)
        plt.subplot(1,2,1)
        print('Resulting Path',[(node.x, node.y) for node in path])
        for i in range(len(path)-1):
            plt.plot([path[i].x,path[i+1].x],[path[i].y,path[i+1].y],'r-')

        plt.scatter(qgoal.x,qgoal.y,100, c="g", marker="*")
        plt.scatter(qstart.x,qstart.y,100, c="r", marker="*")
        plt.title('Resulting Path of RRT')
        plt.imshow(grid_map)
    else:
        print('No Solution Found')
        
        

    if smooth_path:
        print('Smooth Path Distance: ', smooth_distance)
        plt.subplot(1,2,2)
        print('Smoothing Path', [(node.x, node.y) for node in smooth_path])
        for i in range(len(smooth_path)-1):
            plt.plot([smooth_path[i].x,smooth_path[i+1].x],[smooth_path[i].y,smooth_path[i+1].y],'b-')

        plt.scatter(qgoal.x,qgoal.y,100, c="g", marker="*")
        plt.scatter(qstart.x,qstart.y,100, c="r", marker="*")
        plt.title('Smoothing Path of RRT')
        plt.imshow(grid_map)
        plt.show()

if __name__ == "__main__":
    main()

