#python
import numpy as np
import math
from sys import maxsize
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break

def get_obstacle_map(safety_boundary):
    m = Map(100, 100)
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 60):
        ox.append(60)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60)
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)
    
    #Safety boundary
    m.set_obstacle([(i+safety_boundary, j) for i, j in zip(ox, oy)])
    m.set_obstacle([(i-safety_boundary, j) for i, j in zip(ox, oy)])
    m.set_obstacle([(i, j+safety_boundary) for i, j in zip(ox, oy)])
    m.set_obstacle([(i, j-safety_boundary) for i, j in zip(ox, oy)])
        
    return m, ox, oy

def populate_objects(obstacles):
    
    primitiveType = sim.primitiveshape_cuboid

    sizes = [0.10, 0.10, 0.10]
    
    shapeHandles = []
    
    for obs in obstacles:
        shapeHandle = sim.createPrimitiveShape(primitiveType, sizes)
        sim.setObjectPosition(shapeHandle, -1, [obs[0], obs[1], 0])
        sim.setObjectColor(shapeHandle,0,0,[0,0,0])
        shapeHandles.append(shapeHandle)
    
    groupHandle=sim.groupShapes(shapeHandles, True)
    sim.setObjectAlias(groupHandle, 'Obstacles')
   
def EulerToQuaternion():

    rot = Rotation.from_euler('xyz', [1, 1, 0])
    quat = rot.as_quat()

    return quat


def Trajectory(DrawPath):
    
    ControlPoints = []
    
    for path in DrawPath:
      position = [path[0], path[1], 0.1]
      quaternion = EulerToQuaternion()
      ControlPoints.extend(position+quaternion.tolist())
    
    PathLengths,Length=sim.getPathLengths(ControlPoints,7)
    dt = sim.getSimulationTimeStep()
    
    #TrajectoryHandler = sim.createPath(ControlPoints, 16, Length  // dt,0, 0)
    TrajectoryHandler = sim.createPath(ControlPoints, 16, 2 * (Length // dt), 0, 0)

    sim.setInt32Signal("trajectory", TrajectoryHandler)
    
    return TrajectoryHandler

def scale_coordinates(data, input_range, output_range):
    if not isinstance(data, np.ndarray):
        data = np.array(data)
    input_min, input_max = input_range
    output_min, output_max = output_range
    standardized = (data - input_min) / (input_max - input_min)
    scaled_value = standardized * (output_max - output_min) + output_min
    return scaled_value

def sysCall_init():
    
    #Get start object
    mix = [1,1,0] #set color
    getStartObj=sim.getObject('/Start')
    robot=sim.getObjectPosition(getStartObj,-1)
    sim.setObjectColor(getStartObj,0,0,mix)
    start = np.array(robot)
    
    
    #Get Destination/Goal object
    green = [0,1,0] #set color
    getGoalObj=sim.getObject('/Goal')
    goalObj=sim.getObjectPosition(getGoalObj,-1)
    sim.setObjectColor(getGoalObj,0,0,green)
    dest = np.array(goalObj)
    
    #Get Quadcopter
    #quad_handle = sim.getObject('/Sphere[0]')
    #sim.setObjectPosition(quad_handle,-1.5,[-1.5, -1.7, -1.7])
    #quad_pos=sim.getObjectPosition(quad_handle,-1)
 
    #safety safety_boundary
    safety_boundary=6
    
    #Wall / Obstacles
    WallMin=-10
    WallMax=60
    
    #Sim World
    SimMin=-2.5
    SimMax=2.5


    #map scaling
    start = scale_coordinates(start, (SimMin, SimMax), (WallMin, WallMax) ).astype(int)
    dest =  scale_coordinates(dest, (SimMin, SimMax), (WallMin, WallMax) ).astype(int)

    
    m, ox, oy = get_obstacle_map(safety_boundary)
    
    scaleOX= scale_coordinates(np.array(ox), (WallMin, WallMax), (SimMin, SimMax))
    scaleOY= scale_coordinates(np.array(oy), (WallMin, WallMax), (SimMin, SimMax))
    
    #Build Obstacles
    scaleWall = np.stack([scaleOX,scaleOY],axis=1)
    populate_objects(scaleWall)

    #D* from github
    start = m.map[start[0]][start[1]]
    end = m.map[dest[0]][dest[1]]
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)
    
    #Scale Coordinates
    #scaleRX= scale_coordinates(np.array(rx), (WallMin, WallMax), (SimMin, SimMax))
    #scaleRY= scale_coordinates(np.array(ry), (WallMin, WallMax), (SimMin, SimMax))
    #ScalePath = np.stack([scaleRX, scaleRY],axis=1)

    
    
    #Draw Path
    #lineSize = 5
    #maximumLines = 9999
    #blue = [0,0,1]
    
    #drawingObjectHandle=sim.addDrawingObject(sim.drawing_lines, lineSize, 0.0, -1, maximumLines, blue)
    #altitude = 0.1

    #for i in range(len(ScalePath) - 1):
    # Get the current and next points
        #p1 = ScalePath[i]
        #p2 = ScalePath[i+1]
    
    # Create a list of the coordinates to draw a line
        #drawLine = [p1[0], p1[1],0, p2[0], p2[1],0]
    
    # Add the line to the drawing object
        #sim.addDrawingObjectItem(drawingObjectHandle, drawLine)
    
    #scaled path to sim environment
    scaleRX= scale_coordinates(rx, (WallMin, WallMax), (SimMin, SimMax))
    scaleRY= scale_coordinates(ry, (WallMin, WallMax), (SimMin, SimMax))
    
    path = np.stack([scaleRX, scaleRY],axis=1)

    #Trajectory
    TrajectoryHandler = Trajectory(np.stack([scaleRX, scaleRY],axis=1))
