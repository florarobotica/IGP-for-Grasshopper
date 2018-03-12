#!/usr/bin/env python
"""This is a docstring.
This is its end.
"""
import Rhino as rc
import math as math
import Grasshopper as gh
import copy as copy
import random as random
import clr as clr
clr.AddReferenceToFile("LumenWorks.Framework.IO.dll")
from System.IO import StringReader
from LumenWorks.Framework.IO.Csv import CsvReader

#args = parser.parse_args()

# # globals
graphs = []
maxNodeID = 0
envAsList = []
# for sending data out to gh
NodeID = gh.DataTree[object]()
NodePositions = gh.DataTree[object]()
Connections = gh.DataTree[object]()
SensorValues = gh.DataTree[object]()
# for bringing data in, from gh
ROOTPOSITION = RootLocations
# size and orientations of new branches when VMC grows
new_direction_a = GeometryOfGrowth[0]
new_direction_b = GeometryOfGrowth[1]
new_direction_c = GeometryOfGrowth[2]
# environment box 
EnvX = max(abs(EnvironmentBox.X[0]),abs(EnvironmentBox.X[1]))
EnvY = max(abs(EnvironmentBox.Y[0]),abs(EnvironmentBox.Y[1]))
EnvZ = max(abs(EnvironmentBox.Z[0]),abs(EnvironmentBox.Z[1]))

#####################
# VMC parameters
GROWTHTHRESHOLD = 4
GROWTHAMOUNT = 1 #should be 0-1 if probabalistic i.e. if growth function is 0; should be int 1+ if quantitiy i.e. if growth function is 1 or 2
GROWTHFUNCTION = int(GrowthStyle) #0 means probabilistically and 1 means quantity of nodes randomly chosen and 2 means quantity of nodes chosen by bestness
ROOTRESOURCE = float(ResourceConstant)
SUCCESSRATE_C = float(SuccessConstant)
SUCCESSRATE_SENSORS = [1.0]
TRANSFERRATE_C = float(LimitConstant)
TRANSFERRATE_SENSORS = [0.0]
COMPETITIONRATE_C = float(CompetitionConstant)
COMPETITIONRATE_SENSOR = [0.0]
ALPHA = float(SpeedOfAdaptation)

NODECONSUMPTION = 1.0

#############
def bringVoracityFromGH(GROWTHAMOUNT):
    g = GROWTHAMOUNT
    if GROWTHFUNCTION == 0:
        g = float(GrowthVoracity)
    else: 
        g = int(GrowthVoracity)
    return g
############


def ImportEnv():
    #read each line from the file
    file = open(EnvironmentData, "r")
    contents = file.read()
    reader = CsvReader(StringReader(contents), False)
    reader.SkipEmptyLines = False
    rows = []
    while reader.ReadNextRecord():
        row = []
        colCount = reader.FieldCount
        for i in range(colCount):
            row.append(reader[i])
        rows.append(row)
    file.close()
    global envAsList
    envAsList = rows[0]
    return envAsList


#####################
def getSensorValues(position): # scale (remap) values properly!!!
    scaledSensor = [int((position[0]/EnvX)*100),int((position[1]/EnvY)*100),int((position[2]/EnvZ)*100)]
    resourceIndex = int((scaledSensor[0]*(100*100))+(scaledSensor[1]*100)+scaledSensor[2])
    if scaledSensor[0] >= 0 and scaledSensor[0] < 100 and scaledSensor[1] >= 0 and scaledSensor[1] < 100 and scaledSensor[2] >= 0 and scaledSensor[2] < 100:
        sensor1 = float(envAsList[resourceIndex])
    else: 
        sensor1 = float(0.0)
#    print("***"+str(sensor1))
    return [sensor1]

def getSuccessProduction_leaf(position): # between 0 and 1
    global SUCCESSRATE_C
    global SUCCESSRATE_SENSORS
    sensorv = getSensorValues(position)
    assert len(sensorv) == len(SUCCESSRATE_SENSORS)
    s = SUCCESSRATE_C
    for i in range(len(sensorv)):
        s += SUCCESSRATE_SENSORS[i] * sensorv[i]
    s = min(max(0.0,s),1.0)
    return s

def getTransferRate(position): # between 0 and 1
    global TRANSFERRATE_C
    global TRANSFERRATE_SENSORS
    sensorv = getSensorValues(position)
    assert len(sensorv) == len(TRANSFERRATE_SENSORS)
    tran = TRANSFERRATE_C
    for i in range(len(sensorv)):
        tran += TRANSFERRATE_SENSORS[i] * sensorv[i]
    tran = max(min(tran,1.0),0.0)
    return tran

def getCompetitionRate(position):
    global COMPETITIONRATE_C
    global COMPETITIONRATE_SENSOR
    sensorv = getSensorValues(position)
    assert len(sensorv) == len(COMPETITIONRATE_SENSOR)
    comp = COMPETITIONRATE_C
    for i in range(len(sensorv)):
        comp += COMPETITIONRATE_SENSOR[i]*sensorv[i]
    # do not allow too large values, e.g. comp = min(10,comp)
    return comp

class Node:
    # id
    # mygraph
    # childConnections
    # parentConnections
    # R
    # S
    # newS
    # newR
    # position
    
    def __init__(self, mygraph, parentNode, position, id=0):
        self.R = 0.0
        self.S = 0.0
        self.newS = 0.0
        self.newR = 0.0
        
        self.mygraph = mygraph
        self.id = id
        self.childConnections = []
        self.parentConnections = []
        self.position = position
        if parentNode != "":
            parentConnection = Connection(parentNode, self)
            self.parentPosition = parentNode.position
        else: # root
            self.R = mygraph.rootresource
            self.newR = mygraph.rootresource
            self.parentPosition = self.position
        
        self.mygraph.nodes.append(self)
        

    def distributeR(self):
        global NODECONSUMPTION
        totalV = 0
        for connection in self.childConnections:
            totalV += connection.V
        if totalV > 0:
            for connection in self.childConnections:
                connection.child.newR = max(0,self.R - NODECONSUMPTION) * connection.V / totalV
        else:
            for connection in self.childConnections:
                connection.child.newR = max(0,self.R - NODECONSUMPTION) * 1.0/len(self.childConnections)

    def updateS(self):
        if len(self.childConnections) == 0: # leaf
            self.newS = getSuccessProduction_leaf(self.position)
        else:
            totalS = 0
            for connection in self.childConnections:
                totalS += connection.child.S
            self.newS = totalS * getTransferRate(self.position)

    def updateV(self):
        global ALPHA
        for connection in self.childConnections:
            expectedV = pow(connection.child.S, getCompetitionRate(self.position))
            connection.V = connection.V + ALPHA * (expectedV - connection.V)

    def updateParallel(self):
        self.S = self.newS
        self.R = self.newR

    def grow(self):
        global maxNodeID
        for i in range(3): # number of branches: ALL CODE BELOW DEPENDS ON THIS BEING == 3
            #define existing and target vectors 
            parentvector = rc.Geometry.Vector3d(self.position[0] - self.parentPosition[0],self.position[1] - self.parentPosition[1],self.position[2] - self.parentPosition[2])
            zvector = rc.Geometry.Vector3d(0,0,1)
            #move points to current position of growth
            moved_direction_a = [self.position[0] + new_direction_a[0], self.position[1] + new_direction_a[1], self.position[2] + new_direction_a[2]] 
            moved_direction_b = [self.position[0] + new_direction_b[0], self.position[1] + new_direction_b[1], self.position[2] + new_direction_b[2]] 
            moved_direction_c = [self.position[0] + new_direction_c[0], self.position[1] + new_direction_c[1], self.position[2] + new_direction_c[2]] 
            #cast as rhino geometry
            vectororigin = rc.Geometry.Point3d(self.position[0],self.position[1],self.position[2])
            new_point_a = rc.Geometry.Point3d(moved_direction_a[0],moved_direction_a[1],moved_direction_a[2])
            new_point_b = rc.Geometry.Point3d(moved_direction_b[0],moved_direction_b[1],moved_direction_b[2])
            new_point_c = rc.Geometry.Point3d(moved_direction_c[0],moved_direction_c[1],moved_direction_c[2])
            #make copy of points
            branch_a = copy.copy(new_point_a)
            branch_b = copy.copy(new_point_b)
            branch_c = copy.copy(new_point_c)
            #define transfomration
            rotate_transform = rc.Geometry.Transform.Rotation(zvector,parentvector,vectororigin)
            #Transform points (No output, transform copied points)
            rc.Geometry.Point3d.Transform(branch_a,rotate_transform)
            rc.Geometry.Point3d.Transform(branch_b,rotate_transform)
            rc.Geometry.Point3d.Transform(branch_c,rotate_transform)
            #Place the new VMC leaves
            if i == 0:
                direction = [branch_a[0],branch_a[1],branch_a[2]]
            elif i == 1:
                direction = [branch_b[0],branch_b[1],branch_b[2]]
            else:
                direction = [branch_c[0],branch_c[1],branch_c[2]]
            maxNodeID += 1
            new_node_id = maxNodeID
            new_node_position = [direction[0], direction[1], direction[2]] 
            Node(self.mygraph, self, new_node_position, new_node_id)


              
class Connection:
    def __init__(self, parent, child):
        self.V = 0
        self.parent = parent
        self.child = child
        self.parent.childConnections.append(self)
        self.child.parentConnections.append(self)


class Graph:
    def __init__(self, rootresource, rootposition, id=0):
        global maxNodeID
        self.nodes = []
        self.id = id
        self.rootresource = rootresource
        self.root = Node(self, "", rootposition, maxNodeID)
        maxNodeID += 1
    
    
    def updateNodes(self):
        for a_node in self.nodes:
            a_node.updateS()
            a_node.updateV()
        for a_node in self.nodes:
            a_node.distributeR()
        #
        for a_node in self.nodes:
            a_node.updateParallel()

    def printGraph(self):
        print(self.id,"***********",len(self.nodes))
        for node in self.nodes:
            print(node.id, node.position)


def initializeRoots():
    # where are the roots? or how many roots/graphs we have to make?
    for rootposition in ROOTPOSITION:
        a_graph = Graph(ROOTRESOURCE, rootposition)
        graphs.append(a_graph)

def getNodesToGrow():
    GROWTHAMOUNT = bringVoracityFromGH(GrowthVoracity)
    nodes_to_grow = []
    for graph in graphs:
        candidates = []
        for node in graph.nodes:
            if len(node.childConnections)==0 and node.R > GROWTHTHRESHOLD:
                candidates.append(node)

        if GROWTHFUNCTION == 0:
            for leaf in candidates:
                if random.random() < GROWTHAMOUNT:
                    nodes_to_grow.append(leaf)

        elif GROWTHFUNCTION == 1:
            for i in range(GROWTHAMOUNT):
                if len(candidates) > 0:
                    ind = random.randint(0,len(candidates)-1)
                    nodes_to_grow.append(candidates[ind])
                    candidates.remove(candidates[ind])

        elif GROWTHFUNCTION == 2:
            random.shuffle(candidates)
            for i in range(GROWTHAMOUNT):
                if len(candidates) > 0:
                    best = 0
                    for j in range(len(candidates)):
                        if candidates[best].R < candidates[j].R:
                            best = j
                    nodes_to_grow.append(candidates[best])
                    candidates.remove(candidates[best])

    #for a in nodes_to_grow:
    #    print(a.id,"**")
    #print (len(nodes_to_grow),"-------------------------------------------------")
    return nodes_to_grow


def main():
    global graphs
    global GROWTHTHRESHOLD
    
    print("start")
    
    #contents = ImportEnv()
    #print "fromcsv"
    #print contents
    ImportEnv()
    #print envAsList[0]
    
    initializeRoots()
    
    nodes_to_grow = getNodesToGrow()
    

    timestep = 0
    done = False
    if TriggerLoop == True:
        while timestep < int(Timesteps):
    
            #print("time step: "+str(timestep))
            
            ###########################
            for graph in graphs:
                graph.updateNodes()
            
            for a_node in nodes_to_grow:
                #print("growing node: "+str(a_node.id))
                a_node.grow()
               
            
            nodes_to_grow = getNodesToGrow()
            ###########################
            
            timestep += 1


        NID = []
        NPos = []
        Cons = []
        SVal = []
        for a_graph in graphs:
            for a_node in a_graph.nodes:
                NID.append(a_node.id)
                NPos.append(rc.Geometry.Point3d(a_node.position[0],a_node.position[1],a_node.position[2]))
                if len(a_node.childConnections) == 0:
                    SVal.append([a_node.S,a_node.id])
                for connection in a_node.parentConnections:
                    Cons.append([connection.parent.id,connection.child.id])


        print("all nodes")
        for a in NID:
            print a
        print("all positions")
        for a in NPos:
            print a
        print("all connections")
        for a in Cons:
            print a
            
                
        #send out
        for i,l in enumerate(NID):
            NodeID.Add(l,gh.Kernel.Data.GH_Path(i))
        for i,l in enumerate(Cons):
            Connections.AddRange(l,gh.Kernel.Data.GH_Path(i))
        for i,l in enumerate(SVal):
            SensorValues.AddRange(l,gh.Kernel.Data.GH_Path(i))
        for i,l in enumerate(NPos):
            NodePositions.Add(l,gh.Kernel.Data.GH_Path(i))
        #for i in range(len(NID)):
        #    NodeID.Add(NID[i],GH_Path(i))
        #for i in range(len(Cons)):
        #    Connections.Add(Cons[i],GH_Path(i))


if __name__ == "__main__":
    main() # args

    
