from blob_detector import SimpleBlobDetector
from math_utils import *

MAX_ITERATIONS = 2000
BIN_SIZE = 12
HEURISTIC_WEIGHT = 5.0

# Eight-connected graph. First four are one unit away, next are sqrt 2 units away
def getNeighbors(coord):
    neighbors = []
    neighbors.append((coord[0] - 1, coord[1]))
    neighbors.append((coord[0] + 1, coord[1]))
    neighbors.append((coord[0], coord[1] - 1))
    neighbors.append((coord[0], coord[1] + 1))

    neighbors.append((coord[0] - 1, coord[1] - 1))
    neighbors.append((coord[0] - 1, coord[1] + 1))
    neighbors.append((coord[0] + 1, coord[1] - 1))
    neighbors.append((coord[0] + 1, coord[1] + 1))
    return neighbors

# Updates costs downstream of the given coordinate
def updateCosts(graph, coordinate):
    node = graph[coordinate[1]][coordinate[0]]
    for i in range(len(node.getChildren())):
        child_coord = node.getChildren()[i]
        child_node = graph[child_coord[1]][child_coord[0]]

        cost = node.getCostFromStart() + distance2D(coordinate, child_coord) + child_node.getHeuristicCost()
        if (cost < child_node.getTotalCost()):
            child_node.setParentCoord(coordinate)
            child_node.setCostFromStart(node.getCostFromStart() + distance2D(coordinate, child_coord))
            updateCosts(graph, child_coord)

def toBinCoord(imgCoord):
    return int(imgCoord / BIN_SIZE)

def binCoordToImageSpace(coordinate):
    x = coordinate[0] * BIN_SIZE + 0.5 * BIN_SIZE
    y = coordinate[1] * BIN_SIZE + 0.5 * BIN_SIZE
    return (x, y)

class PathPlanner:
    def planPath(self, image, startImgCoords, goalImgCoords):
        imageSizeX = len(image[0])
        imageSizeY = len(image)

        binSizeX = toBinCoord(imageSizeX - 1)
        binSizeY = toBinCoord(imageSizeY - 1)

        map = [[False for xi in range(binSizeX)] for yi in range(binSizeY)]
        graph = [[None for xi in range(binSizeX)] for yi in range(binSizeY)]
        expanded = [[False for xi in range(binSizeX)] for yi in range(binSizeY)]

        for y in range(imageSizeY):
            for x in range(imageSizeX):
                pixel = image[y][x]
                if (int(pixel[0]) + int(pixel[1]) + int(pixel[2]) != 0):
                    map[toBinCoord(y)][toBinCoord(x)] = True

        start = (toBinCoord(startImgCoords[0]), toBinCoord(startImgCoords[1]))
        goal = (toBinCoord(goalImgCoords[0]), toBinCoord(goalImgCoords[1]))
        start_node = Node(HEURISTIC_WEIGHT * distance2D(start, goal))

        print("Start: " + str(start[0]) + ", " + str(start[1]))
        print("Goal: " + str(goal[0]) + ", " + str(goal[1]))

        numRejected = 0

        coord_queue = []
        coord_queue.append(start)
        graph[start[1]][start[0]] = start_node

        if (map[goal[1]][goal[0]]):
            print("Goal isn't valid, aborting")
            return []

        least_cost = 1e4
        least_cost_coord = None
        iterations = 0
        found_goal = False

        while (iterations < MAX_ITERATIONS and not found_goal and len(coord_queue) > 0):
            iterations += 1
            coord_queue.sort(key = lambda c : graph[c[1]][c[0]].getTotalCost())

            coord = coord_queue.pop(0)
            neighbors = getNeighbors(coord)
            node = graph[coord[1]][coord[0]]

            for i in range(len(neighbors)):
                neighbor = neighbors[i]
                if (neighbor[0] < 0 or neighbor[0] >= binSizeX):
                    continue
                if (neighbor[1] < 0 or neighbor[1] >= binSizeY):
                    continue
                if (map[neighbor[1]][neighbor[0]]):
                    continue
                if (expanded[neighbor[1]][neighbor[0]]):
                    continue

                node.addChild(neighbor)
                edge_cost = distance2D(coord, neighbor)

                heuristic_cost = HEURISTIC_WEIGHT * distance2D(neighbor, goal)
                neighbor_node = Node(heuristic_cost)
                neighbor_node.setParentCoord(coord)
                neighbor_node.setCostFromStart(edge_cost + node.getCostFromStart())

                if (neighbor_node.getTotalCost() < least_cost):
                    least_cost = neighbor_node.getTotalCost()
                    least_cost_coord = neighbor

                existing_node = graph[neighbor[1]][neighbor[0]]
                if (existing_node is None):
                    graph[neighbor[1]][neighbor[0]] = neighbor_node
                elif (neighbor_node.getTotalCost() < existing_node.getTotalCost()):
                    graph[neighbor[1]][neighbor[0]] = neighbor_node
                    updateCosts(graph, neighbor)

                if (neighbor[0] == goal[0] and neighbor[1] == goal[1]):
                    least_cost_coord = neighbor
                    found_goal = True

                if (not neighbor in coord_queue):
                    coord_queue.append(neighbor)

            expanded[coord[1]][coord[0]] = True

        path = []
        coord_to_add = least_cost_coord

        print("Finished")
        print("Iterations: " + str(iterations))
        print("Found goal: " + str(found_goal))
        print("Num rejected: " + str(numRejected))

        while True:
            path.append(coord_to_add)
            node = graph[coord_to_add[1]][coord_to_add[0]]
            if (node.getParentCoord() == None):
                break
            else:
                coord_to_add = node.getParentCoord()
        path.reverse()

        print("Path: ")
        for i in range(len(path)):
            print("\t" + str(path[i][0]) + ", " + str(path[i][1]))

        return [binCoordToImageSpace(c) for c in path], expanded

class Node:
    def __init__(self, heuristic_cost):
        self.parent_coord = None
        self.children_coords = []
        self.cost_from_start = 0.0
        self.heuristic_cost = heuristic_cost

    def getTotalCost(self):
        return self.cost_from_start + self.heuristic_cost

    def setParentCoord(self, parent_coord):
        self.parent_coord = parent_coord

    def getParentCoord(self):
        return self.parent_coord

    def addChild(self, child_coord):
        self.children_coords.append(child_coord)

    def getChildren(self):
        return self.children_coords

    def setCostFromStart(self, cost_from_start):
        self.cost_from_start = cost_from_start

    def getCostFromStart(self):
        return self.cost_from_start

    def getHeuristicCost(self):
        return self.heuristic_cost
