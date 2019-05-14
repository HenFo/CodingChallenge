import json
import math


class Node:
    """Represents a Node in a Graph
    """

    def __init__(self, name, ID, targets):
        """Constructor

        Arguments:
            name {String} -- Name of the Node
            ID {Integer} -- Index of the Node
            targets {list} -- a list of tuples wich contains the ID and the distance to connected nodes 
        """
        self.name = name
        self.ID = ID
        self.targets = targets
        self.marked = False
        self.distance_from_start = math.inf
        self.prev = None

    def setDistance(self, distance):
        """sets the total distance between this node and the startingnode

        Arguments:
            distance {float} -- distance between this node and the startingnode 
        """
        self.distance_from_start = distance

    def __repr__(self):
        return "Name: %s; Index: %s, distance from start: %s" % (self.name, self.ID, self.distance_from_start)


class Priorityqueue:
    """Partial implementation of a Priority-Queue 
    """

    def __init__(self):
        self.content = []

    def add(self, newNode):
        """Adds a new Node at the correct place to the queue, depending on its distance from the startingnode

        Arguments:
            newNode {Node} -- New node to be added
        """
        if self.isEmpty():
            self.content.append(newNode)
        else:
            for current_node in enumerate(self.content):
                if newNode.distance_from_start <= current_node[1].distance_from_start:
                    self.content.insert(current_node[0], newNode)
                    break
            else:
                self.content.append(newNode)

    def getNext(self):
        """removes the first element from the queue and returns it
        """
        return self.content.pop(0)

    def isEmpty(self):
        """checks if the queue is empty
        """
        return self.content == []

    def contains(self, node):
        return node in self.content

    def update(self, node):
        if self.contains(node):
            self.content.remove(node)
            self.add(node)
        else:
            raise ValueError("%s could not be found" % node)


def generateNodes(graphList):
    """Function to generate node objects from a dict

    Arguments:
        graphList {JSON/Dict} -- Python dictionary containing the graph
    """
    nodes = []
    for currenNode in enumerate(graphList['nodes']):
        edges = getEdgesFromNode(currenNode, graphList['edges'])
        nodes.append(Node(currenNode[1]['label'], currenNode[0], edges))

    return nodes


def getEdgesFromNode(nodeWithIndex, edgeList):
    """collects all the edges connected to a Node

    Arguments:
        nodeWithIndex {tuple} -- Index of the node at first place
        edgeList {list} -- List containing all the edges of the graph
    """
    edges = []
    for edge in edgeList:
        if nodeWithIndex[0] == edge['source']:
            edges.append((edge['target'], edge['cost']))
        elif nodeWithIndex[0] == edge['target']:
            edges.append((edge['source'], edge['cost']))
    return edges


def dijkstra(startname, zielname):
    """Implements the Dijkstra Algorythm

    Arguments:
        startname {String} -- Name of the node to start with
        zielname {String} -- Name of the node to be found
    """
    graphNodes = generateNodes(json.load(open("generatedGraph.json")))
    start = startname
    startingNode = None
    # find the startingnode. Raises an error if element was not found
    for node in graphNodes:
        if node.name == start:
            startingNode = node
            break
    else:
        raise AttributeError("%s does not exist" % startname)

    startingNode.setDistance(0)
    queue = Priorityqueue()
    queue.add(startingNode)

    gefunden = False

    # Search as long as the Node has not been found or all connected edges have been processed
    while not gefunden and not queue.isEmpty():
        currentNode = queue.getNext()

        if currentNode.name == zielname:
            # shortest way to targetnode has been found
            gefunden = True
        else:
            # shortest way to the current node has been found
            currentNode.marked = True

            # look through all connetcted nodes if a shorter way was found
            for neighborTuple in currentNode.targets:
                neighborNode = graphNodes[neighborTuple[0]]

                # check if a neighbor node already has a shortest connection
                if not neighborNode.marked:
                    if currentNode.distance_from_start + neighborTuple[1] < neighborNode.distance_from_start:

                        # set new shortest distance
                        neighborNode.setDistance(
                            currentNode.distance_from_start + neighborTuple[1])

                        # set current node as the previous node
                        neighborNode.prev = currentNode

                        # add neighbor node to the queue
                        if queue.contains(neighborNode):
                            queue.update(neighborNode)
                        else:
                            queue.add(neighborNode)

    # check if the target was found or not
    if gefunden:
        # print the nodes leading from start to target node
        prevNode = currentNode.prev
        print(currentNode)
        while prevNode is not None:
            print(prevNode)
            prevNode = prevNode.prev

    # if nothing was found, the target may not exist or is not connected to the start node
    else:
        raise AttributeError("%s may not exist or is not connected to the start node" % zielname)


if __name__ == "__main__":
    dijkstra("Erde", "b3-r7-r4nd7")
