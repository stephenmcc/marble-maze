from enum import Enum

####### Status Enum #######
SUCCESS = 0
FAILURE = 1
RUNNING = 2

class SequenceNode:
    def __init__(self):
        self.children = []

    def addChild(self, child):
        self.children.append(child)

    def tick(self):
        for i in range(len(self.children)):
            status = self.children[i].tick()
            if (status == FAILURE or status == RUNNING):
                return status
        return SUCCESS

class FallbackNode:
    def __init__(self):
        self.children = []

    def addChild(self, child):
        self.children.append(child)

    def tick(self):
        for i in range(len(self.children)):
            status = self.children[i].tick()
            if (status == SUCCESS or status == RUNNING):
                return status
        return FAILURE
