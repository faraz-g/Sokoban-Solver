import sys
from time import time
from map import SokobanMap
from copy import deepcopy
from queue import PriorityQueue


class State:
    def __init__(self, sokoban_map, cost, parents, searchtype):
        self.sokobanMap = sokoban_map
        self.cost = cost
        self.parents = parents
        self.searchtype = searchtype

    def __hash__(self):
        boxes = self.sokobanMap.box_positions
        playerX = self.sokobanMap.player_x
        playerY = self.sokobanMap.player_y

        return pow(hash(frozenset(boxes)), 2) * pow(hash(playerX), 3) * pow(hash(playerY), 5)

    def __eq__(self, other):
        boxes = self.sokobanMap.box_positions
        playerX = self.sokobanMap.player_x
        playerY = self.sokobanMap.player_y

        if not isinstance(other, State):
            return False
        if not (other.sokobanMap.player_x == playerX) or not \
                (other.sokobanMap.player_y == playerY):
            return False
        if not set(boxes) == set(other.sokobanMap.box_positions):
            return False
        return True

    def __lt__(self, other):
        return hash(self) < hash(other)

    def next_states(self):
        next_states = []
        for move in ('l', 'r', 'u', 'd'):
            modState = deepcopy(self.get_map())
            newParents = deepcopy(self.get_parents())
            newCost = 0
            newSearchType = self.searchtype
            if modState.apply_move(move):
                if self.searchtype == "UCS" or self.searchtype == 1:
                    newCost = self.cost + 1
                elif self.searchtype == "A*" or self.searchtype == 2:
                    newCost = heuristic(modState) + self.cost + 1

                if move == 'l':
                    newParents.append('l')
                    nextState = State(modState, newCost, newParents, newSearchType)
                    if not nextState.is_dead_lock():
                        next_states.append(nextState)
                elif move == 'r':
                    newParents.append('r')
                    nextState = State(modState, newCost, newParents, newSearchType)
                    if not nextState.is_dead_lock():
                        next_states.append(nextState)
                elif move == 'u':
                    newParents.append('u')
                    nextState = State(modState, newCost, newParents, newSearchType)
                    if not nextState.is_dead_lock():
                        next_states.append(nextState)
                elif move == 'd':
                    newParents.append('d')
                    nextState = State(modState, newCost, newParents, newSearchType)
                    if not nextState.is_dead_lock():
                        next_states.append(nextState)

        return next_states

    def is_dead_lock(self):
        boxes = self.sokobanMap.box_positions
        targets = self.sokobanMap.tgt_positions
        obstacles = self.sokobanMap.obstacle_map
        height = self.sokobanMap.y_size
        width = self.sokobanMap.x_size
        cornerlock = False
        edgelock = False

        for box in boxes:
            up = (box[1] == 0) or ((box[0], box[1] + 1) in obstacles)
            down = (box[1] == height - 1) or ((box[0], box[1] - 1) in obstacles)
            left = (box[0] == 0) or ((box[0] - 1, box[1]) in obstacles)
            right = (box[1] == width - 1) or ((box[0] + 1, box[1]) in obstacles)

            c_lock = (up or down) and (left or right)
            if not cornerlock and c_lock:
                cornerlock = True

        for box in boxes:
            elock = False
            if (box[1] == 0) or (box[1] == width - 1):
                for target in targets:
                    if box[0] - target[0] != 0:
                        if not elock:
                            elock = True
            elif (box[0] == height - 1) or (box[0] == 0):
                for target in targets:
                    if box[1] - target[1] != 0:
                        if not elock:
                            elock = True
            if not edgelock and elock:
                edgelock = True

        return cornerlock or edgelock

    def is_goal(self):
        targets = self.sokobanMap.tgt_positions
        boxes = self.sokobanMap.box_positions
        for box in boxes:
            if box not in targets:
                return False
        return True

    def get_map(self):
        return self.sokobanMap

    def get_parents(self):
        return self.parents

    def get_cost(self):
        return self.cost


def manDistance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def heuristic(SokoMap):
    boxes = deepcopy(SokoMap.box_positions)
    targets = deepcopy(SokoMap.tgt_positions)
    player = (SokoMap.player_y, SokoMap.player_x)
    playerToBox = sys.maxsize
    cost = 0

    while boxes:
        minDistance = sys.maxsize
        minBox = boxes[0]
        minTarget = targets[0]

        for box in boxes:
            for target in targets:
                distance = manDistance(box, target)
                if distance < minDistance:
                    minDistance = distance
                    minBox = box
                    minTarget = target

        boxes.remove(minBox)
        targets.remove(minTarget)
        cost += minDistance

    boxes2 = deepcopy(SokoMap.box_positions)
    for box in boxes2:
        distance = manDistance(box, player)
        if distance < playerToBox:
            playerToBox = distance

    return cost + playerToBox


def UCS(initState):
    startTime = time()
    PQ = PriorityQueue()
    PQ.put((initState.get_cost(), initState))
    visitedStates = set()
    nodes = 1
    fringe = 1

    while True:
        cost, node = PQ.get()
        fringe += 1
        if node.is_goal():
            moves = node.get_parents()
            return moves, nodes, nodes - fringe, len(visitedStates), time() - startTime
        for s in node.next_states():
            nodes += 1
            if s.is_goal():
                moves = s.get_parents()
                return moves, nodes, nodes - fringe, len(visitedStates), time() - startTime
            if s not in visitedStates:
                visitedStates.add(s)
                PQ.put((s.get_cost(), s))


def main(args):
    if len(args) != 3:
        return
    inputFile = sys.argv[1]
    outputFile = sys.argv[2]
    levelToComplete = SokobanMap(inputFile)
    levelState = State(levelToComplete, 0, [], 1)
    UCSOutput = UCS(levelState)
    moves = UCSOutput[0]
    print("Nodes Generated:", UCSOutput[1])
    print("Nodes on Fringe:", UCSOutput[2])
    print("Explored Nodes:", UCSOutput[3])
    print("Time Taken:", UCSOutput[4], "seconds")

    outputString = ','.join(moves)

    f = open(outputFile, "w")
    f.write(outputString)
    f.close()


if __name__ == '__main__':
    main(sys.argv)
