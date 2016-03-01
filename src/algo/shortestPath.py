'''
Created on 12 Feb 2016

@author: Bridge
'''
#===============================================================================
# Map Description
# 0: unexplored cell
# 1: explored cell
# 2: obstacle
# 3: robot head
# 4: robot center
# 5: robot body
# 6: start zone
# 7: goal zone
# 8: free path
# 9: shortest path
#===============================================================================

import Queue
from Queue import PriorityQueue
from algo.constants import *

#Create Priority Queue node
class PqNode(object):
    def __init__(self, _dict):
        self.weight = _dict["weight"]
        self.direction = _dict["direction"]
        self.position = _dict["position"]
    def __lt__(self, rhs):#
        return self["weight"] < rhs["weight"]
    def __gt__(self, rhs):
        return self["weight"] > rhs["weight"]
    def __le__(self, rhs):
        return self["weight"] <= rhs["weight"]
    def __ge__(self, rhs):
        return self["weight"] >= rhs["weight"]
    def __getitem__(self, key):
        return getattr(self, key)
    def __setitem__(self, key, value):
        setattr(self, key, value)

class ShortestPath(object):
    """docstring for ShortestPath"""

    def __init__(self, map, direction, start, goal):
        super(ShortestPath, self).__init__()
        self.map = []
        for row in map:
            self.map.append([])
            for col in row:
                self.map[-1].append(col)
        self.MAX_ROW = len(self.map)
        self.MAX_COL = len(self.map[0])
        # direction: N, E, W, S
        self.directon = direction
        self.start = start
        self.goal = goal
        # note: positive x --> E; positive y --> S
        # coord[0] is y
        # coord[1] is x

    """Main function to calculate the shortest path"""
    def shortestPath(self, mark_value = 9):
        distance = [] # distance between current and start
        prev = [] # previous cell
        next_step = [] # next step in shortest path
        for i in range(len(self.map)):
            distance.append([])
            prev.append([])
            next_step.append([])
            for j in range(len(self.map[i])):
                distance[i].append(INF)
                prev[i].append([-1, -1])
                next_step[i].append([-1, -1])

        # Dijkstra / A*
        priorityqueue = PriorityQueue()
        priorityqueue.put(PqNode({"position": self.start, "direction": self.direction, "weight": 0}))
        distance[self.start[0]][self.start[1]] = 0

        while not priorityqueue.empty():
            head = priorityqueue.get()

            # if reach goal zone, break
            if head["position"][0] == self.goal[0] and head["position"][1] == self.goal[1]:
                break

            # expand head
            neighbors =  self.expand(head["position"])

            # if not yet visited OR can be visited with lower cost, put in priority queue
            for neighbor in neighbors:
                gn = self.cost(head["position"], neighbor, head["direction"])
                cost = gn 
                if distance[head["position"][0]][head["position"][1]] + cost < distance[neighbor[0]][neighbor[1]]:

                    distance[neighbor[0]][neighbor[1]] = distance[head["position"][0]][head["position"][1]] + gn
                    prev[neighbor[0]][neighbor[1]] = head["position"]
                    priorityqueue.put(PqNode({"position": neighbor,
                        "direction": self.direction(head["position"], neighbor),
                        "weight": cost}))

        # construct path from goal
        cur = self.goal
        ret_map = self.map
        while cur[0] != self.start[0] or cur[1] != self.start[1]:
            if mark_value >= 0:
                ret_map[cur[0]][cur[1]] = mark_value
            prev_step = prev[cur[0]][cur[1]]
            next_step[prev_step[0]][prev_step[1]] = [cur[0], cur[1]]
            cur = prev_step
            if cur[0] == -1 and cur[1] == -1:
                break # no path possible

        # construct direction from start
        cur = self.start
        cur_dir = self.directon
        seq = []
        while cur[0] != self.goal[0] or cur[1] != self.goal[1]:
            next_coord = next_step[cur[0]][cur[1]]
            for x in self.action(cur, next_coord, cur_dir):
                seq.append(x)
            cur_dir = self.direction(cur, next_coord)
            cur = next_coord

        trim_seq = []
        ch_cnt = 0
        for ch in seq:
            if ch == '1':
                ch_cnt += 1
                if ch_cnt == 15:
                    trim_seq.append(str(hex(ch_cnt))[2:])
                    ch_cnt = 0
            else:
                if 0 < ch_cnt:
                    trim_seq.append(str(hex(ch_cnt))[2:])
                    ch_cnt = 0
                trim_seq.append(ch)
        if (ch_cnt > 0):
            trim_seq.append(str(hex(ch_cnt))[2:])


        # return sequence of actions ### and the map
        return {
            "sequence": seq,
            "trim_seq": trim_seq,
            "map": ret_map
        }


    """is_okay: detect the cells that are okay to move"""
    def is_safe(self, coord):
       
        # neither unexplored area or obstacle : explored cell
        directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for direction in directions:
            #mark out of range area and obstacle cells false
            if  coord[0] + direction[0] < 0 or coord[0] + direction[0] >= self.MAX_ROW or \
                coord[1] + direction[1] < 0 or coord[1] + direction[1] >= self.MAX_COL or \
                self.map[coord[0] + direction[0]][coord[1] + direction[1]] == 2: # obstacle             
                return False
        return True
   
    """expand: expand explored area"""
    def expand(self, head):
        directions = [[1, 0], [0, 1], [-1, 0], [0, -1]] #S,E,N,W
        ret = []
        for direction in directions:
            if 0 <= direction[0] + head[0] < len(self.map) and 0 <= direction[1] + head[1] < len (self.map[0]) and \
              self.is_safe([direction[0] + head[0], direction[1] + head[1]]):
                ret.append([direction[0] + head[0], direction[1] + head[1]])
        return ret
    
    """direction:decide direction"""
    def direction(self, _from, _to):
        if _to[0] - _from[0] > 0:
            return SOUTH
        elif _to[0] - _from[0] < 0:
            return NORTH
        else: #go EAST or WEST
            if _to[1] - _from[1] > 0:
                return EAST
            else:
                return WEST
    
    """action: decide the robot's actions based on the direction
       LEFT, RIGHT, FORWARD, LEFT X 2 = BACKWARD"""
    def action(self, _from, _to, _current_direction):
        # actions are: L (left), R (right), F (forward)
        if _current_direction == SOUTH:
            if _to[0] - _from[0] < 0: # from SOUTH to NORTH
                return [LEFT, LEFT, FORWARD]
            elif _to[0] - _from[0] == 0: # to EAST or WEST
                if _to[1] - _from[1] > 0: # to EAST
                    return [LEFT, FORWARD] 
                else: # to WEST
                    return [RIGHT, FORWARD]
        elif _current_direction == EAST:
            if _to[1] - _from[1] < 0: # from EAST to WEST
                return [LEFT, LEFT, FORWARD]
            elif _to[1] - _from[1] == 0: # go NORTH or SOUTH 
                if _to[0] - _from[0] > 0: # go SOUTH
                    return [RIGHT, FORWARD]
                else: # go NORTH
                    return [LEFT, FORWARD]
        elif _current_direction == NORTH:
            if _to[0] - _from[0] > 0: # from NORTH to SOUTH
                return [LEFT, LEFT, FORWARD]
            elif _to[0] - _from[0] == 0: # go EAST or WEST
                if _to[1] - _from[1] > 0: # go EAST
                    return [RIGHT, FORWARD]
                else: # go WEST
                    return [LEFT, FORWARD]
        elif _current_direction == WEST:
            if _to[1] - _from[1] > 0: # from WEST to EAST
                return [LEFT, LEFT, FORWARD]
            elif _to[1] - _from[1] == 0: # go NORTH or SOUTH
                if _to[0] - _from[0] > 0: # go SOUTH
                    return [LEFT, FORWARD]
                else: # go NORTH
                    return [RIGHT, FORWARD]
        return [FORWARD]
    
    """cost: find cost of the optimized path"""
    def cost(self, _from, _to, _current_direction):
        # can be the heuristic function
        # if going backward of current direction, cost = 9
        # if going to turn left or right, cost = 4
        # else cost = 1
        action = self.action(_from, _to, _current_direction)
        add_cost = 0

        directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for direction in directions:
            if self.map[_to[0] + direction[0]][_to[1] + direction[1]] == 0:
                add_cost += 30

        cost = 1 + max(0, (len(action) - 1) * (len(action) - 1) * 10) + add_cost
        return cost


