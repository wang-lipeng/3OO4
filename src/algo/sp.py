'''
Created on 12 Feb 2016

@author: Bridge
'''
import Queue
"""
- 0: unexplored
- 1: explored
- 2: obstacle
- 3: robot body
- 4: robot head
- 5: robot center
- 6: start
- 7: goal
- 8: explored path
- 9: optimum path
"""
from Queue import PriorityQueue
from algo.constants import *

class PqNode(object):
    def __init__(self, _dict):
        self.weight = _dict["weight"]
        self.direction = _dict["direction"]
        self.position = _dict["position"]
    def __lt__(self, rhs):
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

    def is_okay(self, coord):
        # return self.map[coord[0]][coord[1]] != 0 and self.map[coord[0]][coord[1]] != 2
        directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for direction in directions:
            if  coord[0] + direction[0] < 0 or coord[0] + direction[0] >= self.MAX_ROW or \
                coord[1] + direction[1] < 0 or coord[1] + direction[1] >= self.MAX_COL or \
                self.map[coord[0] + direction[0]][coord[1] + direction[1]] == 2: # obstacle
                # self.map[coord[0] + direction[0]][coord[1] + direction[1]] == 0 or \ # unexplored
                return False
        return True

    def expand(self, head):
        directions = [[1, 0], [0, 1], [-1, 0], [0, -1]]
        ret = []
        for direction in directions:
            if 0 <= direction[0] + head[0] < len(self.map) and 0 <= direction[1] + head[1] < len (self.map[0]) and  self.is_okay([direction[0] + head[0], direction[1] + head[1]]):
                ret.append([direction[0] + head[0], direction[1] + head[1]])
        return ret
    def direction(self, _from, _to):
        if _to[0] - _from[0] > 0:
            return SOUTH
        elif _to[0] - _from[0] < 0:
            return NORTH
        else:
            if _to[1] - _from[1] > 0:
                return EAST
            else:
                return WEST
    def action(self, _from, _to, _current_direction):
        # actions are: L (left), R (right), F (forward)
        if _current_direction == SOUTH:
            if _to[0] - _from[0] < 0:
                return [LEFT, LEFT, FORWARD]
            elif _to[0] - _from[0] == 0:
                if _to[1] - _from[1] > 0:
                    return [LEFT, FORWARD]
                else:
                    return [RIGHT, FORWARD]
        elif _current_direction == EAST:
            if _to[1] - _from[1] < 0:
                return [LEFT, LEFT, FORWARD]
            elif _to[1] - _from[1] == 0:
                if _to[0] - _from[0] > 0:
                    return [RIGHT, FORWARD]
                else:
                    return [LEFT, FORWARD]
        elif _current_direction == NORTH:
            if _to[0] - _from[0] > 0:
                return [LEFT, LEFT, FORWARD]
            elif _to[0] - _from[0] == 0:
                if _to[1] - _from[1] > 0:
                    return [RIGHT, FORWARD]
                else:
                    return [LEFT, FORWARD]
        elif _current_direction == WEST:
            if _to[1] - _from[1] > 0:
                return [LEFT, LEFT, FORWARD]
            elif _to[1] - _from[1] == 0:
                if _to[0] - _from[0] > 0:
                    return [LEFT, FORWARD]
                else:
                    return [RIGHT, FORWARD]
        return [FORWARD]
    def manhattan(self, _from, _to):
        return abs(_from[0] - _to[0]) + abs(_from[1] - _to[1])

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

        ret_cost = 1 + max(0, (len(action) - 1) * (len(action) - 1) * 10) + add_cost
        return ret_cost

    def shortest_path(self, mark_value = 9):
        dist = [] # for each cell, how far is it from the start?
        prev = [] # for each cell, what is its parent?
        next_post = [] # for each cell in optimized path, what is the next position?
        for i in range(len(self.map)):
            dist.append([])
            prev.append([])
            next_post.append([])
            for j in range(len(self.map[i])):
                dist[i].append(INF)
                prev[i].append([-1, -1])
                next_post[i].append([-1, -1])
        # note that it will be (y, x)

        # do Dijkstra/UCS or A*
        pq = PriorityQueue()
        pq.put(PqNode({"position": self.start, "direction": self.direction, "weight": 0}))
        dist[self.start[0]][self.start[1]] = 0

        while not pq.empty():
            head = pq.get()

            # if goal, break
            if head["position"][0] == self.goal[0] and head["position"][1] == self.goal[1]:
                break

            # expand head
            neighbors =  self.expand(head["position"])

            # if not yet visited OR can be visited with lower cost, put in pq
            for neighbor in neighbors:
                gn = self.cost(head["position"], neighbor, head["direction"])
                hn = self.manhattan(neighbor, self.goal)
                cost = gn# + hn
                if dist[head["position"][0]][head["position"][1]] + cost < dist[neighbor[0]][neighbor[1]]:

                    dist[neighbor[0]][neighbor[1]] = dist[head["position"][0]][head["position"][1]] + gn
                    prev[neighbor[0]][neighbor[1]] = head["position"]
                    #print(neighbor, " from ", head["position"], " so far: ", dist[neighbor[0]][neighbor[1]])
                    pq.put(PqNode({"position": neighbor,
                        "direction": self.direction(head["position"], neighbor),
                        "weight": cost}))

        # construct path from goal
        cur = self.goal
        ret_map = self.map
        while cur[0] != self.start[0] or cur[1] != self.start[1]:
            if mark_value >= 0:
                ret_map[cur[0]][cur[1]] = mark_value
            prev_post = prev[cur[0]][cur[1]]
            next_post[prev_post[0]][prev_post[1]] = [cur[0], cur[1]]
            cur = prev_post
            if cur[0] == -1 and cur[1] == -1:
                break # no path possible

        # construct direction from start
        cur = self.start
        cur_dir = self.directon
        seq = []
        while cur[0] != self.goal[0] or cur[1] != self.goal[1]:
            next_coord = next_post[cur[0]][cur[1]]
            # print(cur, next_coord, cur_dir)
            for x in self.action(cur, next_coord, cur_dir):
                seq.append(x)
                # print(x)
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
        #print(trim_seq)

        # return sequence of actions ### and the map
        return {
            "sequence": seq,
            "trim_seq": trim_seq#,
        #    "map": ret_map
        }
