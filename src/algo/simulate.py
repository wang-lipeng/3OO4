"""
0: unexplored
1: explored
2: obstacle
3: robot body
4: robot head
5: robot center
6: start
7: goal
8: explored path
9: optimum path

"""
from algo.constants import *
import zope.event
class Robot(object):
    """docstring for Robot"""
    def __init__(self):
        super(Robot, self).__init__()
        self.__map = []
        self.explored_map = []
        self.__get_map()
        # self.descriptor_map_one = [row[:] for row in self.explored_map]
        # self.descriptor_map_two = [row[:] for row in self.explored_map]
        
        self.MAX_ROW = len(self.__map)
        self.MAX_COL = len(self.__map[0])
        self.__recolor_later = []

        self.start = [18, 1] # find_centre(start)
        self.goal = [1, 13] # find_centre(goal)

        # mark start & goal area
        self.__mark_surroundings(self.start, 6)
        self.__mark_surroundings(self.goal, 7)

        self.current = [18, 1] # find_centre(start)
        
        self.direction = NORTH # ODO: need to face NORTH in order to work!
        self.__mark_robot()


        zope.event.notify("INIT")

    def __mark_surroundings(self, _center, _value):
        directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for direction in directions:
            current_value = self.explored_map[_center[0] + direction[0]][_center[1] + direction[1]]
            if current_value == 1 or 6 <= current_value <= 9:
                self.__recolor_later.append({
                    "coord": [_center[0] + direction[0], _center[1] + direction[1]],
                    "value": current_value
                })

            self.explored_map[_center[0] + direction[0]][_center[1] + direction[1]] = _value
    
    def __mark_robot(self):
        self.__mark_surroundings(self.current, 3)
        self.explored_map[self.current[0]][self.current[1]] = 5
        if self.direction == NORTH:
            self.explored_map[self.current[0] - 1][self.current[1]] = 4
        elif self.direction == EAST:
            self.explored_map[self.current[0]][self.current[1] + 1] = 4
        elif self.direction == WEST:
            self.explored_map[self.current[0]][self.current[1] - 1] = 4
        else: # self.direction == SOUTH
            self.explored_map[self.current[0] + 1][self.current[1]] = 4
    def __clear_marks(self):
        # need to save original marks (e.g. 1, 6, 7, 8, 9)
        for o in self.__recolor_later:
            self.explored_map[o['coord'][0]][o['coord'][1]] = o['value']
        self.__recolor_later = []

    def __is_safe(self, _center):
        directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for direction in directions:
            if  _center[0] + direction[0] < 0 or _center[0] + direction[0] >= self.MAX_ROW or \
                _center[1] + direction[1] < 0 or _center[1] + direction[1] >= self.MAX_COL or \
                self.__map[_center[0] + direction[0]][_center[1] + direction[1]] == 2:
                # self.__map[_center[0] + direction[0]][_center[1] + direction[1]] == 1:
                return False
        return True

    def __get_map(self):
        start = []
        goal = []
        with open("map.txt") as f:
            content = f.readlines()
            for line in content:
                temp = []
                temp0 = []
                for char in line:
                    if char.isdigit():
                        temp.append(int(char))
                        temp0.append(0)
                self.__map.append(temp)
                # self.explored_map.append(temp)
                self.explored_map.append(temp0)

    def action(self, action, mark_value = 8):
        if action:
            if action == FORWARD or action.isdigit() or action.islower():
                times = int(action, 16)
                print("[Tornado] sim.py > action > %d " % (times))
                for i in range(times):
                    self.forward(mark_value)
            elif action == LEFT or action == RIGHT:
                self.rotate(action)
        zope.event.notify(action)

    def forward(self, mark_value = 8):
        self.__clear_marks()
        self.__mark_surroundings(self.start, 6)
        self.__mark_surroundings(self.goal, 7)
        if mark_value >= 0:
            self.explored_map[self.current[0]][self.current[1]] = mark_value

        next_coords = []
        next_coords.append(self.current[0])
        next_coords.append(self.current[1])
        if self.direction == NORTH:
            next_coords[0] += -1
        elif self.direction == EAST:
            next_coords[1] += 1
        elif self.direction == WEST:
            next_coords[1] += -1
        else: # self.direction == SOUTH
            next_coords[0] += 1
        if 0 <= next_coords[0] < self.MAX_ROW and 0 <= next_coords[1] < self.MAX_COL and self.__is_safe(next_coords):
            self.current[0] = next_coords[0]
            self.current[1] = next_coords[1]
        self.__mark_robot()

    def rotate(self, direction):
        if direction == RIGHT: # clockwise
            if self.direction == NORTH:
                self.direction = EAST
            elif self.direction == EAST:
                self.direction = SOUTH
            elif self.direction == SOUTH:
                self.direction = WEST
            elif self.direction == WEST:
                self.direction = NORTH
        else: # Counter-clockwise
            if self.direction == NORTH:
                self.direction = WEST
            elif self.direction == EAST:
                self.direction = NORTH
            elif self.direction == SOUTH:
                self.direction = EAST
            elif self.direction == WEST:
                self.direction = SOUTH
        self.__mark_robot()

    def __get_value(self, y, x):
        if 0 <= y < self.MAX_ROW and 0 <= x < self.MAX_COL:
            # TODO: if false information is given, how to update?
            if self.__map[y][x] == 2:
                if self.explored_map[y][x] == 0:
                    self.explored_map[y][x] = 2
                return 2
            else:
                if self.explored_map[y][x] == 0:
                    self.explored_map[y][x] = 1
                return 1
        return None

    def get_sensors(self):
        sensors = []
        for i in range(5):
            sensors.append([])
            for j in range(4):
                sensors[i].append(None)

        # FL
        for i in range(4):
            if self.direction == NORTH:
                sensors[0][i] = self.__get_value(self.current[0] - i - 2, self.current[1] - 1)
            elif self.direction == EAST:
                sensors[0][i] = self.__get_value(self.current[0] - 1 , self.current[1] + i + 2)
            elif self.direction == WEST:
                sensors[0][i] = self.__get_value(self.current[0] + 1, self.current[1] - i - 2)
            else: # self.direction == SOUTH:
                sensors[0][i] = self.__get_value(self.current[0] + i + 2, self.current[1] + 1)

            if sensors[0][i] == 2:
                break
        # FM
        for i in range(4):
            if self.direction == NORTH:
                sensors[1][i] = self.__get_value(self.current[0] - i - 2, self.current[1])
            elif self.direction == EAST:
                sensors[1][i] = self.__get_value(self.current[0], self.current[1] + i + 2)
            elif self.direction == WEST:
                sensors[1][i] = self.__get_value(self.current[0], self.current[1] - i - 2)
            else: # self.direction == SOUTH:
                sensors[1][i] = self.__get_value(self.current[0] + i + 2, self.current[1])

            if sensors[1][i] == 2:
                break
        # FR
        for i in range(4):
            if self.direction == NORTH:
                sensors[2][i] = self.__get_value(self.current[0] - i - 2, self.current[1] + 1)
            elif self.direction == EAST:
                sensors[2][i] = self.__get_value(self.current[0] + 1, self.current[1] + i + 2)
            elif self.direction == WEST:
                sensors[2][i] = self.__get_value(self.current[0] - 1, self.current[1] - i - 2)
            else: # self.direction == SOUTH:
                sensors[2][i] = self.__get_value(self.current[0] + i + 2, self.current[1] - 1)

            if sensors[2][i] == 2:
                break
        
        # LT
        for i in range(4):
            if self.direction == NORTH:
                sensors[3][i] = self.__get_value(self.current[0] - 1, self.current[1] - i - 2)
            elif self.direction == EAST:
                sensors[3][i] = self.__get_value(self.current[0] - i - 2, self.current[1] + 1)
            elif self.direction == WEST:
                sensors[3][i] = self.__get_value(self.current[0] + i + 2, self.current[1] - 1)
            else: # self.direction == SOUTH:
                sensors[3][i] = self.__get_value(self.current[0] + 1, self.current[1] + i + 2)

            # sensors[3][i] = self.__get_value(self.current[0] - 1, self.current[1] - i - 2)
            if sensors[3][i] == 2:
                break

        # RT
        for i in range(4):
            if self.direction == NORTH:
                sensors[4][i] = self.__get_value(self.current[0] - 1, self.current[1] + i + 2)
            elif self.direction == EAST:
                sensors[4][i] = self.__get_value(self.current[0] + i + 2, self.current[1] + 1)
            elif self.direction == WEST:
                sensors[4][i] = self.__get_value(self.current[0] - i - 2, self.current[1] - 1)
            else: # self.direction == SOUTH:
                sensors[4][i] = self.__get_value(self.current[0] + 1, self.current[1] - i - 2)

            # sensors[4][i] = self.__get_value(self.current[0] - 1, self.current[1] + i + 2)
            if sensors[4][i] == 2:
                break
        
        # LB
#         for i in range(4):
#             if self.direction == NORTH:
#                 sensors[5][i] = self.__get_value(self.current[0] + 1, self.current[1] - i - 2)
#             elif self.direction == EAST:
#                 sensors[5][i] = self.__get_value(self.current[0] - i - 2, self.current[1] - 1)
#             elif self.direction == WEST:
#                 sensors[5][i] = self.__get_value(self.current[0] + i + 2, self.current[1] + 1)
#             else: # self.direction == SOUTH:
#                 sensors[5][i] = self.__get_value(self.current[0] - 1, self.current[1] + i + 2)
# 
#             #sensors[5][i] = self.__get_value(self.current[0] + 1, self.current[1] - i - 2)
#             if sensors[5][i] == 2:
#                 break
        zope.event.notify("SENSOR")
        return sensors

    def descriptor_one(self):
        ret = [1, 1]
        for row in reversed(self.explored_map):
            for col in row:
                if col > 0:
                    ret.append(1)
                else:
                    ret.append(0)
        ret.append(1)
        ret.append(1)

#         print(ret)
#         print(len(ret))
        hex_ret = []
        temp = []
        for bit in ret:
            if len(temp) < 4:
                temp.append(bit)
            else:
                temp_str = ''.join([str(b) for b in temp])
                hex_ret.append(str(hex(int(temp_str, 2)))[2:])
                temp = [bit]
        if len(temp) > 0:
            temp_str = ''.join([str(b) for b in temp])
            hex_ret.append(str(hex(int(temp_str, 2)))[2:])

#         print(hex_ret)
#         print(len(hex_ret))

        return ''.join([h for h in hex_ret])

    def descriptor_two(self):
        ret = []
        cnt = 0
        for row in reversed(self.explored_map):
            for col in row:
                if col > 0:
                    cnt += 1
                    if col == 2:
                        ret.append(1)
                    else:
                        ret.append(0)
        while cnt % 8 != 0:
            ret.append(0)
            cnt += 1

#         print(ret)
#         print(len(ret))
        hex_ret = []
        temp = []
        for bit in ret:
            if len(temp) < 4:
                temp.append(bit)
            else:
                temp_str = ''.join([str(b) for b in temp])
                hex_ret.append(str(hex(int(temp_str, 2)))[2:])
                temp = [bit]
        if len(temp) > 0:
            temp_str = ''.join([str(b) for b in temp])
            hex_ret.append(str(hex(int(temp_str, 2)))[2:])
        
        # Print the hex value #
#         print(hex_ret)
#         print(len(hex_ret))

        return ''.join([h for h in hex_ret])

if __name__ == '__main__':
    robot = Robot()
    print(robot.get_sensors())
    print(robot.explored_map)
