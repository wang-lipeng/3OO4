from algo.constants import *
import json
import zope.event
import time

class Robot(object):
    """docstring for Robot"""
    def __init__(self):
        super(Robot, self).__init__()
        self.explored_map = []
        self.map_state = []
        self.__map_state_changed = []
        self.__old_map = []
        self.__recolor_later = []

        self.MAX_ROW = 20
        self.MAX_COL = 15
        self.start = [18, 1]
        self.goal = [1, 13]

        self.try_left = False

        self.path_taken = []
        self.action_taken = []

        for i in range(self.MAX_ROW):
            self.explored_map.append([])
            self.map_state.append([])
            for j in range(self.MAX_COL):
                self.explored_map[i].append(0)
                self.map_state[i].append(0)

        directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for direction in directions:
            self.map_state[self.start[0] + direction[0]][self.start[1] + direction[1]] = 1
            self.map_state[self.goal[0] + direction[0]][self.goal[1] + direction[1]] = 1


        self.sensors = []
        for i in range(6):
            self.sensors.append([])
            for j in range(4):
                self.sensors[i].append(None)

        # Mark start and goal zone
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
        # now we're totally blind, we need to use explored_map!
        directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for direction in directions:
            if  _center[0] + direction[0] < 0 or _center[0] + direction[0] >= self.MAX_ROW or \
                _center[1] + direction[1] < 0 or _center[1] + direction[1] >= self.MAX_COL or \
                self.explored_map[_center[0] + direction[0]][_center[1] + direction[1]] == 2:
                # self.explored_map[_center[0] + direction[0]][_center[1] + direction[1]] == 1:
                return False
        return True

    def action(self, action, mark_value = 8):
        if action:
            if action == FORWARD or action.isdigit() or action.islower():
                times = int(action, 16)
                print("[Tornado | %s] real.py > action > %d " % (time.ctime(time.time()), times))
                for i in range(times):
                    self.action_taken.append(FORWARD)
                    self.forward(mark_value)
            elif action == LEFT or action == RIGHT:
                print("[Tornado | %s] real.py > action > %s " % (time.ctime(time.time()), action))
                self.action_taken.append(action)
                self.rotate(action)


        zope.event.notify(action)

    def forward(self, mark_value = 8):
        self.__clear_marks()
        self.__mark_surroundings(self.start, 6)
        self.__mark_surroundings(self.goal, 7)
        if mark_value >= 0:
            self.explored_map[self.current[0]][self.current[1]] = mark_value

        self.path_taken.append([self.current[0], self.current[1], mark_value])
        next_coords = [self.current[0], self.current[1]]
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

    def alignment(self):
        # TODO how to do alignment and only after that trust the sensor?


        # return []
        def is_okay(y, x):
            if 0 <= x < self.MAX_COL and 0 <= y < self.MAX_ROW:
                if self.explored_map[y][x] == 2:
                    return True
                return False
            else:
                return True

        # front
        if self.direction == NORTH:
            front = is_okay(self.current[0] - 2, self.current[1] - 1) and \
            is_okay(self.current[0] - 2, self.current[1]) and \
            is_okay(self.current[0] - 2, self.current[1] + 1)
        elif self.direction == EAST:
            front = is_okay(self.current[0] - 1 , self.current[1] + 2) and \
            is_okay(self.current[0], self.current[1] + 2) and \
            is_okay(self.current[0] + 1, self.current[1] + 2)
        elif self.direction == WEST:
            front = is_okay(self.current[0] + 1, self.current[1] - 2) and \
            is_okay(self.current[0], self.current[1] - 2) and \
            is_okay(self.current[0] - 1, self.current[1] - 2)
        else: # self.direction == SOUTH:
            front = is_okay(self.current[0] + 2, self.current[1] + 1) and \
            is_okay(self.current[0] + 2, self.current[1]) and \
            is_okay(self.current[0] + 2, self.current[1] - 1)

        # left
        if self.direction == NORTH:
            left = is_okay(self.current[0] - 1, self.current[1] - 2) and \
            is_okay(self.current[0], self.current[1] - 2) and \
            is_okay(self.current[0] + 1, self.current[1] - 2)
        elif self.direction == EAST:
            left = is_okay(self.current[0] - 2, self.current[1] + 1) and \
            is_okay(self.current[0] - 2, self.current[1]) and \
            is_okay(self.current[0] - 2, self.current[1] - 1)
        elif self.direction == WEST:
            left = is_okay(self.current[0] + 2, self.current[1] - 1) and \
            is_okay(self.current[0] + 2, self.current[1]) and \
            is_okay(self.current[0] + 2, self.current[1] + 1)
        else: # self.direction == SOUTH:
            left = is_okay(self.current[0] + 1, self.current[1] + 2) and \
            is_okay(self.current[0], self.current[1] + 2) and \
            is_okay(self.current[0] - 1, self.current[1] + 2)

        if front and left:
            return [FD_AL, LD_AL]#, RIGHT, LA_ALIGN]
        elif front:
            return [FA_AL, FD_AL]
        elif left:
            if self.try_left:
                self.try_left = False
                return [LD_AL]
            else:
                return [LA_AL]
        else:
            return []

    def parse_sensors(self, sensorString):
        def represent_float(s):
            try:
                float(s)
                return True
            except ValueError:
                return False
        def convert_short_sensor_distance(sensorValueStr):
            if represent_float(sensorValueStr):
                sensorValue = float(sensorValueStr)
                if ((sensorValue >=  0) and (sensorValue <  10)):
                    return [2, None, None, None]
                elif ((sensorValue >=  10) and (sensorValue <  20)):
                    return [1, 2, None, None]
                elif ((sensorValue >=  20) and (sensorValue <  30)):
                    return [1, 1, 2, None]
                elif ((sensorValue >=  30) and (sensorValue <  40)):
                    return [1, 1, 1, None]
                elif (sensorValue >=  40):
                    return [1, 1, 1, None]

        def convert_front_sensor(sensorValueStr):
            if represent_float(sensorValueStr):
                sensorValue = float(sensorValueStr)
                if ((sensorValue >=  0) and (sensorValue <  10)):
                    return [2, None, None, None]
                elif ((sensorValue >=  10) and (sensorValue <  20)):
                    return [1, 2, None, None]
                elif ((sensorValue >=  20) and (sensorValue <  25)):
                    return [1, 1, 2, None]
                else:
                    return [1, 1, None, None]
        def convert_left_sensor(sensorValueStr):
            if represent_float(sensorValueStr):
                sensorValue = float(sensorValueStr)
                if ((sensorValue >=  0) and (sensorValue <  10)):
                    return [2, None, None, None]
                elif ((sensorValue >=  10) and (sensorValue <  20)):
                    return [1, 2, None, None]
                elif ((sensorValue >=  20) and (sensorValue <  30)):
                    return [1, 1, 2, None]
                elif ((sensorValue >=  30) and (sensorValue <  35)):
                    return [1, 1, 1, 2]
                else:
                    return [1, 1, 1, None]
        def convert_right_sensor(sensorValueStr):
            if represent_float(sensorValueStr):
                sensorValue = float(sensorValueStr)
                if ((sensorValue >=  0) and (sensorValue <  10)):
                    return [2, None, None, None]
                elif ((sensorValue >=  10) and (sensorValue <  20)):
                    return [1, 2, None, None]
                elif ((sensorValue >=  20) and (sensorValue <  30)):
                    return [1, 1, 2, None]
                else:
                    return [1, 1, 1, None]

        sensors = []
        #for i in range(6):
        #    sensors.append([])
        #    for j in range(4):
        #       sensors[i].append(None)

        sensorList = sensorString.split(",")
        print("[Tornado | %s] real.py > sensorString > %s " %(time.ctime(time.time()), sensorString))

        # for i in range(6):
        #     sensors.append(convert_short_sensor_distance(sensorList[i]))
        sensors.append(convert_front_sensor(sensorList[0]))  # FL: max 25
        sensors.append(convert_front_sensor(sensorList[1]))  # FM: max 25
        sensors.append(convert_front_sensor(sensorList[2]))  # FR: max 25
        sensors.append(convert_left_sensor( sensorList[3]))  # LT: max 35
        sensors.append(convert_right_sensor(sensorList[4]))  # RT: max 30
        sensors.append(convert_left_sensor( sensorList[5]))  # LB: max 35

        if abs((float(sensorList[3]) + float(sensorList[5])) / 2.0 - 5.3) >= 0.5:
            self.try_left = True
        else:
            self.try_left = False


        self.sensors = sensors
        return sensors

    def update_map(self):
        self.__old_map = []
        self.__map_state_changed = []
        for i in range(self.MAX_ROW):
            self.__old_map.append([])
            self.__map_state_changed.append([])
            for j in range(self.MAX_COL):
                self.__old_map[i].append(self.explored_map[i][j])
                self.__map_state_changed[i].append("N")


        def upd(y, x, sensorValue):
            if sensorValue and \
            0 <= x < self.MAX_COL and\
            0 <= y < self.MAX_ROW and\
            self.explored_map[y][x] < 3:
                self.explored_map[y][x] = sensorValue
                self.__map_state_changed[y][x] = "C"

        # FL
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - i - 2, self.current[1] - 1, self.sensors[0][i])
            elif self.direction == EAST:
                upd(self.current[0] - 1 , self.current[1] + i + 2, self.sensors[0][i])
            elif self.direction == WEST:
                upd(self.current[0] + 1, self.current[1] - i - 2, self.sensors[0][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + i + 2, self.current[1] + 1, self.sensors[0][i])

        # FM
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - i - 2, self.current[1], self.sensors[1][i])
            elif self.direction == EAST:
                upd(self.current[0], self.current[1] + i + 2, self.sensors[1][i])
            elif self.direction == WEST:
                upd(self.current[0], self.current[1] - i - 2, self.sensors[1][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + i + 2, self.current[1], self.sensors[1][i])

        # FR
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - i - 2, self.current[1] + 1, self.sensors[2][i])
            elif self.direction == EAST:
                upd(self.current[0] + 1, self.current[1] + i + 2, self.sensors[2][i])
            elif self.direction == WEST:
                upd(self.current[0] - 1, self.current[1] - i - 2, self.sensors[2][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + i + 2, self.current[1] - 1, self.sensors[2][i])


        # LT
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - 1, self.current[1] - i - 2, self.sensors[3][i])
            elif self.direction == EAST:
                upd(self.current[0] - i - 2, self.current[1] + 1, self.sensors[3][i])
            elif self.direction == WEST:
                upd(self.current[0] + i + 2, self.current[1] - 1, self.sensors[3][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + 1, self.current[1] + i + 2, self.sensors[3][i])


        # RT
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - 1, self.current[1] + i + 2, self.sensors[4][i])
            elif self.direction == EAST:
                upd(self.current[0] + i + 2, self.current[1] + 1, self.sensors[4][i])
            elif self.direction == WEST:
                upd(self.current[0] - i - 2, self.current[1] - 1, self.sensors[4][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + 1, self.current[1] - i - 2, self.sensors[4][i])



        # LB
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] + 1, self.current[1] - i - 2, self.sensors[5][i])
            elif self.direction == EAST:
                upd(self.current[0] - i - 2, self.current[1] - 1, self.sensors[5][i])
            elif self.direction == WEST:
                upd(self.current[0] + i + 2, self.current[1] + 1, self.sensors[5][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] - 1, self.current[1] + i + 2, self.sensors[5][i])



        for i in range(self.MAX_ROW):
            for j in range(self.MAX_COL):
                if [5, 8, 9].count(self.explored_map[i][j]) > 0:
                    directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
                    for direction in directions:
                        if 0 <= i + direction[0] < self.MAX_ROW and 0 <= j + direction[1] < self.MAX_COL:
                            self.__map_state_changed[i + direction[0]][j + direction[1]] = "F"



        self.update_map_state()

        for coord in self.path_taken:
            self.explored_map[coord[0]][coord[1]] = coord[2]
            directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
            for direction in directions:
                if 0 <= j + direction[1] < self.MAX_COL and 0 <= i + direction[0] < self.MAX_ROW:
                    self.__map_state_changed[i + direction[0]][j + direction[1]] = "F"
        self.__mark_surroundings(self.start, 6)
        self.__mark_surroundings(self.goal, 7)
        self.__mark_robot()



        # TODO: if last four action is LEFT RIGHT LEFT RIGHT, mark sth as obstacle; at exploration.py DO NOT SET repeatedTreshold lower than 5!!!
        # TODO BUG!!!!: got two possibility of obstacle position!!!! one s seen by FL and LB; seen by FR and LT
        if self.action_taken[-4:] == [LEFT, RIGHT, LEFT, RIGHT]:
            print("[Tornado | %s] real.py > action > Something" %(time.ctime(time.time())))
            if self.direction == NORTH:
                self.explored_map[self.current[0] - 1][self.current[1] - 2] = 2
                self.map_state[self.current[0] - 1][self.current[1] - 2] = 4
            elif self.direction == EAST:
                self.explored_map[self.current[0] - 2][self.current[1] + 1] = 2
                self.map_state[self.current[0] - 2][self.current[1] + 1] = 4
            elif self.direction == SOUTH:
                self.explored_map[self.current[0] + 1][self.current[1] + 2] = 2
                self.map_state[self.current[0] + 1][self.current[1] + 2] = 4
            elif self.direction == WEST:
                self.explored_map[self.current[0] + 2][self.current[1] - 1] = 2
                self.map_state[self.current[0] + 2][self.current[1] - 1] = 4
            ## OR THIS CASE! HOW TO DIFFERENTIATE???
            # if self.direction == NORTH:
            #     self.explored_map[self.current[0] + 1][self.current[1] - 2] = 2
            #     self.map_state[self.current[0] + 1][self.current[1] - 2] = 4
            # elif self.direction == EAST:
            #     self.explored_map[self.current[0] - 2][self.current[1] - 1] = 2
            #     self.map_state[self.current[0] - 2][self.current[1] - 1] = 4
            # elif self.direction == SOUTH:
            #     self.explored_map[self.current[0] - 1][self.current[1] + 2] = 2
            #     self.map_state[self.current[0] - 1][self.current[1] + 2] = 4
            # elif self.direction == WEST:
            #     self.explored_map[self.current[0] + 2][self.current[1] + 1] = 2
            #     self.map_state[self.current[0] + 2][self.current[1] + 1] = 4


        print("[Tornado | %s] real.py > update_map > loc: %s, dir: %s " %(time.ctime(time.time()), self.current, self.direction))

        for i in range(self.MAX_ROW):
            print (self.explored_map[i])

        zope.event.notify("SENSOR")
        return self.explored_map


    def update_map_state(self):
        for i in range(self.MAX_ROW):
            for j in range(self.MAX_COL):

                if self.__map_state_changed[i][j] == "F":
                    #if self.map_state[i][j] == 2 or self.map_state[i][j] == 1:
                    self.map_state[i][j] = 1
                    self.explored_map[i][j] = 1
                    ### cause we will only put "F" is we ever been there, i.e. Forever explored
                elif self.__map_state_changed[i][j] == "C":
                    if self.explored_map[i][j] == 2 and self.__old_map[i][j] == 0:
                        self.map_state[i][j] = 3 #assign new state

                    elif self.explored_map[i][j] == 1 and self.__old_map[i][j] == 0:
                        self.map_state[i][j] = 2 #assign new state

                    elif self.explored_map[i][j] == 1 and self.__old_map[i][j] == 1 and self.map_state[i][j] == 2:
                        self.map_state[i][j] = 1 #(-1)

                    elif self.explored_map[i][j] == 1 and self.__old_map[i][j] == 1 and self.map_state[i][j] == 1:
                        self.map_state[i][j] = 1 #constant

                    elif self.explored_map[i][j] == 2 and self.__old_map[i][j] == 2 and self.map_state[i][j] == 3:
                        self.map_state[i][j] = 4 #(+1)

                    elif self.explored_map[i][j] == 2 and self.__old_map[i][j] == 2 and self.map_state[i][j] == 4:
                        self.map_state[i][j] = 4 #constant

                    elif self.explored_map[i][j] == 2 and self.__old_map[i][j] == 1 and self.map_state[i][j] == 1:
                        self.map_state[i][j] = self.map_state[i][j] + 1 #changed in state but current state at extreme end

                    elif self.explored_map[i][j] == 1 and self.__old_map[i][j] == 2 and self.map_state[i][j] == 4:
                        self.map_state[i][j] = self.map_state[i][j] - 1 #changed in state but current state at extreme end

                    elif self.explored_map[i][j] == 2 and self.__old_map[i][j] == 1 and self.map_state[i][j] == 2:
                        self.map_state[i][j] = self.map_state[i][j] + 1 #changed in state and current state in middle
                        self.explored_map[i][j] = 2

                    elif self.explored_map[i][j] == 1 and self.__old_map[i][j] == 2 and self.map_state[i][j] == 3:
                        self.map_state[i][j] = self.map_state[i][j] - 1 #changed in state and current state in middle
                        self.explored_map[i][j] = 1

        # print("explored_map:")
        # for r in self.explored_map:
        #     print(r)
        # print("map_state:")
        # for r in self.map_state:
        #     print(r)
        # print("__map_state_changed:")
        # for r in self.__map_state_changed:
        #     print(r)
        # print("__old_map:")
        # for r in self.__old_map:
        #     print(r)

        return self.map_state

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

        # print(ret)
        # print(len(ret))
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

        # print(hex_ret)
        # print(len(hex_ret))

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

        # print(ret)
        # print(len(ret))
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

        # print(hex_ret)
        # print(len(hex_ret))

        return ''.join([h for h in hex_ret])

    def msg_for_android(self):
        # descriptor
        # 1: obstacle
        # 0: otherwise
        # return in hex
        # in JSON
        #
        # robot coordinate
        # X, Y;
        # in Android, map is 15x20 (row x col); (0, 0) is top left;
        # robot is placed according to robot body's top-left corner (i.e. (0, 0) is meant to be on top-left)
        #

        ret = {"grid": "", "coordinate": dict(), "direction": ""}



        transformed_map = []
        for i in range(self.MAX_COL):
            transformed_map.append([])
            for j in range(self.MAX_ROW):
                transformed_map[i].append(0)

        col = 0
        for i in range(self.MAX_ROW - 1, -1, -1):
            for j in range(self.MAX_COL):
                transformed_map[j][col] = self.explored_map[i][j]
            col += 1
        # print(transformed_map)

        grid_seq = []
        cnt = 0
        for row in reversed(transformed_map):
            for col in row:
                cnt += 1
                if col == 2:
                    grid_seq.append(1)
                else:
                    grid_seq.append(0)
        while cnt % 8 != 0:
            grid_seq.append(0)
            cnt += 1

        hex_val = []
        temp = []
        for bit in grid_seq:
            if len(temp) < 4:
                temp.append(bit)
            else:
                temp_str = ''.join([str(b) for b in temp])
                hex_val.append(str(hex(int(temp_str, 2)))[2:])
                temp = [bit]
        if len(temp) > 0:
            temp_str = ''.join([str(b) for b in temp])
            hex_val.append(str(hex(int(temp_str, 2)))[2:])

        ret['grid'] = ''.join([h for h in hex_val])

        coord = [0, 0]
        for i in range(self.MAX_COL):
            for j in range(self.MAX_ROW):
                if transformed_map[i][j] == 5:
                    coord = [i, j]
                    break

        ret['coordinate']['y'] = coord[0] - 1
        ret['coordinate']['x'] = coord[1] - 1

        direction = EAST
        if self.direction == NORTH:
            direction = EAST
        elif self.direction == EAST:
            direction = SOUTH
        elif self.direction == SOUTH:
            direction = WEST
        else: # self.direction == WEST:
            direction = NORTH

        ret['direction'] = direction

        return json.dumps(ret)
