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
import json
import zope.event
import time

class Robot(object):
    def __init__(self):
        super(Robot, self).__init__()
        """initiate variables"""
        self.explored_map = []
        self.map_state = []
        self.map_state_changed = []
        self.map = []
        self.to_change_color = []
        self.MAX_ROW = 20
        self.MAX_COL = 15
        self.start = [18, 1]
        self.goal = [1, 13]
        self.current = [18, 1] # center at start zone
        self.path_taken = []
        self.steps = []        
        self.try_left = False
        self.direction = NORTH 
               
        for i in range(self.MAX_ROW):
            self.explored_map.append([])
            self.map_state.append([])
            for j in range(self.MAX_COL):
                self.explored_map[i].append(0)
                self.map_state[i].append(0)

        """mark start and goal zone as explored"""
        matrix = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for pos in matrix:
            self.map_state[self.start[0] + pos[0]][self.start[1] + pos[1]] = 1
            self.map_state[self.goal[0] + pos[0]][self.goal[1] + pos[1]] = 1

        """sensor list"""
        self.sensors = []
        for i in range(6):
            self.sensors.append([])
            for j in range(4):
                self.sensors[i].append(None)

        """mark start and goal zone"""
        self.mark_body(self.start, 6)
        self.mark_body(self.goal, 7)
        """mark robot body cells (body, head, center)"""
        self.mark_robot()
        
               
        zope.event.notify("INIT")

    def mark_body(self, center, val):
        """mark value for 9 cells & change color based on current center value"""
        matrix = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for pos in matrix:
            current_value = self.explored_map[center[0] + pos[0]][center[1] + pos[1]]
            if current_value == 1 or 6 <= current_value <= 9:
                self.to_change_color.append({
                    "coordinate": [center[0] + pos[0], center[1] + pos[1]],
                    "value": current_value
                })
            self.explored_map[center[0] + pos[0]][center[1] + pos[1]] = val

    def mark_robot(self):
        """mark value for robot body"""
        self.mark_body(self.current, 3)
        """mark value for robot center and head"""
        self.explored_map[self.current[0]][self.current[1]] = 5
        if self.direction == NORTH:
            self.explored_map[self.current[0] - 1][self.current[1]] = 4
        elif self.direction == EAST:
            self.explored_map[self.current[0]][self.current[1] + 1] = 4
        elif self.direction == WEST:
            self.explored_map[self.current[0]][self.current[1] - 1] = 4
        else: # self.direction == SOUTH
            self.explored_map[self.current[0] + 1][self.current[1]] = 4
    
    def format_marks(self):
        """update explored map"""
        for cell in self.to_change_color:
            self.explored_map[cell['coordinate'][0]][cell['coordinate'][1]] = cell['value']
        """clear change color cell list"""
        self.to_change_color = []

    def isOkay(self, center):
        """not out of map or is obstacle"""
        matrix = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for pos in matrix:
            if  center[0] + pos[0] < 0 or center[0] + pos[0] >= self.MAX_ROW or \
                center[1] + pos[1] < 0 or center[1] + pos[1] >= self.MAX_COL or \
                self.explored_map[center[0] + pos[0]][center[1] + pos[1]] == 2:
                return False
        return True

    def step(self, step, val = 8):
        if step:
            if step == FORWARD or step.isdigit() or step.islower():
                times = int(step, 16)              
                for i in range(times):
                    self.steps.append(FORWARD)
                    self.forward(val)
            elif step == LEFT or step == RIGHT:               
                self.steps.append(step)
                self.rotate(step)   
        zope.event.notify(step)

    def forward(self, val = 8):
        self.format_marks()
        self.mark_body(self.start, 6)
        self.mark_body(self.goal, 7)
        if val >= 0:
            self.explored_map[self.current[0]][self.current[1]] = val

        self.path_taken.append([self.current[0], self.current[1], val])
        next_step = [self.current[0], self.current[1]]
        """decide next step coordinates"""
        if self.direction == NORTH:
            next_step[0] += -1
        elif self.direction == EAST:
            next_step[1] += 1
        elif self.direction == WEST:
            next_step[1] += -1
        else: # self.direction == SOUTH
            next_step[0] += 1
        """check next step is within the map and okay"""
        if 0 <= next_step[0] < self.MAX_ROW and 0 <= next_step[1] < self.MAX_COL and self.isOkay(next_step):
            self.current[0] = next_step[0]
            self.current[1] = next_step[1]
        self.mark_robot()

    def rotate(self, direction):
        """turn robot 90 degrees clockwise"""
        if direction == RIGHT: 
            if self.direction == NORTH:
                self.direction = EAST
            elif self.direction == EAST:
                self.direction = SOUTH
            elif self.direction == SOUTH:
                self.direction = WEST
            elif self.direction == WEST:
                self.direction = NORTH
        else: 
            """turn robot 90 degrees counter-clockwise""" 
            if self.direction == NORTH:
                self.direction = WEST
            elif self.direction == EAST:
                self.direction = NORTH
            elif self.direction == SOUTH:
                self.direction = EAST
            elif self.direction == WEST:
                self.direction = SOUTH
        self.mark_robot()
#
    def alignment(self):
        def is_danger(y, x):
            """within map range""" 
            if 0 <= x < self.MAX_COL and 0 <= y < self.MAX_ROW:
                if self.explored_map[y][x] == 2: #encounter obstacle 
                    return True
                return False
            else:
                return True

        """check front side""" 
        if self.direction == NORTH:
            front = is_danger(self.current[0] - 2, self.current[1] - 1) and \
            is_danger(self.current[0] - 2, self.current[1]) and \
            is_danger(self.current[0] - 2, self.current[1] + 1)
        elif self.direction == EAST:
            front = is_danger(self.current[0] - 1 , self.current[1] + 2) and \
            is_danger(self.current[0], self.current[1] + 2) and \
            is_danger(self.current[0] + 1, self.current[1] + 2)
        elif self.direction == WEST:
            front = is_danger(self.current[0] + 1, self.current[1] - 2) and \
            is_danger(self.current[0], self.current[1] - 2) and \
            is_danger(self.current[0] - 1, self.current[1] - 2)
        else: # self.direction == SOUTH:
            front = is_danger(self.current[0] + 2, self.current[1] + 1) and \
            is_danger(self.current[0] + 2, self.current[1]) and \
            is_danger(self.current[0] + 2, self.current[1] - 1)

        """check left side""" 
        if self.direction == NORTH:
            left = is_danger(self.current[0] - 1, self.current[1] - 2) and \
            is_danger(self.current[0], self.current[1] - 2) and \
            is_danger(self.current[0] + 1, self.current[1] - 2)
        elif self.direction == EAST:
            left = is_danger(self.current[0] - 2, self.current[1] + 1) and \
            is_danger(self.current[0] - 2, self.current[1]) and \
            is_danger(self.current[0] - 2, self.current[1] - 1)
        elif self.direction == WEST:
            left = is_danger(self.current[0] + 2, self.current[1] - 1) and \
            is_danger(self.current[0] + 2, self.current[1]) and \
            is_danger(self.current[0] + 2, self.current[1] + 1)
        else: # self.direction == SOUTH:
            left = is_danger(self.current[0] + 1, self.current[1] + 2) and \
            is_danger(self.current[0], self.current[1] + 2) and \
            is_danger(self.current[0] - 1, self.current[1] + 2)

        """front and left blocked""" 
        if front and left:
            return [FD_AL, LD_AL]
        elif front: #front blocked
            return [FA_AL, FD_AL]
        elif left: #left blocked
            if self.try_left:
                self.try_left = False
                return [LD_AL]
            else:
                return [LA_AL]
        else:
            return []

    """receive sensor values""" 
    def receive_sensors(self, sensorValStr):
        def is_float(s):
            try:
                float(s)
                return True
            except ValueError:
                return False
        def convert_short_sensor_distance(str):
            if is_float(str):
                sensorVal = float(str)
                if ((sensorVal >=  0) and (sensorVal <  10)):
                    return [2, None, None, None]
                elif ((sensorVal >=  10) and (sensorVal <  20)):
                    return [1, 2, None, None]
                elif ((sensorVal >=  20) and (sensorVal <  30)):
                    return [1, 1, 2, None]
                elif ((sensorVal >=  30) and (sensorVal <  40)):
                    return [1, 1, 1, None]
                elif (sensorVal >=  40):
                    return [1, 1, 1, None]

        def convert_front_sensor(str):
            if is_float(str):
                sensorVal = float(str)
                if ((sensorVal >=  0) and (sensorVal <  10)):
                    return [2, None, None, None]
                elif ((sensorVal >=  10) and (sensorVal <  20)):
                    return [1, 2, None, None]
                elif ((sensorVal >=  20) and (sensorVal <  25)):
                    return [1, 1, 2, None]
                else:
                    return [1, 1, None, None]
                
        def convert_left_sensor(str):
            if is_float(str):
                sensorVal = float(str)
                if ((sensorVal >=  0) and (sensorVal <  10)):
                    return [2, None, None, None]
                elif ((sensorVal >=  10) and (sensorVal <  20)):
                    return [1, 2, None, None]
                elif ((sensorVal >=  20) and (sensorVal <  30)):
                    return [1, 1, 2, None]
                elif ((sensorVal >=  30) and (sensorVal <  35)):
                    return [1, 1, 1, 2]
                else:
                    return [1, 1, 1, None]
                
        def convert_right_sensor(str):
            if is_float(str):
                sensorVal = float(str)
                if ((sensorVal >=  0) and (sensorVal <  10)):
                    return [2, None, None, None]
                elif ((sensorVal >=  10) and (sensorVal <  20)):
                    return [1, 2, None, None]
                elif ((sensorVal >=  20) and (sensorVal <  30)):
                    return [1, 1, 2, None]
                else:
                    return [1, 1, 1, None]

        sensors = []
        sensorList = sensorValStr.split(",")
        
        """append converted sensor values""" 
        sensors.append(convert_front_sensor(sensorList[0]))  # FL: max 25
        sensors.append(convert_front_sensor(sensorList[1]))  # FM: max 25
        sensors.append(convert_front_sensor(sensorList[2]))  # FR: max 25
        sensors.append(convert_left_sensor( sensorList[3]))  # LT: max 35
        sensors.append(convert_right_sensor(sensorList[4]))  # RT: max 30
#         sensors.append(convert_left_sensor( sensorList[5]))  # LB: max 35

        if abs((float(sensorList[3]) + float(sensorList[5])) / 2.0 - 5.3) >= 0.5:
            self.try_left = True
        else:
            self.try_left = False

        self.sensors = sensors
        return sensors

    def update_map(self):
        self.map = []
        self.map_state_changed = []
        for i in range(self.MAX_ROW):
            self.map.append([])
            self.map_state_changed.append([])
            for j in range(self.MAX_COL):
                self.map[i].append(self.explored_map[i][j])
                self.map_state_changed[i].append("N")

        def upd(y, x, sensorVal):
            if sensorVal and \
            0 <= x < self.MAX_COL and\
            0 <= y < self.MAX_ROW and\
            self.explored_map[y][x] < 3:
                self.explored_map[y][x] = sensorVal
                self.map_state_changed[y][x] = "C"

        """" front left"""
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - i - 2, self.current[1] - 1, self.sensors[0][i])
            elif self.direction == EAST:
                upd(self.current[0] - 1 , self.current[1] + i + 2, self.sensors[0][i])
            elif self.direction == WEST:
                upd(self.current[0] + 1, self.current[1] - i - 2, self.sensors[0][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + i + 2, self.current[1] + 1, self.sensors[0][i])

        """" front center"""
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - i - 2, self.current[1], self.sensors[1][i])
            elif self.direction == EAST:
                upd(self.current[0], self.current[1] + i + 2, self.sensors[1][i])
            elif self.direction == WEST:
                upd(self.current[0], self.current[1] - i - 2, self.sensors[1][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + i + 2, self.current[1], self.sensors[1][i])

        """" front right"""
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - i - 2, self.current[1] + 1, self.sensors[2][i])
            elif self.direction == EAST:
                upd(self.current[0] + 1, self.current[1] + i + 2, self.sensors[2][i])
            elif self.direction == WEST:
                upd(self.current[0] - 1, self.current[1] - i - 2, self.sensors[2][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + i + 2, self.current[1] - 1, self.sensors[2][i])

        """" left"""
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - 1, self.current[1] - i - 2, self.sensors[3][i])
            elif self.direction == EAST:
                upd(self.current[0] - i - 2, self.current[1] + 1, self.sensors[3][i])
            elif self.direction == WEST:
                upd(self.current[0] + i + 2, self.current[1] - 1, self.sensors[3][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + 1, self.current[1] + i + 2, self.sensors[3][i])

        """" right"""
        for i in range(4):
            if self.direction == NORTH:
                upd(self.current[0] - 1, self.current[1] + i + 2, self.sensors[4][i])
            elif self.direction == EAST:
                upd(self.current[0] + i + 2, self.current[1] + 1, self.sensors[4][i])
            elif self.direction == WEST:
                upd(self.current[0] - i - 2, self.current[1] - 1, self.sensors[4][i])
            else: # self.direction == SOUTH:
                upd(self.current[0] + 1, self.current[1] - i - 2, self.sensors[4][i])
# 
#         """" left bottom"""
#         for i in range(4):
#             if self.direction == NORTH:
#                 upd(self.current[0] + 1, self.current[1] - i - 2, self.sensors[5][i])
#             elif self.direction == EAST:
#                 upd(self.current[0] - i - 2, self.current[1] - 1, self.sensors[5][i])
#             elif self.direction == WEST:
#                 upd(self.current[0] + i + 2, self.current[1] + 1, self.sensors[5][i])
#             else: # self.direction == SOUTH:
#                 upd(self.current[0] - 1, self.current[1] + i + 2, self.sensors[5][i])

        for i in range(self.MAX_ROW):
            for j in range(self.MAX_COL):
                if [5, 8, 9].count(self.explored_map[i][j]) > 0:
                    matrix = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
                    for pos in matrix:
                        if 0 <= i + pos[0] < self.MAX_ROW and 0 <= j + pos[1] < self.MAX_COL:
                            self.map_state_changed[i + pos[0]][j + pos[1]] = "F"

        self.update_map_state()

        for cell in self.path_taken:
            self.explored_map[cell[0]][cell[1]] = cell[2]
            matrix = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
            for pos in matrix:
                if 0 <= j + pos[1] < self.MAX_COL and 0 <= i + pos[0] < self.MAX_ROW:
                    self.map_state_changed[i + pos[0]][j + pos[1]] = "F"
        self.mark_body(self.start, 6)
        self.mark_body(self.goal, 7)
        self.mark_robot()

        if self.steps[-4:] == [LEFT, RIGHT, LEFT, RIGHT]:
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

        for i in range(self.MAX_ROW):
            print (self.explored_map[i])

        zope.event.notify("SENSOR")
        return self.explored_map


    def update_map_state(self):
        for i in range(self.MAX_ROW):
            for j in range(self.MAX_COL):
                if self.map_state_changed[i][j] == "F":
                    
                    self.map_state[i][j] = 1
                    self.explored_map[i][j] = 1
                    
                elif self.map_state_changed[i][j] == "C":
                    if self.explored_map[i][j] == 2 and self.map[i][j] == 0:
                        self.map_state[i][j] = 3 #assign new state

                    elif self.explored_map[i][j] == 1 and self.map[i][j] == 0:
                        self.map_state[i][j] = 2 #assign new state

                    elif self.explored_map[i][j] == 1 and self.map[i][j] == 1 and self.map_state[i][j] == 2:
                        self.map_state[i][j] = 1 #(-1)

                    elif self.explored_map[i][j] == 1 and self.map[i][j] == 1 and self.map_state[i][j] == 1:
                        self.map_state[i][j] = 1 #constant

                    elif self.explored_map[i][j] == 2 and self.map[i][j] == 2 and self.map_state[i][j] == 3:
                        self.map_state[i][j] = 4 #(+1)

                    elif self.explored_map[i][j] == 2 and self.map[i][j] == 2 and self.map_state[i][j] == 4:
                        self.map_state[i][j] = 4 #constant

                    elif self.explored_map[i][j] == 2 and self.map[i][j] == 1 and self.map_state[i][j] == 1:
                        self.map_state[i][j] = self.map_state[i][j] + 1 #changed in state but current state at extreme end

                    elif self.explored_map[i][j] == 1 and self.map[i][j] == 2 and self.map_state[i][j] == 4:
                        self.map_state[i][j] = self.map_state[i][j] - 1 #changed in state but current state at extreme end

                    elif self.explored_map[i][j] == 2 and self.map[i][j] == 1 and self.map_state[i][j] == 2:
                        self.map_state[i][j] = self.map_state[i][j] + 1 #changed in state and current state in middle
                        self.explored_map[i][j] = 2

                    elif self.explored_map[i][j] == 1 and self.map[i][j] == 2 and self.map_state[i][j] == 3:
                        self.map_state[i][j] = self.map_state[i][j] - 1 #changed in state and current state in middle
                        self.explored_map[i][j] = 1

        return self.map_state

    """descriptor for whole map""" 
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

        return ''.join([h for h in hex_ret])

    """descriptor for explored map""" 
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
