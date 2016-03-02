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
    def __init__(self):
        super(Robot, self).__init__()
        self.map = []
        self.explored_map = []
        self.get_map()
        
        self.MAX_ROW = len(self.map)
        self.MAX_COL = len(self.map[0])
        self.to_change_color = []

        self.start = [18, 1] # center at start zone
        self.goal = [1, 13] # center at goal zone
        self.current = [18, 1] # center at start zone

        self.mark_body(self.start, 6) #mark start zone
        self.mark_body(self.goal, 7) #mark goal zone
        
        self.direction = NORTH # set robot direction to NORTH
        self.mark_robot()

        zope.event.notify("INIT")

    def mark_body(self, center, val):
        """mark value for 9 cells & change color based on current center value"""
        matrix = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for pos in matrix:
            current_val = self.explored_map[center[0] + pos[0]][center[1] + pos[1]]
            if current_val == 1 or 6 <= current_val <= 9: #explored but not the robot's body
                self.to_change_color.append({
                    "coordinate": [center[0] + pos[0], center[1] + pos[1]],
                    "value": current_val
                })
            self.explored_map[center[0] + pos[0]][center[1] + pos[1]] = val
    
    def mark_robot(self):
        self.mark_body(self.current, 3)
        """mark value for robot center and head"""
        self.explored_map[self.current[0]][self.current[1]] = 5 #robot center
        if self.direction == NORTH:
            self.explored_map[self.current[0] - 1][self.current[1]] = 4 #robot head
        elif self.direction == EAST:
            self.explored_map[self.current[0]][self.current[1] + 1] = 4 #robot head
        elif self.direction == WEST:
            self.explored_map[self.current[0]][self.current[1] - 1] = 4 #robot head
        else: # self.direction == SOUTH
            self.explored_map[self.current[0] + 1][self.current[1]] = 4 #robot head
    
    def format_mark(self):
        """update explored map"""
        for cell in self.to_change_color:
            self.explored_map[cell['coordinate'][0]][cell['coordinate'][1]] = cell['value']
        """clear change color cell list"""
        self.to_change_color = []

    def isOkay(self, center):
        matrix = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for pos in matrix:
            """out of map or is obstacle"""
            if  center[0] + pos[0] < 0 or center[0] + pos[0] >= self.MAX_ROW or \
                center[1] + pos[1] < 0 or center[1] + pos[1] >= self.MAX_COL or \
                self.map[center[0] + pos[0]][center[1] + pos[1]] == 2:               
                return False
        return True

    def get_map(self):
        start = []
        goal = []
        """read map from txt file"""
        with open("maps/map5.txt") as f:
            content = f.readlines()
            for line in content:
                map = []
                explored_map = []
                for char in line:
                    if char.isdigit():
                        map.append(int(char))
                        explored_map.append(0)
                self.map.append(map)
                self.explored_map.append(explored_map)

    def step(self, step, val = 8):
        if step:
            if step == FORWARD or step.isdigit() or step.islower():
                times = int(step, 16)
                for i in range(times):
                    self.forward(val)
            elif step == LEFT or step == RIGHT:
                self.rotate(step)
        zope.event.notify(step)

    def forward(self, val = 8):
        self.format_mark()
        self.mark_body(self.start, 6)
        self.mark_body(self.goal, 7)
        if val >= 0: #explored or obstacle
            self.explored_map[self.current[0]][self.current[1]] = val #update explored map
        next_step = []
        next_step.append(self.current[0])
        next_step.append(self.current[1])
        """decide next step coordinates"""
        if self.direction == NORTH:
            next_step[0] -= 1
        elif self.direction == EAST:
            next_step[1] += 1
        elif self.direction == WEST:
            next_step[1] -= 1
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

    def get_value(self, y, x):
        if 0 <= y < self.MAX_ROW and 0 <= x < self.MAX_COL:
            """update explored map""" 
            if self.map[y][x] == 2: #obstacle 
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
        """6 sensors version""" 
        for i in range(5):
            sensors.append([])
            for j in range(4):
                sensors[i].append(None)

        """front left""" 
        for i in range(4):
            if self.direction == NORTH:
                sensors[0][i] = self.get_value(self.current[0] - i - 2, self.current[1] - 1)
            elif self.direction == EAST:
                sensors[0][i] = self.get_value(self.current[0] - 1 , self.current[1] + i + 2)
            elif self.direction == WEST:
                sensors[0][i] = self.get_value(self.current[0] + 1, self.current[1] - i - 2)
            else: # self.direction == SOUTH:
                sensors[0][i] = self.get_value(self.current[0] + i + 2, self.current[1] + 1)
            if sensors[0][i] == 2: #encounter obstacle 
                break
            
        """front middle""" 
        for i in range(4):
            if self.direction == NORTH:
                sensors[1][i] = self.get_value(self.current[0] - i - 2, self.current[1])
            elif self.direction == EAST:
                sensors[1][i] = self.get_value(self.current[0], self.current[1] + i + 2)
            elif self.direction == WEST:
                sensors[1][i] = self.get_value(self.current[0], self.current[1] - i - 2)
            else: # self.direction == SOUTH:
                sensors[1][i] = self.get_value(self.current[0] + i + 2, self.current[1])
            if sensors[1][i] == 2: #encounter obstacle 
                break
        
        """front right""" 
        for i in range(4):
            if self.direction == NORTH:
                sensors[2][i] = self.get_value(self.current[0] - i - 2, self.current[1] + 1)
            elif self.direction == EAST:
                sensors[2][i] = self.get_value(self.current[0] + 1, self.current[1] + i + 2)
            elif self.direction == WEST:
                sensors[2][i] = self.get_value(self.current[0] - 1, self.current[1] - i - 2)
            else: # self.direction == SOUTH:
                sensors[2][i] = self.get_value(self.current[0] + i + 2, self.current[1] - 1)
            if sensors[2][i] == 2: #encounter obstacle
                break
        
        """left (front)""" 
        for i in range(4):
            if self.direction == NORTH:
                sensors[3][i] = self.get_value(self.current[0] - 1, self.current[1] - i - 2)
            elif self.direction == EAST:
                sensors[3][i] = self.get_value(self.current[0] - i - 2, self.current[1] + 1)
            elif self.direction == WEST:
                sensors[3][i] = self.get_value(self.current[0] + i + 2, self.current[1] - 1)
            else: # self.direction == SOUTH:
                sensors[3][i] = self.get_value(self.current[0] + 1, self.current[1] + i + 2)
            if sensors[3][i] == 2: #encounter obstacle
                break

        """right (front)""" 
        for i in range(4):
            if self.direction == NORTH:
                sensors[4][i] = self.get_value(self.current[0] - 1, self.current[1] + i + 2)
            elif self.direction == EAST:
                sensors[4][i] = self.get_value(self.current[0] + i + 2, self.current[1] + 1)
            elif self.direction == WEST:
                sensors[4][i] = self.get_value(self.current[0] - i - 2, self.current[1] - 1)
            else: # self.direction == SOUTH:
                sensors[4][i] = self.get_value(self.current[0] + 1, self.current[1] - i - 2)
            if sensors[4][i] == 2:  #encounter obstacle
                break
          
        """left bottom""" 
#         for i in range(4):
#             if self.direction == NORTH:
#                 sensors[5][i] = self.get_value(self.current[0] + 1, self.current[1] - i - 2)
#             elif self.direction == EAST:
#                 sensors[5][i] = self.get_value(self.current[0] - i - 2, self.current[1] - 1)
#             elif self.direction == WEST:
#                 sensors[5][i] = self.get_value(self.current[0] + i + 2, self.current[1] + 1)
#             else: # self.direction == SOUTH:
#                 sensors[5][i] = self.get_value(self.current[0] - 1, self.current[1] + i + 2)
#             if sensors[5][i] == 2: #encounter obstacle
#                 break     
        
        zope.event.notify("SENSOR")
        return sensors

    """descriptor for whole map""" 
    def descriptor_one(self):
        bitStream = [1, 1]
        for row in reversed(self.explored_map):
            for col in row:
                if col > 0:
                    bitStream.append(1) #explored
                else:
                    bitStream.append(0) #unexplored 
        bitStream.append(1)
        bitStream.append(1)

        hexStream = []
        temp = []
        for bit in bitStream:
            if len(temp) < 4:
                temp.append(bit)
            else:
                temp_str = ''.join([str(b) for b in temp])
                hexStream.append(str(hex(int(temp_str, 2)))[2:])
                temp = [bit]
        if len(temp) > 0:
            temp_str = ''.join([str(b) for b in temp])
            hexStream.append(str(hex(int(temp_str, 2)))[2:])

        return ''.join([h for h in hexStream])

    """descriptor for explored map""" 
    def descriptor_two(self):
        bitStream = []
        count = 0
        for row in reversed(self.explored_map):
            for col in row:
                if col > 0:
                    count += 1
                    if col == 2:
                        bitStream.append(1) #obstacle 
                    else:
                        bitStream.append(0) #free cell
        while count % 8 != 0:
            bitStream.append(0)
            count += 1

        hexStream = []
        temp = []
        for bit in bitStream:
            if len(temp) < 4:
                temp.append(bit)
            else:
                temp_str = ''.join([str(b) for b in temp])
                hexStream.append(str(hex(int(temp_str, 2)))[2:])
                temp = [bit]
        if len(temp) > 0:
            temp_str = ''.join([str(b) for b in temp])
            hexStream.append(str(hex(int(temp_str, 2)))[2:])
        
        return ''.join([h for h in hexStream])

if __name__ == '__main__':
    robot = Robot()
    print(robot.get_sensors())
    print(robot.explored_map)
