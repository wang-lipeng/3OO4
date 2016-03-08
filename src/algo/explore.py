
"""
Map description: 
0: unexplored cell
1: explored cell
2: obstacle
3: robot body
4: robot head
5: robot center
6: start zone
7: goal zone
8: explored path
9: shortest path
"""

import random
import time
from algo.shortestPath import ShortestPath
from algo.constants import *
from matplotlib.cbook import Null
from objc._objc import NULL

class Exploration(object):

    """description for map & information tracked for robot"""
    realTimeMap = []     
    pathTaken = [] 
    sensors = [] 
    
    preStep = "O"
    curStep = "O"
    centerX = 1
    centerY = 18
    directionX = 1
    directionY = 17
    
    repeatedCell = 0
    exploredCell = 0
    count = 0 
    terminateRobot= False

    def __init__(self, _coverageFigure):
        super(Exploration, self).__init__()
        
        global realTimeMap
        global pathTaken
        global sensors 
        
        global preStep
        global curStep
        global centerX
        global centerY
        global directionX
        global directionY
        
        global repeatedCell
        global exploredCell
        global count
        global terminateRobot
              
        global sPathList #shortest path list
        global sPathCounter   
        global repeatedTorlerence
        global isInitialStart
        global coverageFigure 
        
        """0 for bottom row, 19 for top row"""
        realTimeMap = []     
        pathTaken = []
        
        sensors = []
               
        preStep = "O"
        curStep = "O"
        centerX = 1
        centerY = 18
        directionX = 1
        directionY = 17
        
        repeatedCell = 0
        exploredCell = 0
        count = 0
        terminateRobot = False
        
        sPathList = []
        sPathCounter = 0
        repeatedTorlerence = 10 #6, 5, 20, 15, 30...
        isInitialStart = True 
        coverageFigure= _coverageFigure
        
        """initial map"""
        for i in range (0,20):
            Row = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            realTimeMap.append(Row)

        """set robot's initial position as 1"""
        matrix = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for pos in matrix:
            realTimeMap[centerY + pos[0]][centerX + pos[1]] = 1


    def main(self, srs, explored_map):
        """translate (x,y) to be within (15,20)"""
        def normalizeY(i): #constraint for 20 rows 
            if 0 <= i <= 19:
                return i
            elif i < 0:
                return 0
            else:
                return 19
        def normalizeX(j): #constraint for 15 columns 
            if 0 <= j <= 14:
                return j
            elif j < 0:
                return 0
            else:
                return 14
        def manhattanDistance(_start, _end):
            return abs(_start[0] - _end[0]) + abs(_start[1] - _end[1])
        def isfree(i, j):  
            global centerX
            global centerY
            global realTimeMap
            if realTimeMap[i][j] == 0 or realTimeMap[i][j] == 2: #check center is not unexplored or obstacle
                return -INF
            matrix = [[0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
            for pos in matrix: 
                if realTimeMap[i + pos[0]][j + pos[1]] == 2:#check the adjacent 8 cell is not obstacle
                    return -INF
            distance = manhattanDistance([centerY, centerX], [i, j])
            matrix = [[0, 2], [1, 2], [-1, 2], [0, -2], [1, -2], [-1, -2], [-2, 0], [-2, 1], [-2, -1], [2, 0], [2, -1], [2, 1]]
            count = 0
            for pos in matrix:
                cellValue = realTimeMap[normalizeY(i + pos[0])][normalizeX(j + pos[1])]
                if cellValue == 0:  #check 4 facing side cells(12) are unexplored 
                    count = count + 2
            return count * count * count - distance 

        global realTimeMap
        global pathTaken
        global sensors 
        
        global preStep
        global curStep
        global centerX
        global centerY
        global directionX
        global directionY
        
        global repeatedCell
        global exploredCell
        global terminateRobot
              
        global sPathList   
        global sPathCounter  
        global repeatedTorlerence
        global isInitialStart
        global coverageFigure 

        """set robot's direction and center"""
        for i in range(20):
            for j in range(15):
                if explored_map[i][j] ==4: #head
                    directionY = i
                    directionX = j
                elif explored_map[i][j] == 5: #center 
                    centerY = i
                    centerX = j

        """set all explored cells in real map to 1"""
        for i in range(20):
            for j in range(15):
                val = explored_map[i][j]
                if val >= 3:
                    val = 1

                realTimeMap[i][j] = val

        """set robot's starting position in real map"""
        realTimeMap[centerY][centerX] = 5
        realTimeMap[directionY][directionX] = 4
        
        """check when robot is at the starting position"""
        atStart = False
        if (centerY, centerX) == (18, 1):
            if not isInitialStart:
                atStart = True
                terminateRobot = True
                curStep = None
        else:
            isInitialStart = False

        if not atStart or (exploredCell * 2 <= coverageFigure * 3):
            exploredCell = 0

            self.takeStep(srs, explored_map)
            if len(sPathList) <= 0:
                for pos in pathTaken:
                    if pos == (centerY, centerX, directionY, directionX):
                        repeatedCell = repeatedCell + 1
                        break
            for i in range(0,20):
                for j in range(0,15):
                    if realTimeMap[i][j] != 0:
                        exploredCell = exploredCell + 1
    
        if coverageFigure == 100:
            if exploredCell >= coverageFigure * 3 and atStart:
                """end exploration with fully explored map"""
                terminateRobot = True
        else:
            if exploredCell >= coverageFigure * 3:
                """end exploration with satisfied percentage"""
                terminateRobot = True
        
        if repeatedCell >= repeatedTorlerence: 
            """end exploration due to exceeding repeat Torlerence"""
            curStep = None            
            terminateRobot = True

            if exploredCell >= coverageFigure * 3 or sPathCounter > 5:
                """end exploration with satisfied result"""
                terminateRobot = True
            
            elif not terminateRobot:
                """continue the exploration"""
                rCenter = [centerY, centerX]
                rDir = NORTH

                if centerY > directionY:
                    rDir = NORTH
                elif centerY < directionY:
                    rDir = SOUTH
                elif centerX < directionX:
                    rDir = EAST
                elif centerX > directionX:
                    rDir = WEST

                """decide a temporary goal for robot to achieve"""
                best_tempGoal = [10, 8] #center of the map
                best_tempGoal_val = -INF
                
                possible_tempGoals = []
                for i in range(1, 19):
                    for j in range(1, 14):
                        val = isfree(i, j)
                        if val > -INF:
                            possible_tempGoals.append([[i, j], val])
                
                for goal in possible_tempGoals:
                    if goal[1] > best_tempGoal_val:
                        best_tempGoal_val = goal[1]
                        best_tempGoal = goal[0]
                
                tempGoal = best_tempGoal

                """execute shortest path"""
                sPath = ShortestPath(realTimeMap, rDir, rCenter, tempGoal)
                sPath_list = sPath.shortest_path(8)
                sPath_sequence = sPath_list['sequence']
                sPath_sequence.reverse()
                sPathList = sPath_sequence
                                
                sPathCounter = sPathCounter + 1
                repeatedCell = 0
                repeatedTorlerence = 3

    

    def getSensor(self, cenX, cenY, dirX, dirY):
        """       
        description of sensors: 
        1: front-left            
        2: front-center
        3: front-right
        4: left
        5: right
        6: bottom-left
        0: robot facing direction(W-up,A-left,D-right,S-down)
        """
        results = []

        if (cenX == dirX) and (dirY < cenY):
            results.append([NORTH])
        elif (cenX == dirX) and (dirY > cenY):
            results.append([SOUTH])
        elif (cenY == dirY) and (dirX < cenX):
            results.append([WEST])
        elif (cenY == dirY) and (dirX > cenX):
            results.append([EAST])
        return results

    def decideStep(self, realTimeMap, cenX, cenY, direction, preStep, sensors):

        def validateMap(y, x):
            if 0 <= y < 20 and 0 <= x < 15 and realTimeMap[y][x] == 1:
                return True
            return False

        global sPathList
        if len(sPathList) > 0:
            step = sPathList.pop()
            return step

        if direction == NORTH:
            if sensors[4][0] != None and validateMap(cenY-1, cenX-2) and validateMap(cenY, cenX-2) and validateMap(cenY+1, cenX-2) and preStep != LEFT:
                step = LEFT
            elif sensors[1][0] != None and validateMap(cenY-2, cenX-1) and validateMap(cenY-2, cenX) and validateMap(cenY-2, cenX+1):
                step = FORWARD
            elif sensors[5][0] != None and validateMap(cenY-1, cenX+2) and validateMap(cenY, cenX+2) and validateMap(cenY+1, cenX+2):
                step = RIGHT
            else:
                step = LEFT
        elif direction == SOUTH:
            if sensors[4][0] != None and validateMap(cenY-1, cenX+2) and validateMap(cenY, cenX+2) and validateMap(cenY+1, cenX+2) and preStep != LEFT:
                step = LEFT
            elif sensors[1][0] != None and validateMap(cenY+2, cenX-1) and validateMap(cenY+2, cenX) and validateMap(cenY+2, cenX+1):
                step = FORWARD
            elif sensors[5][0] != None and validateMap(cenY-1, cenX-2) and validateMap(cenY, cenX-2) and validateMap(cenY+1, cenX-2):
                step = RIGHT
            else:
                step = LEFT
        elif direction == WEST:
            if sensors[4][0] != None and validateMap(cenY+2, cenX-1) and validateMap(cenY+2, cenX) and validateMap(cenY+2, cenX+1) and preStep != LEFT:
                step = LEFT
            elif sensors[1][0] != None and validateMap(cenY-1, cenX-2) and validateMap(cenY, cenX-2) and validateMap(cenY+1, cenX-2):
                step = FORWARD
            elif sensors[5][0] != None and validateMap(cenY-2, cenX-1) and validateMap(cenY-2, cenX) and validateMap(cenY-2, cenX+1):
                step = RIGHT
            else:
                step = LEFT
        elif direction == EAST:
            if sensors[4][0] != None and validateMap(cenY-2, cenX-1) and validateMap(cenY-2, cenX) and validateMap(cenY-2, cenX+1) and preStep != LEFT:
                step = LEFT
            elif sensors[1][0] != None and validateMap(cenY-1, cenX+2) and validateMap(cenY, cenX+2) and validateMap(cenY+1, cenX+2):
                step = FORWARD
            elif sensors[5][0] != None and validateMap(cenY+2, cenX-1) and validateMap(cenY+2, cenX) and validateMap(cenY+2, cenX+1):
                step = RIGHT
            else:
                step = LEFT
        return step
    
    def takeStep(self, srs, explored_map):
        global realTimeMap
        global sensors
        global pathTaken
        global preStep
        global curStep
        global centerX
        global centerY
        global directionX
        global directionY

        sensors = self.getSensor(centerX, centerY, directionX, directionY)
        srs.insert(0, sensors[0])
        sensors = srs

        """set all explored cell in real map to 1"""
        for i in range(20):
            for j in range(15):
                val = explored_map[i][j]
                if val >= 3:
                    val = 1 
                realTimeMap[i][j] = val                   
                 
        """set robot's starting position in real map"""
        realTimeMap[centerY][centerX] = 5
        realTimeMap[directionY][directionX] = 4

        """move step and track taken path"""
        curStep = self.decideStep(realTimeMap, centerX, centerY, sensors[0][0], preStep, sensors)
        preStep = curStep
        if curStep == FORWARD:
            pathTaken.append((centerY, centerX, directionY, directionX))
        
        """update robot's head/center coordinations """
        if sensors[0][0] == NORTH:
            if curStep == RIGHT:
                directionX = directionX + 1
                directionY = directionY + 1
            elif curStep == FORWARD:
                centerY = centerY - 1
                directionY = directionY - 1
            elif curStep == LEFT:
                directionX = directionX - 1
                directionY = directionY + 1
        if sensors[0][0] == SOUTH:
            if curStep == RIGHT:
                directionX = directionX - 1
                directionY = directionY - 1
            elif curStep == FORWARD:
                centerY = centerY + 1
                directionY = directionY + 1
            elif curStep == LEFT:
                directionX = directionX + 1
                directionY = directionY - 1
        if sensors[0][0] == WEST:
            if curStep == RIGHT:
                directionX = directionX + 1
                directionY = directionY - 1
            elif curStep == FORWARD:
                centerX = centerX -1
                directionX = directionX - 1
            elif curStep == LEFT:
                directionX = directionX + 1
                directionY = directionY + 1
        if sensors[0][0] == EAST:
            if curStep == RIGHT:
                directionX = directionX - 1
                directionY = directionY + 1
            elif curStep == FORWARD:
                centerX = centerX + 1
                directionX = directionX + 1
            elif curStep == LEFT:
                directionX = directionX - 1
                directionY = directionY - 1

    def updateMap(self, realTimeMap, cenX, cenY, direction, step):
        """update real time map"""
        if direction == NORTH:
            if step == RIGHT:
                realTimeMap[cenY][cenX+1] = 4
                realTimeMap[cenY-1][cenX] = 1
            elif step == FORWARD:
                realTimeMap[cenY][cenX] = 1
                realTimeMap[cenY-1][cenX] = 5
                realTimeMap[cenY-2][cenX] = 4
            elif step == LEFT:
                realTimeMap[cenY][cenX-1] = 4
                realTimeMap[cenY-1][cenX] = 1
        elif direction == SOUTH:
            if step == RIGHT:
                realTimeMap[cenY][cenX-1] = 4
                realTimeMap[cenY+1][cenX] = 1
            elif step == FORWARD:
                realTimeMap[cenY][cenX] = 1
                realTimeMap[cenY+1][cenX] = 5
                realTimeMap[cenY+2][cenX] = 4
            elif step == LEFT:
                realTimeMap[cenY][cenX+1] = 4
                realTimeMap[cenY+1][cenX] = 1
        elif direction == WEST:
            if step == RIGHT:
                realTimeMap[cenY-1][cenX] = 4
                realTimeMap[cenY][cenX-1] = 1
            elif step == FORWARD:
                realTimeMap[cenY][cenX] = 1
                realTimeMap[cenY][cenX-1] = 5
                realTimeMap[cenY][cenX-2] = 4
            elif step == LEFT:
                realTimeMap[cenY+1][cenX] = 4
                realTimeMap[cenY][cenX-1] = 1
        elif direction == EAST:
            if step == RIGHT:
                realTimeMap[cenY+1][cenX] = 4
                realTimeMap[cenY][cenX+1] = 1
            elif step == FORWARD:
                realTimeMap[cenY][cenX] = 1
                realTimeMap[cenY][cenX+1] = 5
                realTimeMap[cenY][cenX+2] = 4
            elif step == LEFT:
                realTimeMap[cenY-1][cenX] = 4
                realTimeMap[cenY][cenX+1] = 1
        return realTimeMap


    def getRealTimeMap(self, srs, explored_map):
        global count
        global realTimeMap
        global curStep
        global terminateRobot
        global centerX
        global centerY
        global coverageFigure
        self.main(srs, explored_map)
        print count + 1, ":" , "current step: ",curStep, "center: ", "(",centerY, ",", centerX,")"
        print'Defined percentage: ' ,coverageFigure, "%"
        print 'Number of explored cells: ', exploredCell
        print ""
       
        count = count+1
        return (curStep, terminateRobot)
