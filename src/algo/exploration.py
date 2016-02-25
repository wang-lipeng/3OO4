
#===============================================================================
# """
# 0: unexplored
# 1: explored
# 2: obstacle
# 3: robot body
# 4: robot head
# 5: robot center
# 6: start
# 7: goal
# 8: explored path
# 9: optimum path
# """
#===============================================================================

"""
Map description: 
0: unexplored cell
1: explored cell
2: robot head
3: robot center
4: robot body
5: obstacle
6: start zone
7: goal zone
8: free path
9: shortest path
"""

import random
import time
from algo.shortest_path import ShortestPath
from algo.constants import *

class Exploration(object):

    """description for map & information tracked for robot"""
    realTimeMap = [] #########    
    pathTaken = [] ##########
    sensors = [] 
    
    preStep = "O"
    curStep = "O"
    centerX = 1
    centerY = 18
    directionX = 1
    directionY = 17
    
    repeatedCell = 0
    exploredCell = 0
    cnt = 0 ###########
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
        global cnt ########
        global terminateRobot
              
        global spList   ##########
        global spCounter ############    
        global repeatedTreshold
        global isFirstAttempt
        global coverageFigure 
        
        """0 for bottom row, 19 for top row"""
        realTimeMap = []     
        pathTaken = []
        
        """       
        description of sensors: 
        0: front-left            
        1: front-center
        2: front-right
        3: left
        4: right
        5: bottom-left
        6: robot facing direction(W-up,A-left,D-right,S-down)
        """
        sensors = []
               
        preStep = "O"
        curStep = "O"
        centerX = 1
        centerY = 18
        directionX = 1
        directionY = 17
        
        repeatedCell = 0
        exploredCell = 0
        cnt = 0
        terminateRobot = False
        
        spList = []
        spCounter = 0
        repeatedTreshold = 10 #6#5#20#15#10# 30 ################
        isFirstAttempt = True 
        coverageFigure= _coverageFigure
        
        """initial map"""
        for i in range (0,20):
            Row = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            realTimeMap.append(Row)

        """set robot's initial position as 1"""
        directions = [[0, 0], [0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
        for direction in directions:
            realTimeMap[centerY + direction[0]][centerX + direction[1]] = 1


    def main(self, sensors, explored_map):
        def normalizeY(i):
            if 0 <= i <= 19:
                return i
            elif i < 0:
                return 0
            else:
                return 19
        def normalizeX(j):
            if 0 <= j <= 14:
                return j
            elif j < 0:
                return 0
            else:
                return 14
        def manhattanDistance(_start, _end):
            return abs(_start[0] - _end[0]) + abs(_start[1] - _end[1])
        def free(i, j):  
            global centerX
            global centerY
            global realTimeMap
            if realTimeMap[i][j] == 0 or realTimeMap[i][j] == 5: #check center is not unexplored or obstacle
                return -INF
            directions = [[0, 1], [0, -1], [-1, 0], [-1, 1], [-1, -1], [1, 0], [1, 1], [1, -1]]
            for direction in directions: #check the adjacent 8 cell is not obstacle
                if realTimeMap[i + direction[0]][j + direction[1]] == 5:  
                    return -INF
            directions = [[0, 2], [1, 2], [-1, 2], [0, -2], [1, -2], [-1, -2], [-2, 0], [-2, 1], [-2, -1], [2, 0], [2, -1], [2, 1]]
            distance = manhattanDistance([centerY, centerX], [i, j])
            cnt = 0
            for direction in directions:
                temp = realTimeMap[normalizeY(i + direction[0])][normalizeX(j + direction[1])]
                if temp == 0:  #check 4 facing side
                    cnt += 2   #blah
            return cnt * cnt * cnt - distance


        global realTimeMap
        global sensorList
        global pathTaken
        global robotCenterX
        global robotPrevMovement
        global robotCurMovement
        global robotCenterX
        global robotCenterY
        global robotDirectionX
        global robotDirectionY
        global repeatedArea
        global exploredPercentage
        global robotBreak
        global exploredArea
        global isFirstStart

        global spList
        global spCounter
        global repeatedTreshold



        for i in range(20):
            for j in range(15):
                if explored_map[i][j] == 4:
                    robotDirectionY = i
                    robotDirectionX = j
                elif explored_map[i][j] == 5:
                    robotCenterY = i
                    robotCenterX = j

        for i in range(20):
            for j in range(15):
                val = explored_map[i][j]
                if val >= 3:
                    val = 1

                realTimeMap[i][j] = val

        #set robot starting position
        realTimeMap[robotCenterY][robotCenterX] = 5
        realTimeMap[robotDirectionY][robotDirectionX] = 4
        # print(robotCenterY, robotCenterX, ": ", robotDirectionY, robotDirectionX)

        encounteredStart = False

        if (robotCenterY, robotCenterX) == (18, 1):
            if not isFirstStart:
                encounteredStart = True
                robotBreak = True
                robotCurMovement = None
        else:
            isFirstStart = False

        if not encounteredStart or (exploredArea * 2 <= exploredPercentage * 3):
            exploredArea = 0

            self.callAllMethods(sensors, explored_map)
            if len(spList) <= 0:
                for tup in pathTaken:
                    if tup == (robotCenterY, robotCenterX, robotDirectionY, robotDirectionX):
                        repeatedArea = repeatedArea + 1
                        break
            for i in range(0,20):
                for j in range(0,15):
                    if realTimeMap[i][j] != 0:
                        exploredArea = exploredArea + 1
        if repeatedArea >= repeatedTreshold: # and (exploredArea * 2 >= exploredPercentage * 3):
            robotCurMovement = None
            #### JUST GIVE UP AND GO BACK HOME :')

            robotBreak = True

            if exploredArea >= exploredPercentage * 3 or spCounter > 5:
                # good enough, break
                robotBreak = True
            elif not robotBreak:

                # don't give up!
                rcurrent = [robotCenterY, robotCenterX]
                rdirection = NORTH

                if robotCenterY > robotDirectionY:
                    rdirection = NORTH
                elif robotCenterY < robotDirectionY:
                    rdirection = SOUTH
                elif robotCenterX < robotDirectionX:
                    rdirection = EAST
                elif robotCenterX > robotDirectionX:
                    rdirection = WEST

                dest = [10, 8]
                dest_candidate = []
                for i in range(1, 19):
                    for j in range(1, 14):
                        v = okay(i, j)
                        if v > -INF:
                            dest_candidate.append([[i, j], v])
                best_candidate_v = -INF
                best_candidate = [10, 8]
                for cand in dest_candidate:
                    if cand[1] > best_candidate_v:
                        best_candidate_v = cand[1]
                        best_candidate = cand[0]
                #print(dest_candidate)

                print("[Tornado | %s] exploration.py > dest_candidate > %s" % (time.ctime(time.time()), dest_candidate))
                dest = best_candidate

                sp = ShortestPath(realTimeMap, rdirection, rcurrent, dest)
                sp_list = sp.shortest_path(8)
                sp_sequence = sp_list['sequence']
                sp_sequence.reverse()
                spList = sp_sequence
                """
                print("--------------")
                print(rdirection)
                print(rcurrent)
                print(dest)
                print(spList)
                print("--------------")
                """
                #print(spList)
                print("[Tornado | %s] exploration.py > spList > %s" % (time.ctime(time.time()), spList))
                spCounter += 1
                repeatedArea = 0
                repeatedTreshold = 3

        # THIS IS THE CULPRIT, DO NOT MARK realTimeMap with 8 OR ELSE robotMovementAnalyses will fail :)
        # for tup in pathTaken:
        #   if realTimeMap[tup[0]][tup[1]] != 4 and realTimeMap[tup[0]][tup[1]] != 5:
        #       realTimeMap[tup[0]][tup[1]] = 8



    def callAllMethods(self, sensors, explored_map):
        global realTimeMap
        global sensorList
        global pathTaken
        global robotCenterX
        global robotPrevMovement
        global robotCurMovement
        global robotCenterX
        global robotCenterY
        global robotDirectionX
        global robotDirectionY

        sensorList = self.getSensor(robotCenterX, robotCenterY, robotDirectionX, robotDirectionY)
        sensors.insert(0, sensorList[0])
        sensorList = sensors


        for i in range(20):
            for j in range(15):
                val = explored_map[i][j]
                if val >= 3:
                    val = 1

                realTimeMap[i][j] = val

        #set robot starting position
        realTimeMap[robotCenterY][robotCenterX] = 5
        realTimeMap[robotDirectionY][robotDirectionX] = 4
        # realTimeMap = self.updateRealTimeMap(realTimeMap, sensorList, robotCenterX, robotCenterY)

        robotCurMovement = self.robotMovementAnalyses(realTimeMap, robotCenterX, robotCenterY, sensorList[0][0], robotPrevMovement, sensorList)
        robotPrevMovement = robotCurMovement
        if robotCurMovement == FORWARD:
            pathTaken.append((robotCenterY, robotCenterX, robotDirectionY, robotDirectionX))
        #print (robotCurMovement)
        # realTimeMap = self.executeRobotMovement(realTimeMap, robotCenterX, robotCenterY, sensorList[0][0], robotCurMovement)

        if sensorList[0][0] == NORTH:
            if robotCurMovement == RIGHT:
                robotDirectionX = robotDirectionX + 1
                robotDirectionY = robotDirectionY + 1
            elif robotCurMovement == FORWARD:
                robotCenterY = robotCenterY - 1
                robotDirectionY = robotDirectionY - 1
            elif robotCurMovement == LEFT:
                robotDirectionX = robotDirectionX - 1
                robotDirectionY = robotDirectionY + 1
        if sensorList[0][0] == SOUTH:
            if robotCurMovement == RIGHT:
                robotDirectionX = robotDirectionX - 1
                robotDirectionY = robotDirectionY - 1
            elif robotCurMovement == FORWARD:
                robotCenterY = robotCenterY + 1
                robotDirectionY = robotDirectionY + 1
            elif robotCurMovement == LEFT:
                robotDirectionX = robotDirectionX + 1
                robotDirectionY = robotDirectionY - 1
        if sensorList[0][0] == WEST:
            if robotCurMovement == RIGHT:
                robotDirectionX = robotDirectionX + 1
                robotDirectionY = robotDirectionY - 1
            elif robotCurMovement == FORWARD:
                robotCenterX = robotCenterX -1
                robotDirectionX = robotDirectionX - 1
            elif robotCurMovement == LEFT:
                robotDirectionX = robotDirectionX + 1
                robotDirectionY = robotDirectionY + 1
        if sensorList[0][0] == EAST:
            if robotCurMovement == RIGHT:
                robotDirectionX = robotDirectionX - 1
                robotDirectionY = robotDirectionY + 1
            elif robotCurMovement == FORWARD:
                robotCenterX = robotCenterX + 1
                robotDirectionX = robotDirectionX + 1
            elif robotCurMovement == LEFT:
                robotDirectionX = robotDirectionX - 1
                robotDirectionY = robotDirectionY - 1


    def getSensor(self, centerX, centerY, directionX, directionY):
        # returnValue[0] = direction of robot (W-Facing up, S-Facing down, A-facing left, D-facing right)
        # returnValue[1] = frontleft
        # returnValue[2] = frontcenter
        # returnValue[3] = frontright
        # returnValue[4] = left
        # returnValue[5] = right
        # returnValue[6] = bottomleft
        returnValue = []

        if (centerX == directionX) and (directionY < centerY):
            returnValue.append([NORTH])
        elif (centerX == directionX) and (directionY > centerY):
            returnValue.append([SOUTH])
        elif (centerY == directionY) and (directionX < centerX):
            returnValue.append([WEST])
        elif (centerY == directionY) and (directionX > centerX):
            returnValue.append([EAST])
        return returnValue

    def robotMovementAnalyses(self, realTimeMap, CenterX, CenterY, direction, prevMov, sensorList):

        def checkRealTimeMap(y, x):
            if 0 <= y < 20 and 0 <= x < 15 and realTimeMap[y][x] == 1:
                return True
            return False

        global spList
        if len(spList) > 0:
            resultMovement = spList.pop()
            return resultMovement

        if direction == NORTH:
            if sensorList[4][0] != None and checkRealTimeMap(CenterY-1, CenterX-2) and checkRealTimeMap(CenterY, CenterX-2) and checkRealTimeMap(CenterY+1, CenterX-2) and prevMov != LEFT:
                resultMovement = LEFT
            elif sensorList[1][0] != None and checkRealTimeMap(CenterY-2, CenterX-1) and checkRealTimeMap(CenterY-2, CenterX) and checkRealTimeMap(CenterY-2, CenterX+1):
                resultMovement = FORWARD
            elif sensorList[5][0] != None and checkRealTimeMap(CenterY-1, CenterX+2) and checkRealTimeMap(CenterY, CenterX+2) and checkRealTimeMap(CenterY+1, CenterX+2):
                resultMovement = RIGHT
            else:
                resultMovement = LEFT
        elif direction == SOUTH:
            if sensorList[4][0] != None and checkRealTimeMap(CenterY-1, CenterX+2) and checkRealTimeMap(CenterY, CenterX+2) and checkRealTimeMap(CenterY+1, CenterX+2) and prevMov != LEFT:
                resultMovement = LEFT
            elif sensorList[1][0] != None and checkRealTimeMap(CenterY+2, CenterX-1) and checkRealTimeMap(CenterY+2, CenterX) and checkRealTimeMap(CenterY+2, CenterX+1):
                resultMovement = FORWARD
            elif sensorList[5][0] != None and checkRealTimeMap(CenterY-1, CenterX-2) and checkRealTimeMap(CenterY, CenterX-2) and checkRealTimeMap(CenterY+1, CenterX-2):
                resultMovement = RIGHT
            else:
                resultMovement = LEFT
        elif direction == WEST:
            if sensorList[4][0] != None and checkRealTimeMap(CenterY+2, CenterX-1) and checkRealTimeMap(CenterY+2, CenterX) and checkRealTimeMap(CenterY+2, CenterX+1) and prevMov != LEFT:
                resultMovement = LEFT
            elif sensorList[1][0] != None and checkRealTimeMap(CenterY-1, CenterX-2) and checkRealTimeMap(CenterY, CenterX-2) and checkRealTimeMap(CenterY+1, CenterX-2):
                resultMovement = FORWARD
            elif sensorList[5][0] != None and checkRealTimeMap(CenterY-2, CenterX-1) and checkRealTimeMap(CenterY-2, CenterX) and checkRealTimeMap(CenterY-2, CenterX+1):
                resultMovement = RIGHT
            else:
                resultMovement = LEFT
        elif direction == EAST:
            if sensorList[4][0] != None and checkRealTimeMap(CenterY-2, CenterX-1) and checkRealTimeMap(CenterY-2, CenterX) and checkRealTimeMap(CenterY-2, CenterX+1) and prevMov != LEFT:
                resultMovement = LEFT
            elif sensorList[1][0] != None and checkRealTimeMap(CenterY-1, CenterX+2) and checkRealTimeMap(CenterY, CenterX+2) and checkRealTimeMap(CenterY+1, CenterX+2):
                resultMovement = FORWARD
            elif sensorList[5][0] != None and checkRealTimeMap(CenterY+2, CenterX-1) and checkRealTimeMap(CenterY+2, CenterX) and checkRealTimeMap(CenterY+2, CenterX+1):
                resultMovement = RIGHT
            else:
                resultMovement = LEFT
        return resultMovement

    def executeRobotMovement(self, realTimeMap, CenterX, CenterY, direction, movement):
        if direction == NORTH:
            if movement == RIGHT:
                realTimeMap[CenterY][CenterX+1] = 4
                realTimeMap[CenterY-1][CenterX] = 1
            elif movement == FORWARD:
                realTimeMap[CenterY][CenterX] = 1
                realTimeMap[CenterY-1][CenterX] = 5
                realTimeMap[CenterY-2][CenterX] = 4
            elif movement == LEFT:
                realTimeMap[CenterY][CenterX-1] = 4
                realTimeMap[CenterY-1][CenterX] = 1
        elif direction == SOUTH:
            if movement == RIGHT:
                realTimeMap[CenterY][CenterX-1] = 4
                realTimeMap[CenterY+1][CenterX] = 1
            elif movement == FORWARD:
                realTimeMap[CenterY][CenterX] = 1
                realTimeMap[CenterY+1][CenterX] = 5
                realTimeMap[CenterY+2][CenterX] = 4
            elif movement == LEFT:
                realTimeMap[CenterY][CenterX+1] = 4
                realTimeMap[CenterY+1][CenterX] = 1
        elif direction == WEST:
            if movement == RIGHT:
                realTimeMap[CenterY-1][CenterX] = 4
                realTimeMap[CenterY][CenterX-1] = 1
            elif movement == FORWARD:
                realTimeMap[CenterY][CenterX] = 1
                realTimeMap[CenterY][CenterX-1] = 5
                realTimeMap[CenterY][CenterX-2] = 4
            elif movement == LEFT:
                realTimeMap[CenterY+1][CenterX] = 4
                realTimeMap[CenterY][CenterX-1] = 1
        elif direction == EAST:
            if movement == RIGHT:
                realTimeMap[CenterY+1][CenterX] = 4
                realTimeMap[CenterY][CenterX+1] = 1
            elif movement == FORWARD:
                realTimeMap[CenterY][CenterX] = 1
                realTimeMap[CenterY][CenterX+1] = 5
                realTimeMap[CenterY][CenterX+2] = 4
            elif movement == LEFT:
                realTimeMap[CenterY-1][CenterX] = 4
                realTimeMap[CenterY][CenterX+1] = 1
        return realTimeMap


    def getRealTimeMap(self, sensors, explored_map):
        global cnt
        global realTimeMap
        global robotCurMovement
        global robotBreak
        global robotCenterX
        global robotCenterY
        #print(cnt + 1, "before: ",  robotCurMovement)
        self.main(sensors, explored_map)
        print("[Tornado | %s] exploration.py > %d - %s : (%d, %d)" %(time.ctime(time.time()), cnt + 1, robotCurMovement, robotCenterY, robotCenterX))
        cnt += 1
        return (robotCurMovement, robotBreak)
