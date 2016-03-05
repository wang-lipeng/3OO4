import tornado.ioloop
import tornado.web
import tornado.websocket
import threading
from functools import wraps
from tornado.options import define, options, parse_command_line

import datetime
import json

import random
import sys

from algo.constants import *
import algo.realRun
from algo.explore import Exploration
from algo.shortestPath import ShortestPath
import multiprocessing
import gevent

from gevent import socket
from gevent.event import Event
import zope.event

"""record exploration path for examine purpose"""
f = open('log.txt', 'w')
orig_stdout = sys.stdout

clients = dict()
delay_time = 0.1
sensors = []
started = False
android_ok = False
exp_done = False
io_loop = False
completeExplore = False
exploration_started = False

sp_to_goal_started = False

define("port", default=8003, help="run on the defined port", type=int)

class FuncThread(threading.Thread):
    def __init__(self, target, *args):
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        self._target(*self._args)

class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def open(self, *args):
        self.id = self.get_argument("Id")
        self.stream.set_nodelay(True)
        clients[self.id] = {"id": self.id, "object": self}
        tick("INIT")

    def on_message(self, message):
        print ("Client " + str(self.id) + " received a message : " + str(message))
        
    def on_close(self):
        if self.id in clients:
            del clients[self.id]

def delay_call(f, *args, **kwargs):
#     evt.wait()
    # ignore delay_time, don't spawn new thread
    f(*args, **kwargs)

    # global delay_time
    # t = threading.Timer(delay_time, f, args=args, kwargs=kwargs)
    # t.start()

def real_delay_call(f, delay_time, *args, **kwargs):
    t = threading.Timer(delay_time, f, args=args, kwargs=kwargs)
    t.start()

# """execute exploration function"""
# def execute_exploration(percentage, delay):
#     global robot
#     global started
#     global delay_time
#     global log
#     global sensors
#     
#     robot = algo.realRun.Robot()
#     if started:
#         return
#     started = True
#     delay_time = float(delay)    
#     if log.closed:
#         log = open('log.txt', 'w')
#     sys.stdout = log
# #     sensors = robot.get_sensors()
#     sensors = robot.receive_sensors("40,40,40,40,40,40")
#     
#     robot.receive_sensors("40,40,40,40,40,40")
# #     inform(robot.receive_sensors("40,40,40,40,40,40"))
#     robot.step(RIGHT)
#     robot.receive_sensors("40,40,40,40,40,40")
#     robot.step(LEFT)
#     robot.receive_sensors("40,40,40,40,40,40")
# 
#     explore = Exploration(int(percentage))
# 
#     t = FuncThread(exploration, explore)
#     t.start()
#     t.join()
# 
#     inform("Start to explore...")

# """execute shortest path function"""
# def execute_sPath():
#     global robot
#     if not completeExplore:
#         return False
#     sPath = ShortestPath(robot.explored_map, robot.direction, robot.current, robot.goal)
#     sPath_list = sPath.shortestPath()
#     sPath_sequence = sPath_list['trim_seq']
#     sPath_sequence_start_to_goal = sPath_sequence[:]
#     sPath_sequence.reverse() 
#     delay_call(sPath_to_goal, sPath_sequence)
#     inform("Start shortest path...")
#     inform(sPath_sequence_start_to_goal)

"""handler for html file"""
class IndexHandler(tornado.web.RequestHandler):
    @tornado.web.asynchronous
    def get(self):
        self.render("Simulator.html")
        
"""start handler for exploration"""

class StartHandler(tornado.web.RequestHandler):
    @tornado.web.asynchronous
    def get(self, percentage, delay):
        self.write("Starting...")
 
        t = FuncThread(start_exploration, 100, 0.0)
        t.start()
 
        self.flush()

"""start handler for shortest path"""
class StartSPathHandler(tornado.web.RequestHandler):
    @tornado.web.asynchronous
    def get(self):
        self.write("Starting...")
        t = FuncThread(start_sp_to_goal)
        t.start()
        self.flush()

"""stop handler for request"""
class StopHandler(tornado.web.RequestHandler):
    @tornado.web.asynchronous
    def get(self):
        global started
        inform("Stop exploration...")
        started = False
        self.flush()
        
        global f
        sys.stdout = orig_stdout
        f.close()

"""the web application UI"""
app = tornado.web.Application([
    (r'/', IndexHandler),
    (r'/ws', WebSocketHandler),
    (r'/start/(.*)/(.*)', StartHandler),
    (r'/start_sp/', StartSPathHandler),
    (r'/stop/', StopHandler)
])

def tick(step):
    for key in clients:
        message = dict()
        message['type'] = 'map'
        message['step'] = step
        message['time'] = str(datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3])+'Z'
        message['map'] = robot.explored_map
        clients[key]['object'].write_message(json.dumps(message))

def inform(string):
    print(string)
    for key in clients:
        message = dict()
        message['type'] = 'info'
        message['time'] = str(datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3])+'Z'
        message['info'] = string
        clients[key]['object'].write_message(json.dumps(message))

def do_alignment(steps):
    global started
    if not started or len(steps) <= 0:
        return False
    choice = steps[0]
    steps = steps[1:]
    if choice == RIGHT or choice == LEFT:
        robot.step(choice)


    gevent.joinall([
        gevent.spawn(delay_call, do_alignment, steps)
    ])


def sp_to_start(sequence):
    global started
    #if not started:
    #    return False
    if len(sequence) == 0:
        done_sp_to_start()
        return False

    #evt.wait()
    do_alignment(robot.alignment())

    choice = sequence.pop()
    robot.step(choice, -1)
#     send_cmd(choice)

#     print("[Tornado | %s] sp_to_start > %s : %s" %(time.ctime(time.time()), choice, robot.direction))
    gevent.joinall([
        gevent.spawn(sp_to_start, sequence)
    ])

def done_sp_to_start():
    global exp_done
    inform("Gone back to start: Alignment!")

    #evt.wait()
#     send_cmd("W")
    #evt.wait()

    # Calibrate first!
    if robot.direction == NORTH:
        robot.step(LEFT)
#         send_cmd(LEFT)
        #evt.wait()
    elif robot.direction == SOUTH:
        robot.step(RIGHT)
#         send_cmd(RIGHT)
        #evt.wait()
    elif robot.direction == EAST:
        robot.step(LEFT)
#         send_cmd(LEFT)
        #evt.wait()
        robot.step(LEFT)
#         send_cmd(LEFT)
        #evt.wait()

#     send_cmd(FD_AL) # W
    #evt.wait()

#     send_cmd(LD_AL) # Q/
    #evt.wait()

    robot.step(RIGHT)
#     send_cmd(RIGHT) # D
    #evt.wait()

#     send_cmd(LA_AL) # L
    #evt.wait()


    inform(robot.descriptor_one())
    inform(robot.descriptor_two())


    exp_done = True
    global exploration_started
    exploration_started = False
    inform("Gone back to start, for real!")

#####################
###    sp_to_goal
#####################

def start_sp_to_goal():
    global robot
    ## Don't "started" so the map won't update
    #global started
    global sp_to_goal_started
    if sp_to_goal_started:
        return
    sp_to_goal_started = True


    if not exp_done:
        return False
    #started = True
    inform("ShortestPath started!")

    sp = ShortestPath(robot.explored_map, robot.direction, robot.current, robot.goal)
    sp_list = sp.shortestPath()
    sp_sequence = sp_list['trim_seq']
    sp_sequence.reverse()
    inform(sp_sequence)
    gevent.joinall([
        gevent.spawn(sp_to_goal, sp_sequence)
    ])

def sp_to_goal(sequence):

    if len(sequence) == 0:
        done_sp_to_goal()
        return False

    #evt.wait()
    # DON'T DO ALIGNMENT WHILE DOING FASTEST PATH RACE
    # do_alignment(robot.alignment())

    choice = sequence.pop()
    robot.step(choice, 9)
#     send_cmd(choice)

#     print("[Tornado | %s] sp_to_goal > %s : %s" %(time.ctime(time.time()), choice, robot.direction))
    gevent.joinall([
        gevent.spawn(sp_to_goal, sequence)
    ])

def done_sp_to_goal():
    #global started
    inform("ShortestPath done: Alignment!")
    #evt.wait()
#     send_cmd("W")
    #evt.wait()
    inform("ShortestPath done, for real!")
    #started = False
    global sp_to_goal_started
    sp_to_goal_started = False

    global f
    sys.stdout = orig_stdout
    f.close()


def start_exploration(percentage, delay):
    global robot
    global started
    global delay_time
    global exploration_started



    if exploration_started:
        return
    exploration_started = True

    if started:
        return
    global f
    if f.closed:
        f = file('log.txt', 'w')
    sys.stdout = f

    inform("Exploration started: Alignment!")
    robot = algo.realRun.Robot()

    
    delay_time = float(delay)
    robot.step(FD_AL)
    robot.step(LD_AL)
    robot.step(FD_AL)
    robot.step(FD_AL)
    robot.step(RIGHT)
    robot.step(LA_AL)
    started = True
    global sensors
    sensors = robot.receive_sensors("40,40,40,40,40,40")
    
    robot.receive_sensors("40,40,40,40,40,40")
    inform(robot.receive_sensors("40,40,40,40,40,40"))
    robot.step(RIGHT)
    robot.receive_sensors("40,40,40,40,40,40")
    robot.step(LEFT)
    robot.receive_sensors("40,40,40,40,40,40")
#     robot.update_map()
    started = False
    robot.step(LD_AL)
    robot.step(LA_AL)
    started = True
    exp = Exploration(int(percentage))
    robot.update_map()
    inform("Exploration started, for real!")
    inform(robot.current)
    inform(robot.explored_map)
    inform(robot.direction)
    inform(robot.sensors)
    inform(robot.steps)
    t1 = FuncThread(exploration, exp)
    t1.start()
    t1.join()

def exploration(exp):
    global started
    if not started:
        return False

    ###evt.wait()
    do_alignment(robot.alignment())

    global sensors
    
    
    
#     robot.receive_sensors("40,40,40,40,40,40")


    cur = exp.getRealTimeMap(sensors, robot.explored_map)
    
    if cur[1]:
        done_exploration()
        return False

    if cur[0]:
        if robot.try_left:
            ###evt.wait()
            do_alignment(robot.alignment())

        robot.step(cur[0])
        #send_cmd(cur[0])

    print("[Tornado] exploration > %s" %(robot.current))

    
    gevent.joinall([
        gevent.spawn(delay_call, exploration, exp)
    ])
    

def done_exploration():
    global started
    inform("Exploration done!")

    started = False

    inform(robot.descriptor_one())
    inform(robot.descriptor_two())
    # inform(robot.msg_for_android())

    sp = ShortestPath(robot.explored_map, robot.direction, robot.current, robot.start)
    sp_list = sp.shortestPath(-1)
    sp_sequence = sp_list['trim_seq']
    sp_sequence.reverse()
    inform(sp_sequence)

    # call sp to start
    gevent.joinall([
        gevent.spawn(sp_to_start, sp_sequence)
    ])
    

if __name__ == '__main__':
#     print("1")
    parse_command_line()
#     print(str(options.port))
    app.listen(options.port)
    robot = algo.realRun.Robot()
    old_subscribers = zope.event.subscribers[:]
    del zope.event.subscribers[:]
    zope.event.subscribers.append(tick)
    print("Listening to http://localhost:" + str(options.port) + "...")
    started = False
    t = FuncThread(tornado.ioloop.IOLoop.instance().start)

    t.start()
    t.join()
