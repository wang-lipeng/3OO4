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
log = open('log.txt', 'w')
orig_stdout = sys.stdout

clients = dict()
delay_time = 0.1
sensors = []
isStarted = False
android_ok = False
exp_done = False
io_loop = False
completeExplore = False
exploration_isStarted = False
sPath_to_goal_isStarted = False

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
        t = FuncThread(start_sPath_to_goal)
        t.start()
        self.flush()

"""stop handler for request"""
class StopHandler(tornado.web.RequestHandler):
    @tornado.web.asynchronous
    def get(self):
        global isStarted
        inform("Stop exploration...")
        isStarted = False
        self.flush()      
        global log
        sys.stdout = orig_stdout
        log.close()

"""the web application UI"""
app = tornado.web.Application([
    (r'/', IndexHandler),
    (r'/ws', WebSocketHandler),
    (r'/start/(.*)/(.*)', StartHandler),
    (r'/start_sp/', StartSPathHandler),
    (r'/stop/', StopHandler)
])


def delay_call(f, *args, **kwargs):
    f(*args, **kwargs)

def real_delay_call(f, delay_time, *args, **kwargs):
    t = threading.Timer(delay_time, f, args=args, kwargs=kwargs)
    t.start()
    
def tick(step):
    for key in clients:
        message = dict()
        message['type'] = 'map'
        message['step'] = step
        message['time'] = str(datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]) + 'Z'
        message['map'] = robot.explored_map
        clients[key]['object'].write_message(json.dumps(message))

def inform(string):
    print(string)
    for key in clients:
        message = dict()
        message['type'] = 'info'
        message['time'] = str(datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]) + 'Z'
        message['info'] = string
        clients[key]['object'].write_message(json.dumps(message))

def alignment(steps):
    global isStarted
    if not isStarted or len(steps) <= 0:
        return False
    choice = steps[0]
    steps = steps[1:]
    if choice == RIGHT or choice == LEFT:
        robot.step(choice)
    gevent.joinall([
        gevent.spawn(delay_call, alignment, steps)
    ])


def sPath_to_start(sequence):
    global isStarted

    if len(sequence) == 0:
        done_sPath_to_start()
        return False

    alignment(robot.alignment())

    choice = sequence.pop()
    robot.step(choice, -1)
    gevent.joinall([
        gevent.spawn(sPath_to_start, sequence)
    ])

def done_sPath_to_start():
    global exp_done
    inform("Gone back to start: Alignment!")
    # Calibrate first!
    if robot.direction == NORTH:
        robot.step(LEFT)
    elif robot.direction == SOUTH:
        robot.step(RIGHT)
    elif robot.direction == EAST:
        robot.step(LEFT)
        robot.step(LEFT)
    robot.step(RIGHT)
    exp_done = True
    global exploration_isStarted
    exploration_isStarted = False


def start_sPath_to_goal():
    global robot
    # # Don't "started" so the map won't update
    global sPath_to_goal_isStarted
    if sPath_to_goal_isStarted:
        return
    sPath_to_goal_isStarted = True


    if not exp_done:
        return False
    inform("ShortestPath starts!")

    sPath = ShortestPath(robot.explored_map, robot.direction, robot.current, robot.goal)
    sPath_list = sPath.shortestPath()
    sPath_sequence = sPath_list['trim_seq']
    sPath_sequence.reverse()
    inform(sPath_sequence)
    gevent.joinall([
        gevent.spawn(sPath_to_goal, sPath_sequence)
    ])

def sPath_to_goal(sequence):

    if len(sequence) == 0:
        done_sPath_to_goal()
        return False

    # DON'T DO ALIGNMENT WHILE DOING FASTEST PATH RACE
    # do_alignment(robot.alignment())

    choice = sequence.pop()
    robot.step(choice, 9)
    gevent.joinall([
        gevent.spawn(sPath_to_goal, sequence)
    ])

def done_sPath_to_goal():
    inform("ShortestPath done!")
    global sPath_to_goal_isStarted
    sPath_to_goal_isStarted = False

    global log
    sys.stdout = orig_stdout
    log.close()


def start_exploration(percentage, delay):
    global robot
    global isStarted
    global delay_time
    global exploration_isStarted
    if exploration_isStarted:
        return
    exploration_isStarted = True
    if isStarted:
        return
    global log
    if log.closed:
        log = file('log.txt', 'w')
    sys.stdout = log
    inform("Exploration starts: Alignment!")
    robot = algo.realRun.Robot()
    delay_time = float(delay)
    robot.step(FD_AL)
    robot.step(LD_AL)
    robot.step(FD_AL)
    robot.step(FD_AL)
    robot.step(RIGHT)
    robot.step(LA_AL)
    isStarted = True
    
    global sensors
    sensors = robot.receive_sensors("40,40,40,40,40,40")
    robot.receive_sensors("40,40,40,40,40,40")
    robot.step(RIGHT)
    robot.receive_sensors("40,40,40,40,40,40")
    robot.step(LEFT)
    robot.receive_sensors("40,40,40,40,40,40")
    isStarted = False
    robot.step(LD_AL)
    robot.step(LA_AL)
    isStarted = True
    exp = Exploration(int(percentage))
    robot.update_map()
    inform("Exploration starts!")
    
    #####This part is only for debugging purpose###
#     inform(robot.current)
#     inform(robot.direction)
#     inform(robot.sensors)
#     inform(robot.steps)
#     inform(robot.explored_map)
    ################################################
    
    t1 = FuncThread(explore, exp)
    t1.start()
    t1.join()

def explore(exp):
    global isStarted
    if not isStarted:
        return False

    alignment(robot.alignment())
    global sensors
    cur = exp.getRealTimeMap(sensors, robot.explored_map) 
    if cur[1]:
        done_exploration()
        return False

    if cur[0]:
        if robot.try_left:
            alignment(robot.alignment())
        robot.step(cur[0])
    print("[Tornado] exploration > %s" % (robot.current))    
    gevent.joinall([
        gevent.spawn(delay_call, explore, exp)
    ])
    

def done_exploration():
    global isStarted
    inform("Exploration done!")
    isStarted = False
    inform("Descriptor 1:")
    inform(robot.descriptor_one())
    inform("Descriptor 2:")
    inform(robot.descriptor_two())
    sPath = ShortestPath(robot.explored_map, robot.direction, robot.current, robot.start)
    sPath_list = sPath.shortestPath(-1)
    sPath_sequence = sPath_list['trim_seq']
    sPath_sequence.reverse()
    inform(sPath_sequence)
    gevent.joinall([
        gevent.spawn(sPath_to_start, sPath_sequence)
    ])
    

if __name__ == '__main__':
    parse_command_line()
    app.listen(options.port)
    robot = algo.realRun.Robot()
    old_subscribers = zope.event.subscribers[:]
    del zope.event.subscribers[:]
    zope.event.subscribers.append(tick)
    print("Listening to http://localhost:" + str(options.port) + "...")
    isStarted = False
    t = FuncThread(tornado.ioloop.IOLoop.instance().start)
    t.start()
    t.join()
