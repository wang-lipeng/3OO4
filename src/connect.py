import tornado.ioloop
import tornado.web
import tornado.websocket
import threading
from functools import wraps
from tornado.options import define, options, parse_command_line

import datetime
import json
import zope.event
import random
import sys

from algo.constants import *
import algo.simulate
from algo.explore import Exploration
from algo.shortestPath import ShortestPath

"""record exploration path for examine purpose"""
log = open('log.txt', 'w')
orig_stdout = sys.stdout

clients = dict()
delay_time = 0.1

started = False
completeExplore = False

define("port", default=8888, help="run on the defined port", type=int)

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

def delay_call(log, *args, **kwargs):
    global delay_time
    t = threading.Timer(delay_time, log, args=args, kwargs=kwargs)
    t.start()

"""execute exploration function"""
def execute_exploration(percentage, delay):
    global robot
    global started
    global delay_time
    global log
    global sensors
    
    robot = algo.simulate.Robot()
    if started:
        return
    started = True
    delay_time = float(delay)    
    if log.closed:
        log = open('log.txt', 'w')
    sys.stdout = log
    sensors = robot.get_sensors()

    robot.get_sensors()
    robot.step(RIGHT)
    robot.get_sensors()
    robot.step(LEFT)
    robot.get_sensors()

    explore = Exploration(int(percentage))

    t = FuncThread(exploration, explore)
    t.start()
    t.join()

    inform("Start to explore...")

"""execute shortest path function"""
def execute_sPath():
    global robot
    if not completeExplore:
        return False
    sPath = ShortestPath(robot.explored_map, robot.direction, robot.current, robot.goal)
    sPath_list = sPath.shortestPath()
    sPath_sequence = sPath_list['trim_seq']
    sPath_sequence.reverse() 
    inform(sPath_sequence)
    delay_call(sPath_to_goal, sPath_sequence)

    inform("Start shortest path...")

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
        execute_exploration(percentage, delay)
        self.flush()

"""start handler for shortest path"""
class StartSPathHandler(tornado.web.RequestHandler):
    @tornado.web.asynchronous
    def get(self):
        self.write("Starting...")
        execute_sPath()
        self.flush()

"""stop handler for request"""
class StopHandler(tornado.web.RequestHandler):
    @tornado.web.asynchronous
    def get(self):
        global started
        inform("Stop exploration...")
        started = False
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

def tick(step):
    for key in clients:
        message = dict()
        message['type'] = 'map'
        message['step'] = step
        message['time'] = str(datetime.datetime.utcnow())
        message['map'] = robot.explored_map
        clients[key]['object'].write_message(json.dumps(message))

def inform(string):
    print(string)
    for key in clients:
        message = dict()
        message['type'] = 'info'
        message['time'] = str(datetime.datetime.utcnow())
        message['info'] = string
        clients[key]['object'].write_message(json.dumps(message))

"""shortest path from goal to start"""
def sPath_to_start(sequence):
    global started
    global completeExplore
    if not started:
        return False
    if len(sequence) == 0:
        completeExplore = True
        inform("Return to start...")
        return False
    choice = sequence.pop()
    robot.step(choice, -1)
    print(choice, ': ', robot.direction)
    delay_call(sPath_to_start, sequence)

"""shortest path from start to goal"""
def sPath_to_goal(sequence):
    global started
#     if not started:
#         return False
    if len(sequence) == 0:
        inform("Complete shortestPath...")
        started = False
        return False
    choice = sequence.pop()
    robot.step(choice, 9)
    print(choice, ': ', robot.direction)
    delay_call(sPath_to_goal, sequence)
    
def exploration(explore):
    global started
    if not started:
        return False

    global sensors
    cur = explore.getRealTimeMap(sensors, robot.explored_map)
    if not cur[1]:
        robot.step(cur[0])
        print(robot.current)
        sensors = robot.get_sensors()
        delay_call(exploration, explore)
    else:
        inform("Complete exploration...")
        inform("Descriptor1:")
        inform(robot.descriptor_one())
        inform("Descriptor2:")
        inform(robot.descriptor_two())

        sPath = ShortestPath(robot.explored_map, robot.direction, robot.current, robot.start)
        sPath_list = sPath.shortestPath(-1)
        sPath_sequence = sPath_list['trim_seq']
        sPath_sequence.reverse() 
        inform(sPath_sequence)

        delay_call(sPath_to_start, sPath_sequence)

if __name__ == '__main__':
    parse_command_line()
    app.listen(options.port)
    robot = algo.simulate.Robot()
    old_subscribers = zope.event.subscribers[:]
    del zope.event.subscribers[:]
    zope.event.subscribers.append(tick)
    print("Listening to http://localhost:" + str(options.port) + "...")
    t = FuncThread(tornado.ioloop.IOLoop.instance().start)
    t.start()
    t.join()
