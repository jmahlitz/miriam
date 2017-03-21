import datetime
import logging
import time
from threading import Lock
from enum import Enum

import numpy as np
from PyQt4 import QtCore
from numpy import linalg
from numpy.core.numeric import ndarray, array
#import planner.process_sim.process_test as process_test
msb = None

class Route(object):
    """a route to be simulated"""

    def __init__(self, start, goal, _id, s):
        self.lock = Lock()
        self.sim = s # typeOf SimpSim
        self.id = _id # wahrscheinlich laufende unique id
        self.state = RouteState.QUEUED

        assert start.__class__ is ndarray, 'Start needs to be a numpy.ndarray'
        self.start = start
        assert goal.__class__ is ndarray, 'Goal needs to be a numpy.ndarray'
        self.goal = goal
        self.car = None
        self.vector = goal - start
        self.distance = linalg.norm(self.vector)

        self.creation_time = datetime.datetime.now()

        if self.sim.msb_select:
            global msb
            from planner import msb

        logging.debug(str(self))

    def assign_car(self, _car):
        self.lock.acquire()
        logging.debug("Assigning a car to " + str(self))
        if self.car == _car:
            # nothing changed
            self.lock.release()
            return
        if self.state == RouteState.QUEUED:  # starting the route
            self.free_car(_car)
            self.car = _car
            self.state = RouteState.TO_START
            logging.debug(str(self))
            _car.route = self

            if self.sim.msb_select:
                data = {"agvId": self.car.id, "jobId": self.id}
                msb.Msb.mwc.emit_event(msb.Msb.application, msb.Msb.eAGVAssignment, data=data)
        elif self.state == RouteState.TO_START:  # had another car already
            self.free_car(_car)
            # assert self.car, "Should have had a car, had: " + str(self.car) + ", should get: " + str(_car)
            self.car = _car
            _car.route = self
        else:
            assert False, "Can not assign car in state " + str(self.state)
        self.lock.release()

    def free_car(self, _car):
        if _car.route:
            _car.route.state = RouteState.QUEUED  # Other route is now queued again
            if _car.route.car:
                _car.route.car = None  # not on that route any more

    def new_step(self, stepSize):
        # HOW is stepSize defined: ?
        self.lock.acquire()
        assert self.car, "Should have a car"
        #prev index = aktueller index pfad i
        i_prev = self.car.path_index
        self.car.path_index += stepSize

        i_prev_round = int(np.ceil(i_prev)) #aufrunden auf Ganzzahl
        i_next_round = int(np.floor(self.car.path_index)) #abrunden auf Ganzzahl

        assert i_next_round <= len(self.car.paths) + 5, "shooting far over goal" # check for count error
        i_next_round = min(i_next_round, len(self.car.paths) - 1)  # make sure no count error applies
        assert not self.is_finished(), "Should not be finished" # make totally sure that paths are not in finished state

        while self.car is None:
            time.sleep(.1)
            logging.warning("Waiting for car to be assigned")

        for i in range(i_prev_round, i_next_round + 1):  # from prev to next including next
            if (self.car.paths[i][0:2] == tuple(self.start)) or \
                    (tuple(self.car.pose) == tuple(self.start)): # set state at_start()
                self.at_start()
            elif ((self.car.paths[i][0:2] == tuple(self.goal)) & self.is_on_route()) or \
                    (tuple(self.car.pose) == tuple(self.start)):  # set state at_goal
                self.at_goal()
                break
            if self.is_running(): #agv is moving on its path
                if (not self.car.set_pose(np.array(self.car.paths[i][0:2]))): # if Path is Blocked retry next time
                    #self.car.path_index -= stepSize;
                    pass

        self.sim.emit(QtCore.SIGNAL("update_route(PyQt_PyObject)"), self) # wo befindet sich entsprechende Funktion ?

        if self.sim.msb_select:
            emit_car(msb, self.car) #logs current position of agv
        self.lock.release()

    def at_goal(self):
        self.car.route = None
        self.car.set_pose(self.goal)
        self.car = None
        assert self.state == RouteState.ON_ROUTE, "must have been on route before"
        self.state = RouteState.FINISHED
        logging.info(str(self) + " reached Goal")
        if self.sim.msb_select:
            msb.Msb.mwc.emit_event(msb.Msb.application, msb.Msb.eReached, data=self.id)

    def at_start(self):
        self.car.set_pose(self.start)
        self.state = RouteState.ON_ROUTE
        self.preRemaining = 0
        logging.info(str(self) + " reached Start")
        if self.sim.msb_select:
            data = {"agvId": self.car.id, "jobId": self.id}
            msb.Msb.mwc.emit_event(msb.Msb.application, msb.Msb.eReachedStart, data=data)

    def is_running(self):
        return self.state == RouteState.TO_START or self.state == RouteState.ON_ROUTE

    def is_on_route(self):
        return self.state == RouteState.ON_ROUTE

    def is_finished(self):
        return self.state == RouteState.FINISHED

    def to_job_tuple(self):
        return tuple([(self.start[0], self.start[1]),
                      (self.goal[0], self.goal[1]),
                      (datetime.datetime.now() - self.creation_time).total_seconds()])

    def __str__(self):
        return "R%d: %s -> %s (%s) = %s" % (
            self.id, str(self.start), str(self.goal), str(self.state).split('.')[1], str(self.car))


def emit_car(msb, car):
    data = {"id": car.id, "x": float(car.pose[0]), "y": float(car.pose[1])}
    logging.debug(data)
    msb.Msb.mwc.emit_event(msb.Msb.application, msb.Msb.ePose, data=data)


class Car(object):
    """an AGV to be simulated"""

    nextId = 0

    def __init__(self, s):
        self.sim = s

        # assert s.__class__ is SimpSim, "Pass the simulation object to the new car"
        # each car has a random start location
        self.pose = array([
            4, 3 + Car.nextId
            # random.randint(0, s.area.shape[0]),
            # random.randint(0, s.area.shape[1])
        ])

        self.route = False

        self.id = Car.nextId
        Car.nextId += 1

        logging.info("New car:" +
                     str(self.id) +
                     " at "
                     + str(self.pose))

        self.paths = None
        self.lock = Lock()

    def set_pose(self, pose): #move agv to new pose
        self.lock.acquire()

        if self.sim.check_free(self, pose):
            self.pose = pose
            self.sim.emit(QtCore.SIGNAL("update_car(PyQt_PyObject)"), self)
            logging.info("Car " + str(self.id) + " @ " + str(self.pose))
            #process_test.free_Map(pose[0], pose[1])

            self.lock.release()
            return True

        else:
            logging.warning("Car " + str(self.id) +" @ " + str(self.pose) + " BLOCKED @ " + str(pose))
            #process_test.block_Map(pose[0],pose[1])
            self.lock.release()
            return False

    def set_paths(self, _paths):
        self.lock.acquire()
        self.path_index = 0
        self.paths = []
        for path in _paths:
            self.paths += path
        self.lock.release()

    def to_tuple(self):
        assert len(self.pose) == 2, "A cars pose must have 2 coordinates"
        return (int(self.pose[0]),
                int(self.pose[1]))

    def __str__(self):
        return "C%d: [%.2f %.2f]" % (self.id, self.pose[0], self.pose[1])


class RouteState(Enum):
    QUEUED = 0
    TO_START = 1
    ON_ROUTE = 2
    FINISHED = 3
