import datetime
import logging
import time
from enum import Enum

import numpy as np
from PyQt4 import QtCore
from numpy import linalg
from numpy.core.numeric import ndarray, array

msb = None


class Route(object):
    """a route to be simulated"""

    def __init__(self, start, goal, _id, s):
        self.sim = s

        self.id = _id
        self.state = RouteState.QUEUED

        assert start.__class__ is ndarray, 'Start needs to be a numpy.ndarray'
        self.start = start
        assert goal.__class__ is ndarray, 'Goal needs to be a numpy.ndarray'
        self.goal = goal

        self.car = None

        self.vector = goal - start
        self.distance = linalg.norm(self.vector)

        self.preVector = None
        self.preDistance = None
        self.preRemaining = None

        self.creationTime = datetime.datetime.now()

        if self.sim.msb_select:
            global msb
            from planner import msb

        logging.debug(str(self))

    def assign_car(self, _car):
        logging.debug(str(self))
        if self.car == _car:
            # nothing changed
            return
        if self.state == RouteState.QUEUED:  # starting the route
            self.car = _car
            self.state = RouteState.TO_START
            logging.debug(str(self))

            assert not _car.route, "Car should not be on another route already"
            _car.route = self

            if self.sim.msb_select:
                data = {"agvId": self.car.id, "jobId": self.id}
                msb.Msb.mwc.emit_event(msb.Msb.application, msb.Msb.eAGVAssignment, data=data)
        elif self.state == RouteState.TO_START:  # had another car already
            assert self.car, "Should have had a car"
            self.car = _car

            assert _car.route, "Car should be on another route already"
            _car.route = self
        else:
            assert False, "Can not assign car in state " + str(self.state)

    def new_step(self, stepSize):
        if not self.car:
            return
        else:  # wrk with a path ...

            i_prev = self.car.i
            self.car.i += stepSize
            i_prev_round = int(np.ceil(i_prev))
            i_next_round = int(np.floor(self.car.i))

            # e.g.

            #   1     2     3
            #       ^   ^
            #  i_prev   car.i

            # -> consider pos of t = 2

            assert i_next_round <= len(self.car.paths) + 5, "shooting far over goal"
            i_next_round = min(i_next_round, len(self.car.paths))
            if not self.is_finished():
                while self.car.__class__ == bool:
                    time.sleep(.1)
                    logging.warning("Waiting for car to be assigned")
                for _i in range(i_prev_round, i_next_round + 1):  # e.g. [2]
                    if self.car.paths[_i][0:2] == tuple(self.start):
                        self.at_start()
                    elif (self.car.paths[_i][0:2] == tuple(self.goal)) & self.is_on_route():  # @ goal
                        self.at_goal()
                    # somewhere else
                    if self.is_running():
                        self.car.setPose(np.array(self.car.paths[_i][0:2]))

        self.sim.emit(QtCore.SIGNAL("update_route(PyQt_PyObject)"), self)

        if self.sim.msb_select:
            emit_car(msb, self.car)

    def at_goal(self):
        self.car.route = None
        self.car.setPose(self.goal)
        self.car = None
        assert self.state == RouteState.ON_ROUTE, "must have been on route before"
        self.state = RouteState.FINISHED
        logging.info(str(self) + " reached Goal")
        self.sim.replan = True
        if self.sim.msb_select:
            msb.Msb.mwc.emit_event(msb.Msb.application, msb.Msb.eReached, data=self.id)

    def at_start(self):
        self.car.setPose(self.start)
        self.state = RouteState.ON_ROUTE
        self.preRemaining = 0
        logging.info(str(self) + " reached Start")
        self.sim.replan = True
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
                      (self.creationTime - datetime.datetime.now()).total_seconds()])

    def __str__(self):
        return "R%d: %s -> %s (%s) = %s" % (self.id, str(self.start), str(self.goal), str(self.state).split('.')[1], str(self.car))


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

    def setPose(self, pose):
        if self.sim.check_free(self, pose):
            self.pose = pose
            self.sim.emit(QtCore.SIGNAL("update_car(PyQt_PyObject)"), self)
            logging.info("Car " + str(self.id) + " @ " + str(self.pose))
        else:
            self.sim.replan = True
            logging.warning("Car " + str(self.id) + " BLOCKED @ " + str(self.pose))

    def setPaths(self, _paths):
        self.i = 0
        self.paths = []
        for path in _paths:
            self.paths += path

    def toTuple(self):
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
