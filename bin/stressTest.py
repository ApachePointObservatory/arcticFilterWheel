#!/usr/bin/env python2
from __future__ import division, absolute_import
"""Run the Arctic Filter Wheel actor
"""
from twistedActor import startSystemLogging, UserCmd
from arcticFilterWheel import ArcticFWActor
from twisted.internet import reactor
import numpy

UserPort = 37000

ITER = 0
MAXITER = 5
nextMove = 0

class ParsedCommand(object)
    def __init__(self, pos):
        self.parsedPositionalArgs = [pos]

def expandCommand():
    return UserCmd()

def fakeParse(userCmd,pos):
    userCmd.parsedCommand = ParsedCommand(pos)


if __name__ == "__main__":
    print("arcticFilterWheel running on port %i"%UserPort)
    startSystemLogging(ArcticFWActor.Facility)
    arcticFilterWheel = ArcticFWActor(name="arcticFilterWheel", userPort=UserPort)


    def beginCycle(userCmd):
        global ITER
        global nextMove
        if not userCmd.isDone:
            return
        ITER += 1
        if ITER > MAXITER:
            reactor.stop()
            print("done")
            return

        print("move result: %i %i %s %s"%(ITER, nextMove, str(arcticFilterWheel.status.status._wheelID), str(userCmd.didFail)))
        if not arcticFilterWheel.status.isHomed:
            home()
            return
        while True:
            tmp = numpy.random.randint(1,7)
            if tmp != nextMove:
                nextMove = tmp
                break
        nextCmd = expandCommand()
        fakeParse(nextCmd, nextMove)
        arcticFilterWheel.cmd_move(nextCmd)
        nextCmd.addCallback(beginCycle)

    def home():
        print("homing")
        homeCmd = expandCommand()
        arcticFilterWheel.cmd_home(homeCmd)
        homeCmd.addCallback(beginCycle)

    reactor.callLater(2,home)

    reactor.run()
