#!/usr/bin/env python2
from __future__ import division, absolute_import
"""Run the Arctic Filter Wheel actor
"""
import sys

import os
os.nice(-20)

from twisted.internet import reactor
from twisted.internet import task
import time

from filter import FilterWheel

poslims = [-2147483648, 2147483647]
stepsPerRev = 8000

if __name__ == "__main__":
    fwNum = int(sys.argv[1])
    filePath = "/root/fwMeas/fwScan_%i.txt"%fwNum
    fw = FilterWheel()
    startTime = None
    targetStep = 40000
    moveIter = 0
    maxIter = 40
    currentStep = 0
    motorState = 1
    tArr = [None]*600000
    motorArr = [None]*600000
    hallArr = [None]*600000
    currStepArr = [None]*600000
    currentEncoderArr = [None]*600000
    tStatusArr = [None]*600000
    arrayInd = 0

    def writeFile():
        with open(filePath, "w") as ff:
            ff.write("#time, motor, hall, step, enc, statDTime\n")
            for a,b,c,d,e,f in zip(tArr,motorArr,hallArr,currStepArr,currentEncoderArr,tStatusArr):
                if a is None:
                    break
                ff.write("%.4f %i %s %i %i %.4f\n"%(a,b,c,d,e,f))


    def initfw():
        fw.connect()

    def end():
        reactor.stop()
        fw.disconnect()
        writeFile()

    def zerofw():
        global startTime
        startTime = time.time()
        fw.zero()

    def printStatus():
        global startTime
        global currentStep
        global motorState
        global tArr
        global motorArr
        global hallArr
        global currStepArr
        global currentEncoderArr
        global tStatusArr
        global arrayInd
        tnow = time.time()
        t = tnow - startTime
        status = fw.status()
        ttook = time.time() - tnow
        currentStep = status['currentStep']
        motorState = status['motor']
        # print("%.4f %i %s %i %i %.4f"%(t, motorState, status['hall'], status['currentStep'], status['currentEncoder'], ttook))
        tArr[arrayInd] = t
        motorArr[arrayInd] = motorState
        hallArr[arrayInd] = status['hall']
        currStepArr[arrayInd] = currentStep
        currentEncoderArr[arrayInd] = status['currentEncoder']
        tStatusArr[arrayInd] = ttook
        arrayInd = arrayInd + 1

    def checkMove():
        global currentStep
        global targetStep
        global motorState
        global moveIter
        if currentStep == targetStep and motorState == 0:
            # reverse move direction
            moveIter += 1
            if moveIter > maxIter:
                end()
                return
            if currentStep == 40000:
                targetStep = 0
            else:
                targetStep = 40000
            print("move iter %i"%moveIter)
            reactor.callLater(3, fw.moveArb, (str(targetStep)))


    def beginLooping():
        l = task.LoopingCall(printStatus)
        l.start(0.02)

    def checkMoveLoop():
        # every 5 seconds check that move is done
        l = task.LoopingCall(checkMove)
        l.start(5)

    def movefw():
        print("begin move")
        out = fw.moveArb(str(targetStep))
        checkMoveLoop()


    reactor.callLater(1, initfw)
    reactor.callLater(14, zerofw)
    reactor.callLater(15, beginLooping)
    reactor.callLater(16, movefw)
    reactor.run()
