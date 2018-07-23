#! /usr/bin/python

import os
# we have rapid polling for position magnent ensure it isn't late
os.nice(-15)
import sys
import time
import functools
import numpy
from ctypes import CDLL, c_char_p

from twisted.internet import reactor, defer
from twisted.internet import task

# hall effects were measured to have a typical width of 30-35 steps
# spacing between hall effects were in the range [1290,1370] steps

FILT_MAX_DIST = 1500 # amonut of steps to drive to find next filter
STATUS_POLL = 1 #seconds
MOVE_DIR = None
MOVE_COUNTER = 0
MOVE_COUNTER_TARGET = None

### GPIO MAP
# ev* code in evgpio.h
HALL_POS = 36 # hall position bit
ID_1 = 37 # fw ID ones bit
ID_2 = 38 # fw ID twos bit
ID_4 = 39 # fw ID fours bit
DIFFU = 76 # diffuser bit

# initialization send to the stepper motor controller
# 2nd (float) value indicates the time delay imposed after
# sending the command
motorInitList = [
    ("ID", 0.1),
    ("DN", 0.1),
    ("ABS", 0.1),
    ("DOBOOT=0", 0.1),
    ("EDIO=0", 0.1),
    ("POL=4", 0.1), # are POL required? we dont use
    ("POL=6", 0.1),
    ("POL=1", 0.1),
    ("SCV=0", 0.1),
    ("IERR=1", 0.1), # changed from 1
    ("MST", 0.1),
    ("RR", 2.5),
    ("DRVRC=1500", 0.1),
    ("RW", 2),
    ("SL=1", 1), # disable sn loop
    ("SLR=25", 2.5),
    ("CLR", 0.1),
    ("LSPD=10", 0.1),
    ("HSPD=250", 0.1),
    ("ACC=70", 0.1),
    ("DEC=70", 0.1),
    ("EO=1", 0.1), #power up the motor
]

fwDIR = os.environ["ARCTICFILTERWHEEL_DIR"]
soPath = os.path.join(fwDIR,"src/device.so")

device = CDLL(soPath)
device.getUSBReply.restype = c_char_p

class Status(object):
    def __init__(self):
        self.motorMoving = None
        self.motorPos = None
        self.encPos = None
        self._wheelID = None
        self.wheelID = None
        self.inPosition = None
        self.diffuserIn = None

        self.isHomed = False
        self.targetPos = None
        self.isHoming = False
        self.homeCallback = None
        self.moveCallback = None
        self.filterID = None
        self.cmdFilterID = None
        self.isMoving = False

    @property
    def atHome(self):
        return bool(self._wheelID)

    def setWheelID(self):
        # lock it in
        self.wheelID = self._wheelID

    def addHomeCallback(self, homeCB):
        self.homeCallback = homeCB

    def addMoveCallback(self, moveCB):
        self.moveCallback = moveCB

    def update(self):
        self.motorMoving = motorStatus()!=0
        self.motorPos = motorPos()
        self.encPos = encPos()
        self._wheelID = readWheelID()
        self.inPosition = positionTriggered()
        self.diffuserIn = diffuserInPos()

    def __repr__(self):
        """blah
        """
        attrList = [
            "motorMoving",
            "motorPos",
            "encPos",
            "wheelID",
            "inPosition",
            "diffuserIn",
        ]
        statusStr="Status:\n"
        for attr in attrList:
            statusStr+="%s: %s\n"%(attr, str(getattr(self,attr)))
        return statusStr

status = Status()

def positionTriggered():
    # goes low when hall is sensed
    posBit = device.evgetin(HALL_POS)
    # print("motor state", motorStatus())
    # print("motor pos", motorPos())
    # print("enc pos", encPos())
    return posBit==0

def readWheelID():
    # return integer of filterwheel
    # return none, if sensors not triggered
    # 0 indicates bit in place so invert it
    oneBit = 1 - device.evgetin(ID_1)
    twoBit = 1 - device.evgetin(ID_2)
    fourBit = 1 - device.evgetin(ID_4)
    fwID = int("%i%i%i"%(fourBit,twoBit,oneBit),2)
    if fwID in [0,1]:
        # not in place
        return None
    return fwID

def diffuserInPos():
    # return true if diffuser is in position
    posBit = device.evgetin(DIFFU)
    return posBit==1

def setDiffuserIn():
    device.evsetdata(DIFFU, 1)

def setDiffuserOut():
    device.evsetdata(DIFFU, 0)

def motorPos():
    success = device.sendCmd("PX")
    if success == -1:
        return None
    return float(device.getUSBReply())

def encPos():
    success = device.sendCmd("EX")
    if success == -1:
        return None
    return float(device.getUSBReply())

def setPos(pos):
    # set motor and encoder pos
    success = device.sendCmd("PX=%i"%pos)
    if success == -1:
        return None
    success = device.sendCmd("EX=%i"%pos)
    if success == -1:
        return None
    return True

def motorStatus():
    success = device.sendCmd("MST")
    if success == -1:
        return None
    return int(device.getUSBReply())

def motorOn():
    cmdStr = "EO=1"
    success = device.sendCmd(cmdStr)
    return success == 1

def motorOff():
    cmdStr="EO=0"
    success = device.sendCmd(cmdStr)
    return success == 1

def stop():
    cmdStr = "STOP"
    success = device.sendCmd(cmdStr)
    ### blocks but who cares, its very short
    while motorStatus() != 0:
        pass
    return success == 1

def connect():
    device.connect()
    while True:
        out=device.commFlush()
        if out==-1:
            time.sleep(1)
        else:
            break
    for cmdStr, delay in motorInitList:
        device.sendCmd(cmdStr)
        time.sleep(delay)

def disconnect():
    device.disconnect()


def move(absPos):
    # success = motorOn()
    # if not success:
    #     return
    # joe put a CLR command here...I'm not
    cmdStr = "X%i"%absPos
    success = device.sendCmd(cmdStr)
    status.targetPos = absPos
    targetDir = 1
    if absPos < status.encPos:
        targetDir = -1
    status.targetDir = targetDir
    return success == 1

def stopNext(d):
    # stop at next sensed hall
    # must go from low to high
    # incase we start on a hall
    global GOT_LOW
    global LOOP_CALL
    GOT_LOW = False
    def checkBit():
        global GOT_LOW
        inPos = positionTriggered()
        if not GOT_LOW and not inPos:
            GOT_LOW = True
        elif GOT_LOW and inPos:
            LOOP_CALL.stop()
            stop()
            d.callback(None)
        elif motorStatus() == 0:
            try:
                LOOP_CALL.stop()
                d.callback(None) # should be error back?
            except:
                pass # may have been called if user commanded a stop
    LOOP_CALL = task.LoopingCall(checkBit)
    LOOP_CALL.start(0.02)


def checkPosition(dummy):
    global MOVE_COUNTER
    global MOVE_COUNTER_TARGET
    status.update()
    MOVE_COUNTER += 1
    if not status.inPosition:
        status.isHomed = False
        status.isHoming = False
        status.isMoving = False
        status.filterID = None
        if status.isHoming:
            status.homeCallback()
        else:
            status.moveCallback()
    elif MOVE_COUNTER < MOVE_COUNTER_TARGET:
        #another move wanted
        if status.atHome:
            setPos(0)
            if status.isHoming and status.filterID is None:
                status.setWheelID()
                status.filterID = 1
                # found home.  Now cycle wheel through
                # through all positions to make sure we
                # get all dectections
                MOVE_COUNTER = 0
                MOVE_COUNTER_TARGET = 6
        offsetFilter()
    else:
        status.isMoving = False
        if status.isHoming:
            status.isHoming = False
            if not status.atHome:
                # homing failed
                status.isHomed = False
                status.filterID = None
            else:
                status.isHomed = True
                status.filterID = 1
                status.setWheelID()
            status.homeCallback()
        else:
            # this is a move command, set filter id and callback
            filterID = status.filterID + MOVE_DIR*MOVE_COUNTER
            if filterID > 6:
                filterID -= 6
            elif filterID <= 0:
                filterID += 6
            status.filterID = filterID
            status.moveCallback()

def beginStatusLoop():
    l = task.LoopingCall(status.update)
    l.start(STATUS_POLL)

def offsetFilter():
    status.update()
    currPos = status.encPos
    nextPos = currPos + MOVE_DIR*FILT_MAX_DIST
    d = defer.Deferred()
    d.addCallback(checkPosition)
    move(nextPos)
    stopNext(d)

def home():
    global MOVE_DIR
    global MOVE_COUNTER_TARGET
    global MOVE_COUNTER
    MOVE_DIR = 1
    MOVE_COUNTER_TARGET = 7 # be sure to detect all filters
    MOVE_COUNTER = 0
    setPos(0)
    status.isHoming = True
    status.isMoving = True
    offsetFilter()

def moveToFilter(filterID):
    global MOVE_DIR
    global MOVE_COUNTER
    global MOVE_COUNTER_TARGET
    MOVE_COUNTER = 0
    status.update()
    if filterID not in [1,2,3,4,5,6]:
        return
    if not status.isHomed:
        return
    if not status.inPosition:
        return
    if status.isMoving:
        return
    status.isMoving = True
    status.cmdFilterID = filterID
    currFilter = status.filterID
    filtCounts = filterID - currFilter
    MOVE_COUNTER_TARGET = numpy.abs(filtCounts)
    MOVE_DIR = 1
    if filtCounts < 0:
        MOVE_DIR = -1
    if MOVE_COUNTER_TARGET > 2:
        MOVE_DIR = -1*MOVE_DIR
        MOVE_COUNTER_TARGET = 6 - MOVE_COUNTER_TARGET
    offsetFilter()

def setupGPIO():
    # begin masking all
    device.evgpioinit()
    for ii in range(128):
        device.evsetmask(ii,1)
    # set diffuser direction to output
    device.evsetddr(DIFFU,1)
    # unmask hall position bit (not sure it's necessary?)
    device.evsetmask(HALL_POS, 0)
    device.evclrwatch()

def init():
    connect()
    setupGPIO()
    stop()
    status.isHomed = False
    setPos(0)
    status.update()
    beginStatusLoop()


if __name__ == "__main__":
    connect()
    setupGPIO()
    setPos(0)
    status.update()
    beginStatusLoop()
    if len(sys.argv)==2:
        if sys.argv[1].lower() == "home":
            m6 = functools.partial(moveToFilter, 6)
            m6 = functools.partial(moveToFilter, 1, m6)
            m5 = functools.partial(moveToFilter, 6,m6)
            m4 = functools.partial(moveToFilter, 1,m5)
            m3 = functools.partial(moveToFilter, 2,m4)
            m2 = functools.partial(moveToFilter, 6, m3)
            m = functools.partial(moveToFilter, 3, m2)
            home(m)
        else:
            movePos = int(sys.argv[1])
            move(movePos)
            stopNext()
        reactor.run()


########### for reference using callbacks with c types, couldn't
# figure out how to work in a non blocking way
# def positionCallback(dio, value):#
# c_positionCallback = CFUNCTYPE(None, write_callback_prototype(positionCallback)
# device.evwatchin.restype = None
# CBFUNC = CFUNCTYPE(None, c_int, c_int)
# cbFunc = CBFUNC(positionCallback)
# reactor.callLater(0, device.evwatchin,cbFunc)