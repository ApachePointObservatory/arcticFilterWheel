from __future__ import division, absolute_import

import syslog
import collections

import numpy

from RO.Comm.TwistedTimer import Timer

from twistedActor import Actor, expandUserCmd, log, UserCmd

from .commandSet import arcticFWCommandSet
from .version import __version__

# from filter import FilterWheel
from .device import moveToFilter, home, status, connect, stop

from .fakeFilterWheel import FilterWheel as FakeFilterWheel

UserPort = 37000

IN_BEAM = 1
OUT_OF_BEAM = 0
DIFFU_TIMEOUT = 5 # 5 second timeout to move in and out of beam?



class ArcticFWStatus(object):
    Homing = "Homing"
    NotHomed = "NotHomed"
    Moving = "Moving"
    Done = "Done"
    def __init__(self):
        self.id = 0
        self.currentEncoder = "NaN"
        self.motor = False
        self.hall = "?"
        self.position = 0
        # self.power = "?"
        self.isHomed = False
        self.isHoming = False
        self.desiredStep = "NaN"
        self.currentStep = "NaN"
        self._cmdFilterID = "NaN"
        self.diffuInBeam = 0
        self.diffuRotating = 0
        self.lastFoundFilter = [None, None] # [filterID, encValue@mag]
        self.filterDelta = None

    def getFilterPosList(self):
        if None in self.lastFoundFilter:
            zeroOffset = 0
        else:
            filterID, filterPos = self.lastFoundFilter
            zeroOffset = filterPos - (filterID - 1)*self.filterDelta
        positionList = [zeroOffset+i*self.filterDelta for i in range(6)]
        return positionList

    @property
    def kwMap(self):
        return collections.OrderedDict((
            ("state", self.state),
            ("wheelID", self.id),
            ("filterID", self.currFilterID),
            ("cmdFilterID", "NaN" if self.cmdFilterID is None else self.cmdFilterID),
            ("encoderPos", self.currentEncoder),
            ("hall", self.hall),
            ("desiredStep", "NaN" if self.desiredStep is None else self.desiredStep),
            ("currentStep", "NaN" if self.currentStep is None else self.currentStep),
            ("diffuInBeam", "?" if self.diffuInBeam is None else self.diffuInBeam),
            ("diffuRotating", "?" if self.diffuRotating is None else self.diffuRotating),
        ))


    @property
    def state(self):
        state = self.Done
        if self.isHoming:
            state = self.Homing
        elif not self.isHomed:
            state = self.NotHomed
        elif self.motor:
            state = self.Moving
        return state

    @property
    def currFilterID(self):
        # determine filterID based on ... math!
        # now that i'm commanding low level moves, i
        # don't think the controller knows which filter is in place

        # if not homed return "NaN", else position
        if self.state != self.Done:
            return "NaN"
        if not self.hallInPos:
            return "NaN"
        if self.filterDelta is None:
            return "NaN"
        # determine filter by finding the nearest filter id in the
        # filter position list
        filterPosList = numpy.array(self.getFilterPosList())
        print('filterPosList', filterPosList)
        return numpy.argmin(numpy.abs(self.currentEncoder-filterPosList)) + 1

    @property
    def cmdFilterID(self):
        if self._cmdFilterID == "NaN" and not True in [self.motor, self.isHoming] and self.isHomed:
            # asked for commanded filter but currently set to NaN, everything looks fine
            # set it to commanded ID
            return self.position
        else:
            return self._cmdFilterID # may return none

    def setCmdFilterID(self, value):
        self._cmdFilterID = value

    def _subStatusStr(self, kwList):
        """Return status of only kw in kwList
        """
        statusList = []
        for kw in list(kwList):
            statusList.append("%s=%s"%(kw, str(self.kwMap[kw])))
        return "; ".join(statusList)

    @property
    def moveStr(self):
        return self._subStatusStr(["state", "cmdFilterID"])

    @property
    def diffuStr(self):
       return self._subStatusStr(["diffuInBeam", "diffuRotating"])

    @property
    def statusStr(self):
        # todo: only output a changed status value?
        return "; ".join(["%s=%s"%(kw, str(val)) for kw, val in self.kwMap.iteritems()])

    @property
    def hallInPos(self):
        """Return true if the magnent is lined up
        with sensor (a filter is in position)
        """
        if self.hall[0] == "0":
            return True
        else:
            return False

class ArcticFWActor(Actor):
    Facility = syslog.LOG_LOCAL1
    DefaultTimeLim = 5 # default time limit, in seconds
    PollTime = 0.01 # seconds (go faster!)
    MoveRange = range(1,7)
    # State options
    def __init__(self,
        name,
        userPort = UserPort,
        commandSet = arcticFWCommandSet,
        fakeFilterWheel = False,
    ):
        """!Construct an ArcticFWActor

        @param[in] name  actor name
        @param[in] userPort  int, a port on which this thing runs
        @param[in] commandSet  a twistedActor.parse.CommandSet used for command def, and parsing
        @param[in] fakeFilterWheel  bool.  If true use a fake filter wheel device, for safe testing.
        """
        self.status = ArcticFWStatus()
        self.fwMoveCmd = UserCmd()
        self.fwMoveCmd.setState(self.fwMoveCmd.Done)
        self.diffuMoveCmd = UserCmd()
        self.diffuMoveCmd.setState(self.diffuMoveCmd.Done)
        self.diffuTarg = self.status.diffuInBeam # probably initializes as None.
        self.pollTimer = Timer()
        # range of encValues within which to look for a magnent during a move
        self.magSearchRange = [None, None]
        if fakeFilterWheel:
            self.FilterWheelClass = FakeFilterWheel
        else:
            self.FilterWheelClass = FilterWheel
        self.filterWheel = self.FilterWheelClass()
        self.filterWheel.connect()
        # init the filterWheel
        # this sets self.filterWheel
        Actor.__init__(self,
            userPort = userPort,
            maxUsers = 1,
            name = name,
            version = __version__,
            commandSet = commandSet,
            )
        self.init(getStatus=False)

    @property
    def filterDelta(self):
        """encoder pos
        """
        return self.filterWheel.filterDelta

    @property
    def filterSearchRange(self):
        """Identify a range about a filter
        to look for the magnet within.

        encoder distance between filters is self.filterDelta
        so choose something significantly less than that
        to ensure we don't stop on the wrong magnent
        """
        # how about 20 percent of that distance?
        return self.filterDelta * 0.2

    def filterAbsPos(self, filterID):
        """filterID is int 1-6
        warning, this is known to drift as the zero point drifts
        """
        return (filterID-1)*self.filterDelta


    @property
    def filterWheelMoving(self):
        return self.status.motor != 0

    def init(self, userCmd=None, getStatus=True, timeLim=DefaultTimeLim):
        """! Initialize all devices, and get status if wanted
        @param[in]  userCmd  a UserCmd or None
        @param[in]  getStatus if true, query all devices for status
        @param[in]  timeLim
        """
        userCmd = expandUserCmd(userCmd)
        log.info("%s.init(userCmd=%s, timeLim=%s, getStatus=%s)" % (self, userCmd, timeLim, getStatus))
        if getStatus:
            self.cmd_status(userCmd) # sets done
        else:
            userCmd.setState(userCmd.Done)
        return userCmd

    def cmd_init(self, userCmd):
        """! Implement the init command
        @param[in]  userCmd  a twistedActor command with a parsedCommand attribute
        """
        log.info("%s.cmd_init(userCmd=%s)"%(self, str(userCmd)))
        # print("%s.cmd_init(userCmd=%s)"%(self, str(userCmd)))
        self.init(userCmd, getStatus=True)
        # userCmd.setState(userCmd.Done)
        return True

    def cmd_ping(self, userCmd):
        """! Implement the ping command
        @param[in]  userCmd  a twistedActor command with a parsedCommand attribute
        """
        log.info("%s.cmd_ping(userCmd=%s)"%(self, str(userCmd)))
        # print("%s.cmd_ping(userCmd=%s)"%(self, str(userCmd)))
        userCmd.setState(userCmd.Done, textMsg="alive")
        return True

    def cmd_stop(self, userCmd):
        log.info("%s.cmd_stop(userCmd=%s)"%(self, str(userCmd)))
        # print("%s.cmd_stop(userCmd=%s)"%(self, str(userCmd)))
        if not self.fwMoveCmd.isDone:
            self.fwMoveCmd.setState(self.fwMoveCmd.Failed, "stop commanded")
        self.filterWheel.stop()
        userCmd.setState(userCmd.Done)
        return True

    def cmd_move(self, userCmd):
        desPos = int(userCmd.parsedCommand.parsedPositionalArgs[0])
        log.info("%s.cmd_move(userCmd=%s) desPos: %i"%(self, userCmd, desPos))
        # print("%s.cmd_move(userCmd=%s) desPos: %i"%(self, userCmd, desPos))
        if desPos not in self.MoveRange:
            # raise ParseError("desPos must be one of %s for move command"%(str(self.MoveRange),))
            userCmd.setState(userCmd.Failed, "desPos must be one of %s for move command"%(str(self.MoveRange),))
        elif not self.status.isHomed:
            userCmd.setState(userCmd.Failed, "cannot command move, home filter wheel first.")
        elif not self.fwMoveCmd.isDone:
            userCmd.setState(userCmd.Failed, "filter wheel is moving")
        else:
            # get a fresh status
            self.getStatus()
            self.fwMoveCmd = userCmd
            if not self.fwMoveCmd.isActive:
                self.fwMoveCmd.setState(self.fwMoveCmd.Running)
            self.status.setCmdFilterID(desPos)
            currentPosition = self.status.currentEncoder
            if None in self.status.lastFoundFilter:
                # no 'found' filter recored do a move based
                # on homed solution
                desEncoderPos = self.filterAbsPos(desPos)
            else:
                # determine move position based on last
                # 'found filter' this should be safe from
                # zeropoint drifting overtime
                lastFilter, lastEncPos = self.status.lastFoundFilter
                filtIdDelta = desPos - lastFilter
                # if filtIDdelta is negative move to a lower
                # absolute value than the previously saved one...
                desEncoderPos = lastEncPos + filtIdDelta*self.filterDelta
            self.magSearchRange = [
                int(desEncoderPos - 0.5*self.filterSearchRange),
                int(desEncoderPos + 0.5*self.filterSearchRange),
                ]
            # set the target move position to the far end of the
            # search range.
            if self.magSearchRange[0] < currentPosition < self.magSearchRange[1]:
                # command was to move to the current filter?
                if self.status.hallInPos:
                    self.writeToUsers("i", 'text="commanded filter position is current filter position"', cmd=userCmd)
                    userCmd.setState(userCmd.Done)
                    return True
                else:
                    # we are currently in move range but hall isn't registering...
                    # best thing to try is to move it closer to the middle of the range?
                    pass
            else:
                # modify the desired positon such that the move will go through the whole
                # range and stop at the far edge, the move routine *should* detect a magnent
                if desEncoderPos - currentPosition > 0:
                    # a positive offset, choose the more positive value
                    desEncoderPos = self.magSearchRange[1]
                else:
                    desEncoderPos = self.magSearchRange[0]
            print("search range", self.magSearchRange)
            print("command move from ", currentPosition, "to ", desEncoderPos)
            self.filterWheel.moveArb(desEncoderPos)
            self.getStatus()
            self.writeToUsers("i", self.status.moveStr, cmd=userCmd)
            self.pollStatus()
        return True

    def _diffuMove(self, diffuTarg, userCmd):
        self.diffuTarg = diffuTarg
        self.diffuMoveCmd = userCmd
        self.diffuMoveCmd.setTimeLimit(DIFFU_TIMEOUT)
        if not self.diffuMoveCmd.isActive:
            self.diffuMoveCmd.setState(self.diffuMoveCmd.Running)
        if diffuTarg == IN_BEAM:
            self.filterWheel.diffuIn()
        else:
            assert diffuTarg == OUT_OF_BEAM
            self.filterWheel.diffuOut()
        # self.getStatus()
        # self.writeToUsers("i", self.status.diffuStr, cmd=userCmd)
        self.pollStatus()
        return True

    def cmd_diffuIn(self, userCmd):
        """Move the diffuser into the beam
        """
        print("move diffuser in")
        return self._diffuMove(IN_BEAM, userCmd)

    def cmd_diffuOut(self, userCmd):
        """Move the diffuser out of the beam
        """
        return self._diffuMove(OUT_OF_BEAM, userCmd)

    def cmd_startDiffuRot(self, userCmd):
        """Begin rotating the diffuser
        assume returns immediately with rotating status
        """
        self.filterWheel.startDiffuRot()
        self.getStatus()
        self.writeToUsers("i", self.status.diffuStr, cmd=userCmd)
        userCmd.setState(userCmd.Done)

    def cmd_stopDiffuRot(self, userCmd):
        """Stop rotating the diffuser
        assume returns immediately with rotating status
        """
        self.filterWheel.stopDiffuRot()
        self.getStatus()
        self.writeToUsers("i", self.status.diffuStr, cmd=userCmd)
        userCmd.setState(userCmd.Done)

    def cmd_home(self, userCmd):
        log.info("%s.cmd_home(userCmd=%s)"%(self, str(userCmd)))
        # print("%s.cmd_home(userCmd=%s)"%(self, str(userCmd)))
        self.status.isHoming = 1
        # self.writeToUsers("i", "isHoming=1", cmd=userCmd)
        # send out status (basically announce I'm homing)
        self.cmd_status(userCmd, setDone=False)
        # self.homeCmd = userCmd
        Timer(0.01, self.startHome, userCmd) # on a timer to make sure state is output before block
        return True

    def startHome(self, userCmd):
        self.filterWheel.home() # blocks
        self.status.filterDelta = self.filterDelta
        self.status.isHomed = 1
        self.status.isHoming = 0
        self.cmd_status(userCmd, setDone=True) # return full status after a home

    def cmd_status(self, userCmd=None, setDone=True):
        """! Implement the status command
        @param[in]  userCmd  a twistedActor command with a parsedCommand attribute
        """
        log.info("%s.cmd_status(userCmd=%s)"%(self, str(userCmd)))
        # print("%s.cmd_status(userCmd=%s)"%(self, str(userCmd)))
        # statusStr = self.getCameraStatus()
        # self.writeToUsers("i", statusStr, cmd=userCmd)
        userCmd = expandUserCmd(userCmd)
        self.getStatus()
        self.writeToUsers("i", self.status.statusStr, cmd=userCmd)
        if setDone:
            userCmd.setState(userCmd.Done)
        return True

    def pollStatus(self):
        """Begin continuously filter wheel for status, to determine when a pending move command is done.
        Only poll while filter wheel or diffuser is moving in or out of beam.
        """
        self.getStatus()
        if self.filterWheelMoving and not self.fwMoveCmd.isDone:
            # filter wheel is moving and filter wheel command is not done
            # (i think there should always be an active fw cmd when the filter
            # wheel is moving but whatever, the check is probably harmless)

            # if we are in the target move range, begin checking to see
            # if we see a hall effect, and if so command a fw stop
            inRange = self.magSearchRange[0] < self.status.currentEncoder < self.magSearchRange[1]
            print("currPos", self.status.currentEncoder, "inPos", inRange, "hall", self.status.hallInPos)
            if inRange:
                print("move in range")
            if inRange and self.status.hallInPos:
                print("stopping fw! in range!")
                self.filterWheel.stop()
        if not self.filterWheelMoving and not self.fwMoveCmd.isDone:
            # filter wheel has stopped moving but command is not yet
            # set to done
            log.info("current move done")
            # print("current move done")
            # check for any error condition in move before setting done
            inRange = self.magSearchRange[0] < self.status.currentEncoder < self.magSearchRange[1]
            if not inRange:
                self.status.isHomed = 0
                self.fwMoveCmd.setState(self.fwMoveCmd.Failed, "Commanded move didn't stop in desired range!")
            elif not self.status.hallInPos:
                # hall effect not active, we are out of position, fail the cmd.
                self.status.isHomed = 0
                self.fwMoveCmd.setState(self.fwMoveCmd.Failed, "Motor Step Err > 500, home filter wheel")
            else:
                # no error detected report status and set done
                # set move done first then command a full status
                # so that state reports Done as
                self.status.lastFoundFilter = [self.status.cmdFilterID, self.status.currentEncoder]
                self.fwMoveCmd.setState(self.fwMoveCmd.Done)
            self.writeToUsers("i", self.status.statusStr)
        if not self.diffuMoveCmd.isDone:
            # we're waiting for the diffuser to move in or out of the beam.
            if self.diffuTarg == self.status.diffuInBeam:
                # diffuser has moved to expected position set the command done
                self.writeToUsers("i", self.status.diffuStr, cmd=self.diffuMoveCmd)
                self.diffuMoveCmd.setState(self.diffuMoveCmd.Done)
        if not self.diffuMoveCmd.isDone or self.filterWheelMoving:
            # something is still moving, continue polling
            self.pollTimer.start(self.PollTime, self.pollStatus)

    def getStatus(self):
        """! A generic status command
        @param[in] userCmd a twistedActor UserCmd or none
        """
        print("status values:")
        for key, val in self.filterWheel.status().iteritems():
            if key == 'hall':
                print("hall", val)
            if key == 'currentEncoder':
                print("enc", val)
            if key == "position":
                val += 1 # filter wheel is zero indexed
            elif key == "motor":
                # motor status values 1-7 indicate motion: see
                # ACE-SDE_Manual_Rev_1.22-2.pdf pg 38 for full bit field
                # eventually add code to detect states when motor value > 7?
                # for now treat 0 as not moving, 1-7 as moving.
                # although I think 7 sould be an impossible state (accel + decel?)
                # whatever.
                if val > 7:
                    self.writeToUsers("w", "text=motor status > 7 detected!")
                bitField = bin(val)[2:][::-1] # remove leading "0b" and reverse remaining bit field so lowest bit is first
                # add zeros to bitField until it is at least 3 characters long (first 3 bits indicate motion)
                while len(bitField) < 3:
                    bitField += "0"
                moving = False
                for bit in bitField[:3]:
                    if bit == "1":
                        moving = True
                        break
                val = int(moving) # overwrite value to be be 0 or 1
                    # loop from lowest to hightest bit (reverse)
            setattr(self.status, key, val)





