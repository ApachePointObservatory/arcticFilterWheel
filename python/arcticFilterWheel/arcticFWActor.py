from __future__ import division, absolute_import

import syslog
import collections

from twistedActor import Actor, expandUserCmd, log, UserCmd

from .commandSet import arcticFWCommandSet
from .version import __version__

# from filter import FilterWheel
from .device import moveToFilter, home, status, stop, init, setDiffuserIn, setDiffuserOut

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
        self.status = status

    def update(self):
        self.status.update()

    def addMoveCallback(self, callback):
        self.status.addMoveCallback(callback)

    def addHomeCallback(self, callback):
        self.status.addHomeCallback(callback)

    @property
    def kwMap(self):
        return collections.OrderedDict((
            ("state", self.state),
            ("wheelID", "NaN" if self.status.wheelID is None else self.status.wheelID),
            ("filterID", "NaN" if self.status.filterID is None else self.status.filterID),
            ("cmdFilterID", "NaN" if self.status.cmdFilterID is None else self.status.cmdFilterID),
            ("encoderPos", self.status.encPos),
            ("inPosition", self.status.inPosition),
            ("atHome", self.status.atHome),
            ("diffuInBeam", "?" if self.status.diffuserIn is None else self.status.diffuserIn),
        ))

    @property
    def currentPos(self):
        return self.status.filterID

    @property
    def isHomed(self):
        return self.status.isHomed

    @property
    def atHome(self):
        return self.status.atHome

    @property
    def inPosition(self):
        return self.status.inPosition

    @property
    def state(self):
        state = self.Done
        if self.status.isHoming:
            state = self.Homing
        elif not self.status.isHomed:
            state = self.NotHomed
        elif self.status.isMoving:
            state = self.Moving
        return state

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
       return self._subStatusStr(["diffuInBeam"])

    @property
    def statusStr(self):
        # todo: only output a changed status value?
        return "; ".join(["%s=%s"%(kw, str(val)) for kw, val in self.kwMap.iteritems()])

class ArcticFWActor(Actor):
    Facility = syslog.LOG_LOCAL1
    DefaultTimeLim = 5 # default time limit, in seconds
    MoveRange = range(1,7)
    # State options
    def __init__(self,
        name,
        userPort = UserPort,
        commandSet = arcticFWCommandSet,
    ):
        """!Construct an ArcticFWActor

        @param[in] name  actor name
        @param[in] userPort  int, a port on which this thing runs
        @param[in] commandSet  a twistedActor.parse.CommandSet used for command def, and parsing
        @param[in] fakeFilterWheel  bool.  If true use a fake filter wheel device, for safe testing.
        """
        self.status = ArcticFWStatus()
        self.fwHomeCmd = UserCmd()
        self.fwHomeCmd.setState(self.fwHomeCmd.Done)
        self.fwMoveCmd = UserCmd()
        self.fwMoveCmd.setState(self.fwMoveCmd.Done)
        self.status.addHomeCallback(self.homeCallback)
        self.status.addMoveCallback(self.moveCallback)
        # init the filterWheel
        # this sets self.filterWheel
        Actor.__init__(self,
            userPort = userPort,
            maxUsers = 1,
            name = name,
            version = __version__,
            commandSet = commandSet,
            )
        init()


    @property
    def filterWheelMoving(self):
        return self.status.isMoving

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
        if not self.fwHomeCmd.isDone:
            self.fwHomeCmd.setState(self.fwHomeCmd.Failed, "stop commanded")
        stop()
        userCmd.setState(userCmd.Done)
        return True

    def moveCallback(self):
        self.getStatus()
        self.writeToUsers("i", self.status.statusStr, cmd=self.fwMoveCmd)
        if self.fwMoveCmd.isDone:
            # probably cancelled by stop
            return
        if self.status.inPosition:
            self.fwMoveCmd.setState(self.fwMoveCmd.Done)
        else:
            self.fwMoveCmd.setState(self.fwMoveCmd.Failed, "Failed to stop at filter sensor")

    def homeCallback(self):
        self.getStatus()
        self.writeToUsers("i", self.status.statusStr, cmd=self.fwHomeCmd)
        if self.fwHomeCmd.isDone:
            # don't do anything (probably cancelled by a stop?)
            return
        if self.status.inPosition and self.status.atHome:
            self.fwHomeCmd.setState(self.fwHomeCmd.Done)
        else:
            self.fwHomeCmd.setState(self.fwHomeCmd.Failed, "Failed to stop at home sensor")

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
        elif desPos == self.status.currentPos:
            userCmd.setState(userCmd.Done, "Filter currently in position")
        else:
            # get a fresh status
            self.getStatus()
            self.fwMoveCmd = userCmd
            if not self.fwMoveCmd.isActive:
                self.fwMoveCmd.setState(self.fwMoveCmd.Running)
            moveToFilter(desPos)
            self.getStatus()
            self.writeToUsers("i", self.status.moveStr, cmd=userCmd)
        return True

    def cmd_diffuIn(self, userCmd):
        """Move the diffuser into the beam
        """
        setDiffuserIn()
        self.getStatus()
        self.writeToUsers("i", self.status.statusStr, cmd=userCmd)
        userCmd.setState(userCmd.Done)
        return True

    def cmd_diffuOut(self, userCmd):
        """Move the diffuser out of the beam
        """
        setDiffuserOut()
        self.getStatus()
        self.writeToUsers("i", self.status.statusStr, cmd=userCmd)
        userCmd.setState(userCmd.Done)
        return True

    def cmd_startDiffuRot(self, userCmd):
        """Move the diffuser into the beam
        """
        userCmd.setState(userCmd.Failed, 'Rotation not implemented')
        return True

    def cmd_stopDiffuRot(self, userCmd):
        """Move the diffuser out of the beam
        """
        userCmd.setState(userCmd.Failed, 'Rotation not implemented')
        return True


    def cmd_home(self, userCmd):
        log.info("%s.cmd_home(userCmd=%s)"%(self, str(userCmd)))
        # print("%s.cmd_home(userCmd=%s)"%(self, str(userCmd)))
        if False in [self.fwMoveCmd.isDone, self.fwHomeCmd]:
            userCmd.setState(userCmd.Failed, "filter wheel is moving")
        else:
            self.fwHomeCmd = userCmd
            if not self.fwHomeCmd.isActive:
                self.fwHomeCmd.setState(self.fwHomeCmd.Running)
            home()
            self.cmd_status(userCmd, setDone=False)
            return True

    def cmd_status(self, userCmd=None, setDone=True):
        """! Implement the status command
        @param[in]  userCmd  a twistedActor command with a parsedCommand attribute
        """
        log.info("%s.cmd_status(userCmd=%s)"%(self, str(userCmd)))
        userCmd = expandUserCmd(userCmd)
        self.getStatus()
        self.writeToUsers("i", self.status.statusStr, cmd=userCmd)
        if setDone:
            userCmd.setState(userCmd.Done)
        return True

    def getStatus(self):
        """! A generic status command
        @param[in] userCmd a twistedActor UserCmd or none
        """
        self.status.update()




