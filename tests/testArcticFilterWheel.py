#!/usr/bin/env python2
from __future__ import division, absolute_import

from twisted.trial.unittest import TestCase
from twisted.internet.defer import Deferred

from twistedActor import testUtils, UserCmd

testUtils.init(__file__)

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")

from arcticFilterWheel import ArcticFWActorWrapper

class TestArcticFilterWheel(TestCase):
    """Tests for each command, and how they behave in collisions
    """
    def setUp(self):
        # self.name = "arctic"
        print 'setup'
        self.aw = ArcticFWActorWrapper(
            name="arcticFWActorWrapper",
        )
        d = self.aw.readyDeferred
        return d

    @property
    def arcticFWActor(self):
        return self.aw.actor

    def fakeHome(self):
        self.arcticFWActor.status.isHomed = 1

    def commandActor(self, cmdStr, shouldFail=False):
        d = Deferred()
        cmd = UserCmd(cmdStr=cmdStr)
        def fireDeferred(cbCmd):
            if cbCmd.isDone:
                d.callback("done")
        def checkCmdState(cb):
            self.assertTrue(shouldFail==cmd.didFail)
        cmd.addCallback(fireDeferred)
        d.addCallback(checkCmdState)
        self.arcticFWActor.parseAndDispatchCmd(cmd)
        return d

    def tearDown(self):
        print 'shutdown'
        return self.aw.close()

    def testNothing(self):
        print "testNothing"
        pass

    def testPing(self):
        print "testPing"
        return self.commandActor(cmdStr="ping")

    def testStatus(self):
        print "testStatus"
        return self.commandActor(cmdStr="status")

    def testInit(self):
        print "testInit"
        return self.commandActor(cmdStr="init")

    def testHome(self):
        print "testHome"
        return self.commandActor(cmdStr="home")

    def testMove1(self):
        print "testMove1"
        self.fakeHome()
        d = self.commandActor(cmdStr="move 1")
        def checkPos(cb):
            self.assertTrue(self.arcticFWActor.status.position==1)
        d.addCallback(checkPos)
        return d

    def testMove5(self):
        print "testMove5"
        self.fakeHome()
        d = self.commandActor(cmdStr="move 5")
        def checkPos(cb):
            self.assertTrue(self.arcticFWActor.status.position==5)
        d.addCallback(checkPos)
        return d

    def testMove15(self):
        print "testMove15"
        prevPos = self.arcticFWActor.status.position
        d = self.commandActor(cmdStr="move 15", shouldFail=True)
        def checkPos(cb):
            self.assertTrue(self.arcticFWActor.status.position==prevPos)
        d.addCallback(checkPos)
        return d

    def testStop(self):
        print "test stop"
        return self.commandActor(cmdStr="stop")


if __name__ == '__main__':
    from unittest import main
    main()