#!/usr/bin/env python2
from __future__ import division, absolute_import

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from twisted.trial.unittest import TestCase

from twistedActor import testUtils
testUtils.init(__file__)

from arcticFilterWheel import ArcticFWActorWrapper

class TestMirrorDeviceWrapper(TestCase):
    """Test basics of MirrorDeviceWrapper
    """
    def setUp(self):
        self.aw = ArcticFWActorWrapper(name="arcticFWActorWrapper")
        return self.aw.readyDeferred

    def tearDown(self):
        d = self.aw.close()
        return d

    def testSetUpTearDown(self):
        self.assertFalse(self.aw.didFail)
        self.assertFalse(self.aw.isDone)
        self.assertTrue(self.aw.isReady)


if __name__ == '__main__':
    from unittest import main
    main()
