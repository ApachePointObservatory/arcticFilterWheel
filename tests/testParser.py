#!/usr/bin/env python2
from __future__ import division, absolute_import

import unittest

from arcticFilterWheel.commandSet import arcticFWCommandSet

commandList = [
    "stop",
    "move 1",
    "move 4",
    "home",
    "ping",
    "init",
    "status",
]


class TestParser(unittest.TestCase):
    def testCommandList(self):
        # cmdStr = "camera"
        # parsedCommand = arcticCommandSet.parse(cmdStr)

        for cmdStr in commandList:
            print "cmdStr: ", cmdStr
            parsedCommand = arcticFWCommandSet.parse(cmdStr)


if __name__ == '__main__':
    unittest.main()