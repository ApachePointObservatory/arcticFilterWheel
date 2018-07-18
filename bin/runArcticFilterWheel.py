#!/usr/bin/env python2
from __future__ import division, absolute_import
"""Run the Arctic Filter Wheel actor
"""
import os

from twisted.internet import reactor
from twistedActor import startSystemLogging

from arcticFilterWheel import ArcticFWActor


UserPort = 37000

if __name__ == "__main__":
    print("arcticFilterWheel running on port %i"%UserPort)
    startSystemLogging(ArcticFWActor.Facility)
    arcticFilterWheel = ArcticFWActor(name="arcticFilterWheel", userPort=UserPort)
    reactor.run()
