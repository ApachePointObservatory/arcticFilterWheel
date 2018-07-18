#!/usr/bin/env python2
from __future__ import absolute_import, division
# import subprocess


from twisted.internet import reactor

from twistedActor import startFileLogging

from arcticFilterWheel import ArcticFWActorWrapper

UserPort = 37000

try:
    startFileLogging("emulateArcticFilterWheel")
except KeyError:
   # don't start logging
   pass

if __name__ == "__main__":
    print("emulate arctic filter wheel running on port %i"%UserPort)
    arcticFWWrapper = ArcticFWActorWrapper(name="mockArcticFW", userPort=UserPort)
    reactor.run()