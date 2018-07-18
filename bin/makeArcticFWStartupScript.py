#!/usr/bin/env python2
from __future__ import division, absolute_import
"""Make a arcticFilterWheel startup script for a specified telescope

$ setup arcticICC
$ sudo makeArcticFWStartupScript.py >/usr/local/bin/arcticICC
$ sudo chmod +x /usr/local/bin/arcticICC
"""
import syslog

from twistedActor import makeStartupScript

import arcticFilterWheel

if __name__ == '__main__':
    startupScript = makeStartupScript(
        actorName="arcticFilterWheel",
        pkgName="arcticFilterWheel",
        binScript="runArcticFilterWheel.py",
        userPort=arcticFilterWheel.UserPort,
        facility=syslog.LOG_LOCAL1,

    )
    print startupScript
