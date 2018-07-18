#!/usr/bin/env python2
from __future__ import division, absolute_import

import os
from arcticFilterWheel import arcticFWCommandSet

if __name__ == '__main__':
    arcticICCDir = os.getenv("ARCTICFILTERWHEEL_DIR")
    with open(os.path.join(arcticICCDir, "doc", "arcticFWCommands.html"), "w") as outfile:
        outfile.write(arcticFWCommandSet.toHTML())

