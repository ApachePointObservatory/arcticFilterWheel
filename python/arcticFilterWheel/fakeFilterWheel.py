from __future__ import division, absolute_import

import time

from RO.Comm.TwistedTimer import Timer

MoveTime = 3 # seconds for all moves

StatusDict = {
    "id": 7,
    "currentEncoder": "NaN",
    "motor": 0,
    "hall": "0000",
    "position": 0,
    "power": None,
    "desiredStep": 100,#"NaN",
    "currentStep": 501,#"NaN"
    "diffuRotating": 0,
    "diffuInBeam": 0,
}


class FilterWheel(object):
    def __init__(self):
        self.moveTimer = Timer()

    def stopMove(self):
        self.moveTimer.cancel()
        StatusDict["motor"]=0
        # StatusDict["motor"]=8

    def connect(self):
        """
        initiate filter wheel communication
        """
        pass

    def home(self):
        """
        home filter wheel and set the filter wheel id
        """
        # emulate the block
        time.sleep(3)
        # StatusDict["id"] = 1
        return StatusDict["id"]

    def stop(self):
        """
        stop filter wheel motion
        """
        return self.stopMove()

    def moveToPosition(self, pos):
        """
        move to a specific filter wheel position
        arguments:
            int pos - filter position between 1 and 6
        status:
            0 - fail
            1 - succeed
            -1 - unknown
        """
        StatusDict["position"] = int(pos)
        # StatusDict["motor"] = 1
        StatusDict["motor"] = 3
        self.moveTimer.start(MoveTime, self.stopMove)
        return self.status()

    def _setDiffuPos(self, val):
        StatusDict["diffuInBeam"] = val

    def startDiffuRot(self):
        StatusDict["diffuRotating"] = 1

    def stopDiffuRot(self):
        StatusDict["diffuRotating"] = 0

    def diffuIn(self):
        Timer(0.2, self._setDiffuPos, 1)# set on a timer to emulate move time

    def diffuOut(self):
        Timer(0.2, self._setDiffuPos, 0)# set on a timer to emulate move time

    def status(self):
        """
        return current status of filter wheel
        """
        return StatusDict

