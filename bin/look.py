import numpy
import matplotlib.pyplot as plt


def parsefile(filename):
    meas = []
    with open(filename, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            t,m,h,s,e,t2 = line.split()
            inPosition = 0
            if h[0]=="0":
                inPosition = 1
            fwID = -1
            if "0" in h[1:]:
                invStr = ""
                for bit in h[::-1][:3]:
                    bitStr = str(1-int(bit))
                    invStr += bitStr
                fwID = int(invStr,2)

            l = [
                float(t),
                int(m),
                inPosition,
                fwID,
                int(s),
                int(e),
                float(t2)
                ]
            meas.append(l)
    return numpy.asarray(meas)

def parsefile_old(filename):
    startRead = False
    meas = []
    with open(filename, "r") as f:
        for line in f:
            if "move output" in line:
                startRead = True
            elif startRead:
                t2 = None
                try:
                    t,m,h,s,e,t2 = line.split()
                except:
                    t,m,h,s,e = line.split()
                inPosition = 0
                if h[0]=="0":
                    inPosition = 1
                fwID = -1
                if "0" in h[1:]:
                    invStr = ""
                    for bit in h[::-1][:3]:
                        bitStr = str(1-int(bit))
                        invStr += bitStr
                    fwID = int(invStr,2)
                l = [
                    float(t),
                    int(m),
                    inPosition,
                    fwID,
                    int(s),
                    int(e),
                    ]
                if t2 is not None:
                    l.append(float(t2))
                meas.append(l)
    return numpy.asarray(meas)

hallWidth = []

class HallDetection(object):
    def __init__(self, wheelID):
        self.wheelID = wheelID
        self.timeList = []
        self.motorList = []
        self.encList = []
        self.idList = []
        self.priorTime = None
        self.postTimes = []
        self.stepsPerRev = None
        self.homeZero = None
        self._wrapEncList = None
        self._wrapPosition = None
        self._angPositionList = None
        self.id = None

    def removeDrift(self,p):
        self.encListD = numpy.array(self.encList) - (p[1] + p[0]*numpy.array(self.timeList))

    def append(self, time, motor, enc, fwid):
        self.timeList.append(time)
        self.motorList.append(motor)
        self.encList.append(enc)
        self.idList.append(fwid)

    @property
    def time(self):
        return numpy.median(self.timeList)

    @property
    def wrapEncList(self):
        if self._wrapEncList is None:
            posOffset = self.position - self.wrapPosition
            self._wrapEncList = [x-posOffset for x in self.encList]
        return self._wrapEncList


    @property
    def wrapPosition(self):
        # might have fucked this up
        # need to subtract zero position then wrap?
        if self._wrapPosition is None:
            fwOffset = self.stepsPerRev/6.0
            fwRange = [
                -fwOffset/2.0, # some buffer lower
                self.stepsPerRev - fwOffset/2.0
                ]
            pos = self.position - self.homeZero

            while pos < fwRange[0]:
                pos += self.stepsPerRev
            while pos > fwRange[1]:
                pos -= self.stepsPerRev

            self._wrapPosition = pos
        return self._wrapPosition

    @property
    def angPosition(self):
        return numpy.median(self.angPositionList)

    @property
    def angPositionList(self):
        if self._angPositionList is None:
            self._angPositionList = [x/float(self.stepsPerRev)*360.0 for x in self.wrapEncList]
        return self._angPositionList

    @property
    def position(self):
        return numpy.median(self.encList)

    @property
    def encWidth(self):
        if len(self.encList) > 1:
            return numpy.abs(self.encList[0]-self.encList[-1])
        else:
            return 1

    @property
    def timeWidth(self):
        if len(self.timeList) > 1:
            return numpy.abs(self.timeList[0]-self.timeList[-1])
        else:
            return 0.02

    @property
    def nDetect(self):
        return len(self.encList)

    @property
    def isHome(self):
        if self.wheelID in self.idList:
            return True
        else:
            return False

    @property
    def moveDir(self):
        if self.encList[-1] > self.encList[0]:
            return 1
        else:
            return -1

    @property
    def isEmpty(self):
        return len(self.timeList)==0


class FilterWheel(object):
    def __init__(self, wheelID):
        if wheelID == 14:
            self.wheelID = 7
        elif wheelID > 7:
            self.wheelID = 2
        self.wheelID = wheelID
        self.meas = parsefile("fwScan_%i.txt"%wheelID)
        self.getDetections()
        self.getFWIDs()
        # self.getZeroAndScale()
        self.getLag()
        # self.fitDrift()

    def getDetections(self):
        self.hallDetections = []
        currDetect = None
        prevM = None
        for m in self.meas:
            if m[2]:
                if currDetect is None:
                    currDetect = HallDetection(self.wheelID)
                    if prevM is not None:
                        currDetect.priorTime = prevM[0]
                currDetect.append(m[0],m[4],m[5],m[3])

            elif currDetect is not None:
                # check if this detection has passed
                currDetect.postTimes.append(m[0])
                if m[0] - currDetect.timeList[-1] > 0.5:
                    self.hallDetections.append(currDetect)
                    currDetect = None
            prevM = m

    def getFWIDs(self):
        lastFWID = None
        lastFWPos = None
        for h in self.hallDetections:
            if lastFWID is None and h.isHome:
                lastFWID = 1
                lastFWPos = h.position
            if lastFWPos is None:
                continue
            elif numpy.abs(lastFWPos-h.position)<700:
                # same filterwheel
                lastFWPos = h.position
            else:
                # new filter
                if lastFWPos > h.position:
                    lastFWID -= 1
                else:
                    lastFWID += 1
            if lastFWID < 1:
                lastFWID += 6
            elif lastFWID > 6:
                lastFWID -= 6
            h.id = lastFWID
            lastFWPos = h.position

    def getFWSpacing(self):
        midPoints = [h.position for h in self.hallDetections]
        diff = numpy.sort(numpy.abs(numpy.diff(midPoints)))
        return diff[diff>1000]

    def getZeroAndScale(self):
        ### next determine full revolution
        # for normalizing
        homeVals = []
        for hd in self.hallDetections:
            if hd.isHome:
                homeVals.append(hd.position)
        homeDiff = numpy.abs(numpy.diff(homeVals))
        self.stepsPerRev = numpy.median(homeDiff[homeDiff>2000])
        self.homeZero = homeVals[0]
        for hd in self.hallDetections:
            hd.stepsPerRev = self.stepsPerRev
            hd.homeZero = self.homeZero

    def getLag(self):
        dt = numpy.diff(self.meas[:,0])
        ind = numpy.argwhere(dt > 0.06)
        ts = self.meas[ind,0].flatten()
        encs = self.meas[ind,5].flatten()
        self.lagDT = dt[ind].flatten()
        self.lagT = ts
        self.lagEnc = encs
        self.maxLags = sorted(dt)[-10:]

    def fitDrift(self):
        # get all positions between 0 and 5000 that are home
        # and fit a line to them
        homeHalls = []
        homeTimes = []
        for hall in self.hallDetections:
            if hall.isHome and hall.position < 5000:
                homeHalls.append(hall.position)
                homeTimes.append(hall.time)
        self.p = numpy.polyfit(homeTimes, homeHalls,1)
        for h in self.hallDetections:
            h.removeDrift(self.p)




    def plotAngPos(self):
        plt.figure()
        for hd in self.hallDetections:
            if hd.isHome:
                color='red'
            else:
                color='blue'
            plt.plot(hd.timeList,hd.angPositionList,'.', color=color)

    def plotWrappedPos(self):
        plt.figure()
        ii=1
        for hd in self.hallDetections:
            if hd.isHome:
                color='red'
            else:
                color='blue'
            pos = hd.wrapEncList
            plt.plot(hd.timeList,pos,'.', color=color)
            ii+=1

    def plotPos(self):
        # this shows missing points!!!!
        plt.figure()
        for hd in self.hallDetections:
            if hd.isHome:
                color='red'
            else:
                color='blue'
            plt.plot(hd.timeList, hd.encList, '.', color=color)

    def plotPosFit(self):
        # this shows missing points!!!!
        plt.figure()
        for hd in self.hallDetections:
            if hd.isHome:
                color='red'
            else:
                color='blue'
            fitEnc = numpy.array(hd.encList) - (self.p[1] + self.p[0]*numpy.array(hd.timeList))
            plt.plot(hd.timeList, fitEnc, '.', color=color)

    def plotWidths(self):
        plt.figure()
        widths = numpy.array([h.encWidth for h in fw.hallDetections])
        print("min width", sorted(widths)[:5])
        plt.hist(widths, 100)

    def plotWidthsPerID(self):
        fig, ((ax1,ax2,ax3),(ax4,ax5,ax6)) = plt.subplots(2,3, figsize=(10,10))
        axList = [ax1,ax2,ax3,ax4,ax5,ax6]
        idList = [1,2,3,4,5,6]
        for ax,ii in zip(axList,idList):
            hIDWidth = [h.encWidth for h in self.hallDetections if h.id==ii]
            ax.hist(hIDWidth,bins=range(50))
            ax.set_title("Filt %i"%ii)
            ax.set_xlim([0,50])
        plt.suptitle("FW ID: %i"%self.wheelID)


allSpacings = []

# for ii in [2,3,4,5,6,7,8,9,10,11,12]:
for ii in [7,14]:
    print("fw %i"%ii)
    # plt.figure()
    fw = FilterWheel(ii)
    fw.plotPos()
    # fwSpacing = fw.getFWSpacing()
    # allSpacings+=list(fwSpacing)
    # plt.hist(fwSpacing,100)
    # plt.title("fw %i"%ii)
    # plt.figure()
    # fw.plotPosFit()

# plt.figure()
# aa = numpy.array(allSpacings)
# plt.hist(aa,100)
# plt.title("all")
plt.show()

# missing #2 and #5

#     meas = parsefile("fwOutput_%i.txt"%ii)
#     keepInds = numpy.argwhere(meas[:,2]==1)
#     time = meas[keepInds,0].flatten()
#     encs = meas[keepInds,5].flatten()
#     dt = numpy.diff(time)
#     tInds = numpy.argwhere(dt>3).flatten()
#     indStart = 0
#     for tInd in tInds:
#         enc = encs[indStart:tInd]
#         try:
#             hallWidth.append(numpy.abs(enc[0]-enc[-1]))
#         except:
#             pass
#         indStart = tInd+1

# print("step width mean", numpy.median(hallWidth))

# plt.figure()
# plt.hist(hallWidth,100)
# plt.show()

    # plt.figure()
    # ids = meas[:,3]
    # print("list length", len(ids))
    # idInds = numpy.argwhere(ids > 0)
    # ids = ids[idInds]
    # plt.hist(ids)

# meas = parsefile("fwOut.txt")
# tdiff = numpy.diff(meas[:,0])
# print("max tdiff", numpy.max(tdiff))
# plt.hist(tdiff,100)

# meas = parsefile("fwOutput_7.txt")
# plt.figure()
# keepInds = numpy.argwhere(meas[:,2]==1)
# plt.plot(meas[keepInds,0], meas[keepInds,2], '.b')
# plt.plot(meas[keepInds,0], meas[keepInds,3], '.r')
# plt.ylim([0,8])

# plt.figure()
# plt.plot(meas[keepInds,5], meas[keepInds,2], '.b')
# plt.plot(meas[keepInds,5], meas[keepInds,3], '.r')
# plt.ylim([0,8])
# plt.show()

