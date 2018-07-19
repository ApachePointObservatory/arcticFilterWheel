Random info I've collected related to the arctic-controller:

The code base is here: https://github.com/ApachePointObservatory/arcticFilterWheel

The PC104 controller is a Technologic Systems TS-7250-V2: https://wiki.embeddedarm.com/wiki/TS-7250-V2.  It is running Linux.

The motor controller is an Arcus ACE-SDE micro step driver.  We are using the USB interface.  This is accessed through the driver ArcusPerformaxDriver.c.  This code is in the src directory in the arcticFilterWheel package.  The documentation for this is included in the doc directory in the code base.

The PC104 controller reads 4 GPIO bits, linked to the hall effect sensors for determining filter wheel position and home.  These are:

36: position bit for filter wheel.
39 38 37 create a binary representation of filter wheel ID.

Hall effect sensors read low (0) when triggered, and high (1) otherwise.

GPIO bit 76 is configured as output.  The PC104/code toggles this bit to put the diffuser in and out.

GPIO control is included in evgpio.c, provided by the PC104 vendor.



Modifying and testing before tagging:
---------------------------------------------------------------
Test version of the code is kept in /root/code/arcticFilterWheel (make modifications in this code first)

Software dependences and setup are handled using eups.  To set up the test version of the code:

$setup arcticFilterWheel -t test

To modify motor parameters, find motorInitList in arcticFilterWheel/python/arcticFilterWheel/device.py.  You can review what these parameters mean from the arcus documentation pdf included in the doc directory.

If any modifications to the C code are made (you probably don't want to do this) you will have to recompile like:

$cd /root/code/arcticFilterWheel/src
$make

To run the test filter wheel:
$setup arcticFilterWheel -t test
$arcticFilterWheel [start|stop|restart]


Tagging/Installing:
------------------------------------------------------------

After making changes and testing them commit them to git, update the doc/versionHistory.html and the python/arcticFilterWheel/version.py file. (replace 1.2.3 with whatever your new version is)

$git commit -m '1.2.3'
$git push
$git tag 1.2.3
$git push
$git push --tags

next copy the code into the installed location on the PC104, and compile the required C code:

$git clone https://github.com/ApachePointObservatory/arcticFilterWheel.git /root/code/installed/arcticFilterWheel/1.2.3

$cd /root/code/installed/arcticFilterWheel/1.2.3

$git checkout 1.2.3 # ensure the correct tag is in place

$rm -rf .git # remove the git directory to 'freeze the code'

$cd src

$make

lastly declare this in eups and make it the current version

$eups declare -r /root/code/installed/arcticFilterWheel/1.2.3 arcticFilterWheel 1.2.3 --current

To restart as tagged version:

$setup arcticFilterWheel
$eups list (verify that 1.2.3 is current)
$arcticFilterWheel [stop|start|restart]



