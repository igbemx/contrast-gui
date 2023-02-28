"""
Sets up a Soft beamline with real motors and detectors.
"""
# need this main guard here because Process.start() (so our recorders)
# import __main__, and we don't want the subprocess to start new sub-
# processes etc.

import os
import time
import contrast
import tango
from contrast.environment import env, runCommand
from contrast.environment.data import SdmPathFixer
from contrast.recorders import Hdf5Recorder, StreamRecorder
from contrast.motors.TangoMotor import TangoMotor
from contrast.detectors import Detector, PseudoDetector
from contrast.detectors.TangoAttributeDetector import TangoAttributeDetector
from contrast.detectors.DhyanaAndor import DhyanaAndor
from contrast.scans import SoftwareScan, Ct
# import contrast.widgets

os.environ["TANGO_HOST"] = "b-v-softimax-csdb-0:10000"

from contrast.motors import DummyMotor
from contrast.motors.SoftiPiezoShutter import SoftiPiezoShutter
# from contrast.motors.SoftiPolarizationCtrl import SoftiPolarizationCtrl
from contrast.detectors import DummyDetector
from contrast.motors.TangoAttributeMotor import TangoAttributeMotor

env.userLevel = 5
# env.paths = SdmPathFixer('B318A/CTL/SDM-01')
env.paths.directory = '/tmp'

# a zmq recorder
#zmqrec = StreamRecorder(name='zmqrec')
#zmqrec.start() # removed for now

# polarization control
# the available values are : horizontal, circularpositive, circularnegative
# pol_ctrl = tango.DeviceProxy('B318A/CTL/ID-ENERGY-CTRL')
# pol_ctrl = TangoAttributeMotor(name='pol_ctrl', device='B318A/CTL/ID-ENERGY-CTRL', attribute='polarizationmode')
# pol_ctrl = SoftiPolarizationCtrl(name='pol_ctrl', device='B318A/CTL/ID-ENERGY-CTRL')

# motors
# finex = TangoMotor(device='PiezoPiE712/CTL/X', name='finex', user_format='%.3f', dial_format='%.3f', dial_limits=(0, 100), offset=50, scaling=-1)
# finey = TangoMotor(device='PiezoPiE712/CTL/Y', name='finey', user_format='%.3f', dial_format='%.3f', dial_limits=(0, 100), offset=50, scaling=-1)

# finex = TangoMotor(device='B318A-EA01/CTL/PI_X', name='finex', user_format='%.3f', dial_format='%.3f', dial_limits=(-50, 50), offset=0, scaling=1)
# finey = TangoMotor(device='B318A-EA01/CTL/PI_Y', name='finey', user_format='%.3f', dial_format='%.3f', dial_limits=(-50, 50), offset=0, scaling=1)

# coarse_x = TangoMotor(device='B318A-EA01/dia/SAMS-01-X', name='coarse_x', user_format='%.3f', dial_format='%.3f', dial_limits=(-5, 5), offset=0.0, scaling=1000)
# coarse_y = TangoMotor(device='B318A-EA01/dia/SAMS-01-Y', name='coarse_y', user_format='%.3f', dial_format='%.3f', dial_limits=(-5, 5), offset=0.0, scaling=1000)

# osax = TangoMotor(device='motor/osa_ctrl/1', name='osax', user_format='%.3f', dial_format='%.3f', offset=0.83)
# osay = TangoMotor(device='motor/osa_ctrl/2', name='osay', user_format='%.3f', dial_format='%.3f', offset=0.32)
# beamline_energy = TangoMotor(device='B318A/CTL/BEAMLINE-ENERGY', name='beamline_energy', user_format='%.3f', dial_format='%.3f', dial_limits=(275, 1600))

#    shutter0 = SoftiPiezoShutter(device='B318A-EA01/CTL/PiezoShutter', name='shutter0')
# shutter0 = SoftiPiezoShutter(device='B318A-EA01/CTL/GalilShutter', name='shutter0')

# zp = tango.DeviceProxy('B318A-EA01/CTL/SoftiZPEnergy')
# zp_mot = TangoMotor(device='B318A-EA01/CTL/SoftiZPEnergy', name='zp', user_format='%.3f', dial_format='%.3f', dial_limits=(-1300, -15000))
# zp_energy = TangoAttributeMotor(name='zp_E_mot', device='B318A-EA01/CTL/SoftiZPEnergy', attribute='Energy')

finex = TangoMotor(device='B318A/CTL/DUMMY-01', name='finex', user_format='%.3f', dial_format='%.3f', dial_limits=(0, 100))
finey = TangoMotor(device='B318A/CTL/DUMMY-02', name='finey', user_format='%.3f', dial_format='%.3f', dial_limits=(0, 100))

samx = DummyMotor(name='samx', dial_limits = (0, 10))
samy = DummyMotor(name='samy', dial_limits = (-5, 5))

# cameras
# andor = DhyanaAndor(device='B318A-EA01/dia/andor-zyla-01', name='andor', hdf_name='zyla')
# dhyana = DhyanaAndor(name='dhyana', hdf_name='dhyana', device='b318a-ea01/dia/dhyana')

# other detectors    
abs_x = TangoAttributeDetector('abs_x', 'B318A-EA01/CTL/PandaPosTrig', 'AbsX')
abs_y = TangoAttributeDetector('abs_y', 'B318A-EA01/CTL/PandaPosTrig', 'AbsY')
roi = TangoAttributeDetector('roi', 'B318A-EA01/dia/andor-requests', 'data_mean')

    # default detector selection
for d in Detector.getinstances():
    d.active = False
for d in [roi, abs_x, abs_y]:
    d.active = True

# def pre_scan_stuff(slf):
#         shutter0.Open()
#         time.sleep(0.2)

# def post_scan_stuff(slf):
#         shutter0.Close()

# SoftwareScan._before_scan = pre_scan_stuff
# SoftwareScan._after_scan = post_scan_stuff
# Ct._before_ct = pre_scan_stuff
# Ct._after_ct = post_scan_stuff

contrast.wisdom()

# the Hdf5Recorder later gets its path from the env object
h5rec = Hdf5Recorder(name='h5rec')
h5rec.start()

try:
    l = os.listdir(env.paths.directory)
    last = max([int(l_[:-3]) for l_ in l if (len(l_)==9 and l_.endswith('.h5'))])
    env.nextScanID = last + 1
    print('\nNote: inferring that the next scan number should be %u' % (last+1))
except Exception as e:
    print(e)

# print('\n\nThe current polarization mode is: ', pol_ctrl.get_polarization())
print('The beamline energy is: ', tango.DeviceProxy('B318A/CTL/BEAMLINE-ENERGY').Position)

