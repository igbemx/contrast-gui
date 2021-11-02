"""
This file contains convenience macros for nanomax, kept in
a separate file so as not to clutter the main beamline file.
"""

import PyTango
from contrast.environment import macro, register_shortcut, runCommand
from contrast.motors import Motor
from contrast.detectors import Detector

# some handy shortcuts
register_shortcut('diode1in', 'umv diode1_x 18000')
register_shortcut('diode1out', 'umv diode1_x -18000')
register_shortcut('diode2in', 'umv diode2_y 15000')
register_shortcut('diode2out', 'umv diode2_y -15000')
register_shortcut('fsin', 'umv fastshutter_y -11600')
register_shortcut('fsout', 'umv fastshutter_y 14000')
register_shortcut('watten', 'wm attenuator*')
register_shortcut('wsample', 'wm base* s?')
register_shortcut('wbl', 'wm ivu_* energy mono_x2per ssa_gap*')
register_shortcut('wtab', 'wm table*')

def fastshutter_action(state, name):
    """
    Open (state=False) or close (state=True) the fast shutter,
    by setting BITS.outb high or low on the panda box with
    the give name.
    """
    try:
        panda = [m for m in Detector.getinstances() if m.name == name][0]
    except IndexError:
        raise Exception('No Gadget named %s'%name)
    response = panda.query('BITS.B=%u' % (int(state)))
    if 'OK' in response:
        act = {False: 'opened', True: 'closed'}[state]
        print('Fastshutter %s'%act)
    else:
        print('Could not actuate the shutter')

@macro
class FsOpen(object):
    """
    Opens the fast shutter.
    """
    def run(self):
        fastshutter_action(False, 'panda0')

@macro
class FsClose(object):
    """
    Closes the fast shutter.
    """
    def run(self):
        fastshutter_action(True, 'panda0')

@macro
class M1shift(object):
    """
    Shift the focal plane of the vertically focusing KB
    mirror (M1) by the specified distance (in microns).
    """
    def __init__(self, dist):
        self.dist = dist
    def run(self):
        cmd = 'mvr m1fpitch %f' % (self.dist / -1381.0)
        print("Moving the M1 fine pitch piezo like this:\n%s" % cmd)
        runCommand(cmd)

@macro
class M2shift(object):
    """
    Shift the focal plane of the horizontally focusing KB
    mirror (M2) by the specified distance (in microns).
    """
    def __init__(self, dist):
        self.dist = dist
    def run(self):
        cmd = 'mvr m2fpitch %f' % (self.dist / -857.0)
        print("Moving the M2 fine pitch piezo like this:\n%s" % cmd)
        runCommand(cmd)

@macro
class Table(object):
    """
    Turn on or off the detector table motors.

    table <on / off>
    """
    def __init__(self, arg):
        self.arg = arg
    def run(self):
        for m in Motor.getinstances():
            if 'table_' in m.name:
                print('Turning %s %s' % (self.arg, m.name))
                m.proxy.PowerOn = (self.arg.lower() == 'on')

