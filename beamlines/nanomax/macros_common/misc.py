"""
This file contains convenience macros for nanomax, kept in
a separate file so as not to clutter the main beamline file.
"""

import PyTango
from contrast.environment import macro, register_shortcut
from contrast.detectors import Detector
from contrast.motors import Motor

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
    by setting BITS1.outb high or low on the panda box with
    the give name.
    """
    try:
        panda = [m for m in Detector.getinstances() if m.name == name][0]
    except IndexError:
        raise Exception('No Gadget named %s' % name)
    response = panda.query('BITS1.B=%u' % (int(state)))
    if 'OK' in response:
        act = {False: 'opened', True: 'closed'}[state]
        print('Fastshutter %s' % act)
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
class Optics(object):
    """
    Turn the optics motors on, off, or see their state.

    optics <on / off>
    optics  - prints status
    """
    def __init__(self, arg=None):
        self.arg = arg

    def run(self):
        for m in Motor.getinstances():
            if ('hfm_' in m.name
                or 'vfm_' in m.name
                or ('mono_' in m.name and not 'f' in m.name)):
                if self.arg is None:
                    print('(%s) %s' % ({True:'on', False:'OFF'}[m.proxy.PowerOn], m.name))
                else:
                    print('Turning %s %s' % (self.arg, m.name))
                    m.proxy.PowerOn = (self.arg.lower() == 'on')

