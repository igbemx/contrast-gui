from contrast.environment import env, macro, MacroSyntaxError
from contrast.scans import SoftwareScan
from contrast.detectors import Detector
from contrast.motors import Motor, all_are_motors
from contrast.recorders import active_recorders, RecorderHeader, RecorderFooter
import numpy as np
import time
from collections import OrderedDict
from tango import DeviceProxy


@macro
class Stxm(SoftwareScan):
    """
    Sample stxm for softimax.
        
    stxm <fast motor> <start> <stop> <intervals> <slow motor> <start> <stop> <intervals> <exp time> <latency> <trigger axis X or Y>

    """

    SLEEP = .01
    PIEZO_SYNC_DELAY = 0.5 # delay in seconds to syncronize the piezo stage with the interferometer
    FAST = 1000
    MARGIN = 1.
    PANDA = 'B318A-EA01/CTL/PandaPosTrig'

    def __init__(self, *args, **kwargs):
        """
        Parse and check arguments.
        """
        
        if len(args) < 10:
            raise ValueError('Not enough parameters. See "%stxm?" for usage.')

        (self.fast_motor, self.fast_begin, self.fast_end, self.fast_ints,
         self.slow_motor, self.slow_begin, self.slow_end, self.slow_ints,
         self.exptime, self.latency, self.trigger_axis) = args

        # make sure the fast motor has a velocity or proxy.velocity attribute
        ok = False
        if hasattr(self.fast_motor, 'velocity'):
            ok = True
            self.proxy_attr = False
        elif hasattr(self.fast_motor, 'proxy'):
            if hasattr(self.fast_motor.proxy, 'Velocity'):
                ok = True
                self.proxy_attr = True
        if not ok:
            raise ValueError('Fast motor must have .velocity or .proxy.velocity attribute')

        super().__init__(self.exptime)

                # The panda is a special device here, not just a detector.
        if self.PANDA:
            # real pandabox
            self.panda = DeviceProxy(self.PANDA)
            self.panda.DetPosCapt = False
            self.panda.DetPosCapt = True
            self.panda.ResetTrigCntr()
            self.panda.ResetPointCntr()
        else:
            # dummy panda object
            self.panda = DummyPanda()
        
        print('Trigger axis (should normally coincide with the fast scan axis) is: ', self.trigger_axis)

        self._prepare_line_scan(self.fast_begin, self.fast_end, self.fast_ints+1, self.exptime, self.latency, self.trigger_axis)
        

    def run(self):
        """
        Run the actual scan. These little hooks can be included for full
        compatibility with SoftwareScan (opening shutters, printing
        progress, etc):

            self._before_scan()
            self._before_move()
            self._before_arm()
            self._before_start()
            self._while_acquiring()
            self._after_scan()
        """

        # Pre-scan stuff (incl snapshots and detector preparation)
        self._before_scan()
        self._setup()
        print('\nScan #%d starting at %s' % (self.scannr, time.asctime()))

        try:
            slow_positions = np.linspace(
                                 self.slow_begin, self.slow_end, self.slow_ints+1
                             )
            for y_i, y_val in enumerate(slow_positions):
                # enable position capturing
                self.panda.DetPosCapt = True
                
                # move to the next line
                self._before_move()
                self.slow_motor.move(y_val)
                while self.slow_motor.busy():
                    time.sleep(.01)

                # arm detectors (like triggered ones)
                self._before_arm()
                self.group.arm()

                # start detectors (like not triggered ones)
                self._before_start()
                self.group.start(trials=10)

                # run the stxm line
                self._do_line(self.fast_begin, self.fast_end)

                while (self.panda.PointNOut is None
                       or (len(self.panda.PointNOut) < self.fast_ints+1)
                       or self.fast_motor.busy()
                       or self.group.busy()):
                    self._while_acquiring()
                    time.sleep(.01)

                # disable position capturing to clean the data buffer
                self.panda.DetPosCapt = False
                # read detectors and panda
                dt = time.time() - self.t0
                dct = OrderedDict()
                for d in self.group:
                    dct[d.name] = d.read()
                dct['dt'] = dt
                # weird that this [:N] indexing is needed, something wrong with panda scheme?
                dct['panda'] = {'x': self.panda.XPosOut[:self.fast_ints].reshape(1, -1),
                                'y': self.panda.YPosOut[:self.fast_ints].reshape(1, -1),
                                'PMT': self.panda.PMTOut[:self.fast_ints].reshape(1, -1),
                                'diode': self.panda.PDiodeOut[:self.fast_ints].reshape(1, -1)}
                print(f'XPosOut length is: {len(self.panda.XPosOut)}')

                # pass data to recorders
                for r in active_recorders():
                    r.queue.put(dct)

                # print spec-style info
                self.output(y_i, dct.copy())

            print('\nScan #%d ending at %s' % (self.scannr, time.asctime()))

        except KeyboardInterrupt:
            for r in active_recorders():
                r.queue.put(RecorderFooter(scannr=self.scannr,
                                           status='cancelled',
                                           path=env.paths.directory))
            return

        for r in active_recorders():
            r.queue.put(RecorderFooter(scannr=self.scannr,
                                       status='finished',
                                       path=env.paths.directory))

    def _setup(self):
        # find and prepare the detectors
        self.group = Detector.get_active()
        self.group.prepare(self.exposuretime, self.scannr, self.slow_ints + 1, trials=10)
        self.t0 = time.time()
        # send a header to the recorders
        snap = env.snapshot.capture()
        for r in active_recorders():
            r.queue.put(RecorderHeader(scannr=self.scannr, 
                                       status='started',
                                       path=env.paths.directory,
                                       snapshot=snap, 
                                       description=self._command))

    def _set_vel(self, vel):
        if self.proxy_attr:
            self.fast_motor.proxy.Velocity = vel
        else:
            self.fast_motor.velocity = vel

    def _prepare_line_scan(self, start, end, N_points, exptime, latency, trigger_axis):
        """Prepares panda for the line acquisition"""

        self.panda.TrigAxis = trigger_axis # triger axis X or Y for horizontal and vertical respectively
        print(f'preparing line scan, moving the fast motor to {start}')
        if trigger_axis == 'X':
            self.fast_motor.move(start)
            time.sleep(self.PIEZO_SYNC_DELAY)
            self.panda.TrigXPos = self.panda.AbsX # position in microns
        elif trigger_axis == 'Y':
            self.fast_motor.move(start)
            time.sleep(self.PIEZO_SYNC_DELAY)
            self.panda.TrigYPos = self.panda.AbsY
        else:
            raise ValueError('The triggered axis name should be either X or Y.')
        self.panda.DetTimePulseStep = 1e3 * (exptime + latency)
        self.panda.DetTimePulseWidth = 1e3 * exptime
        self.panda.DetTimePulseN = N_points
        self.panda.TimePulsesEnable = True

        # setting up trigger axis motor velocity
        vel = (abs(start - end)) / (N_points * (exptime + latency))
        self._set_vel(vel)
        # self._set_vel(self.FAST)

    def _do_line(self, start, end):
        """Arms panda before each new line and moves the fast motor"""
        
        # go to the starting position
        self.fast_motor.move(start - self.MARGIN)
        # arm panda for a single line acquisition
        self.panda.ArmSingle()
        while self.fast_motor.busy():
            time.sleep(self.SLEEP)

        # do a controlled movement
        self.fast_motor.move(end)

    def _while_acquiring(self):
        print('%s: %.1f, point#: %i  \r' % (self.fast_motor.name, self.fast_motor.user_position, self.panda.DetPointCntr), end=' ')


class DummyPanda(object):

    dum = np.arange(10000, dtype=np.uint8)
    PointNOut = dum
    XPosOut = dum
    YPosOut = dum
    PMTOut = dum
    PDiodeOut = dum

    def ArmSingle(self):
        pass

@macro
class InterfSync(object):
    """
    Syncronizes panda interferometer readings with the coarse motors positions

        InterfSync [<coarse_x>] [<coarse_y>]
    """
    
    PANDA = 'B318A-EA01/CTL/PandaPosTrig'

    def __init__(self, coarse_x='coarse_x', coarse_y='coarse_y'):

        if ('motor' in str(coarse_x.__class__) and 
            'motor' in str(coarse_y.__class__)):
            x_coarse_pos = coarse_x.position()
            y_coarse_pos = coarse_y.position()
            print(x_coarse_pos, y_coarse_pos)
        elif coarse_x.__class__ == str and coarse_y.__class__ == str:
            motors = {m.name: m for m in Motor.getinstances()}
            x_coarse_pos = motors[coarse_x].position()
            y_coarse_pos = motors[coarse_y].position()
            print(x_coarse_pos, y_coarse_pos)
        else:
            raise ValueError('Valid motor objects or motor names have to be provided')
        
        panda = DeviceProxy(self.PANDA)
        panda.AbsX = x_coarse_pos
        panda.AbsY = y_coarse_pos

    def run(self):
        pass