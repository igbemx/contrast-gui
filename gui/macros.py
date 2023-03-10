import time
import json
import logging
import numpy as np
from collections import OrderedDict

from tango import DeviceProxy

from contrast.scans.Scan import SoftwareScan
from contrast.detectors.Detector import Detector, DetectorGroup, TriggerSource
from contrast.recorders import active_recorders, RecorderHeader, RecorderFooter
from contrast.motors import all_are_motors
from contrast.environment import MacroSyntaxError

from PyQt5.QtCore import QCoreApplication, QObject, QThread, QTimer, pyqtSignal, pyqtSlot

logging.basicConfig(format="%(message)s", level=logging.INFO)

class LineScan(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def __init__(self, *args, params={}, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.params = params
    
    @pyqtSlot()
    def cancel(self):
        pass
    
    def _generate_positions(self):
        positions = []
        for i in range(len(self.motors)):
            positions.append(np.linspace(self.params['start'],
                                         self.params['stop'],
                                         self.params['points_n']))
        for i in range(len(positions[0])):
            yield {m.name: pos[i] for (m, pos) in zip(self.motors, positions)}
    
    def run(self):
        """Long-running task."""
        for i in range(5):
            time.sleep(1)
            self.progress.emit(i + 1)
        self.finished.emit()
        logging.info(f"Line scan finished, step {i + 1}/5")


class QSoftwareScan(SoftwareScan, QObject):
    """ Base class for the qt involved scans"""
    
    scan_started = pyqtSignal(RecorderHeader)
    scan_finished = pyqtSignal(RecorderFooter)
    scan_data = pyqtSignal(OrderedDict)

    def __init__(self, *args, env=None, **kwargs):
        # super(QSoftwareScan, self).__init__(*args, **kwargs)
        SoftwareScan.__init__(self, *args, **kwargs)
        QObject.__init__(self)
        self.env = env
        self._macro_cancel = False
    
    @pyqtSlot()
    def macro_cancel(self):
        logging.info(f"Setting _macro_cancel to True.")
        self._macro_cancel = True

    def run(self):
        """
        This is the main acquisition loop where interaction with motors,
        detectors and other ``Gadget`` objects happens.
        """
        self._before_scan()
        print('\nScan #%d starting at %s' % (self.scannr, time.asctime()))
        positions = self._generate_positions()
        # find and prepare the detectors
        det_group = Detector.get_active()
        trg_group = TriggerSource.get_active()
        group = det_group + trg_group
        if group.busy():
            print('These gadgets are busy: %s'
                  % (', '.join([d.name for d in group if d.busy()])))
            return
        group.prepare(self.exposuretime, self.scannr, self.n_positions,
                      trials=10)
        t0 = time.time()
        # send a header to the recorders
        snap = self.env.snapshot.capture()
        for r in active_recorders():
            r.queue.put(RecorderHeader(scannr=self.scannr,
                                       status='started',
                                       path=self.env.paths.directory,
                                       snapshot=snap,
                                       description=self.description))
        # emit the qt start signal
        self.scan_started.emit(RecorderHeader(scannr=self.scannr,
                                       status='started',
                                       path=self.env.paths.directory,
                                       snapshot=snap,
                                       description=self.description))
        
        try:
            for i, pos in enumerate(positions):
                # processing possible events such as e.g. cancel event
                QCoreApplication.processEvents()
                # move motors
                self._before_move()
                for m in self.motors:
                    m.move(pos[m.name])
                while True in [m.busy() for m in self.motors]:
                    time.sleep(.01)
                # arm detectors
                self._before_arm()
                group.arm()
                # start detectors
                self._before_start()
                group.start(trials=10)
                while det_group.busy():
                    self._while_acquiring()
                    time.sleep(.01)
                # read detectors and motors
                dt = time.time() - t0
                dct = OrderedDict()
                for m in self.motors:
                    dct[m.name] = m.position()
            
                for d in det_group:
                    dct[d.name] = d.read()
                dct['dt'] = dt
                
                # send data as a signal for gui
                self.scan_data.emit(dct)
                
                # pass data to recorders
                for r in active_recorders():
                    r.queue.put(dct)
                # print spec-style info
                self.output(i, dct.copy())
                if self._macro_cancel:
                    group.stop()
                    print('\nScan #%d cancelled at %s' % (self.scannr, time.asctime()))

                    # emit the qt finished signal
                    self.scan_finished.emit(RecorderFooter(scannr=self.scannr,
                                                status='interrupted',
                                                path=self.env.paths.directory,
                                                snapshot=snap,
                                                description=self.description))

                    # tell the recorders that the scan was interrupted
                    for r in active_recorders():
                        r.queue.put(RecorderFooter(scannr=self.scannr,
                                                status='interrupted',
                                                path=self.env.paths.directory,
                                                snapshot=snap,
                                                description=self.description))
                    break
            print('\nScan #%d ending at %s' % (self.scannr, time.asctime()))

            # tell the recorders that the scan is over
            for r in active_recorders():
                r.queue.put(RecorderFooter(scannr=self.scannr,
                                           status='finished',
                                           path=self.env.paths.directory,
                                           snapshot=snap,
                                           description=self.description))
                
            self.scan_finished.emit(RecorderFooter(scannr=self.scannr,
                                           status='finished',
                                           path=self.env.paths.directory,
                                           snapshot=snap,
                                           description=self.description))

        except:
            group.stop()
            print('\nScan #%d cancelled at %s' % (self.scannr, time.asctime()))

            # emit the qt finished signal
            self.scan_finished.emit(RecorderFooter(scannr=self.scannr,
                                           status='interrupted',
                                           path=self.env.paths.directory,
                                           snapshot=snap,
                                           description=self.description))
            self._after_scan()
            
            # tell the recorders that the scan was interrupted
            for r in active_recorders():
                r.queue.put(RecorderFooter(scannr=self.scannr,
                                           status='interrupted',
                                           path=self.env.paths.directory,
                                           snapshot=snap,
                                           description=self.description))
            raise

        # do any user-defined cleanup actions
        self._after_scan()
        logging.info(f"At the end of the macro run.")

        
class QAScan(QSoftwareScan):
    """
    Software scan one or more motors in parallel. ::

        ascan <motor1> <start> <stop> ... <intervals> <exp_time>
    """
    # finished = pyqtSignal()
    progress = pyqtSignal(int)
    
    def __init__(self, *args, env=None, **kwargs):
        self.motors = []
        self.limits = []
        logging.info(f"args of QAScan are: {args}")
        logging.info(f"kwargs of QAScan are: {kwargs}")
        try:
            exposuretime = float(args[-1])
            self.intervals = int(args[-2])
            super(QAScan, self).__init__(exposuretime, env=env)
            for i in range(int((len(args) - 2) / 3)):
                self.motors.append(args[3 * i])
                self.limits.append(
                    [float(m) for m in args[3 * i + 1:3 * i + 3]])
            self.n_positions = self.intervals + 1
            assert all_are_motors(self.motors)
            assert (len(args) - 2) % 3 == 0
            
            self.description = {
                'scan' : 'ascan',
                'motors': [m.name for m in self.motors],
                'limits': [i for i in self.limits],
                'exposure': exposuretime
            }
            self.description = json.dumps(self.description)
        except:
            raise MacroSyntaxError

    def _generate_positions(self):
        positions = []
        for i in range(len(self.motors)):
            positions.append(np.linspace(self.limits[i][0],
                                         self.limits[i][1],
                                         self.intervals + 1))
        for i in range(len(positions[0])):
            yield {m.name: pos[i] for (m, pos) in zip(self.motors, positions)}
            
class QMesh(QSoftwareScan):
    """
    Software scan on a regular grid of N motors. ::

        mesh <motor1> <start> <stop> <intervals> ... <exp_time>

    optional keyword arguments:
        jitter: float ... Randomizes perfect grid positions.
    """
    progress = pyqtSignal(int)
    
    def __init__(self, *args, env=None, **kwargs):
        self.motors = []
        self.limits = []
        self.intervals = []
        self.kwargs = kwargs
        logging.info(f"args of QMesh are: {args}")
        logging.info(f"kwargs of QMesh are: {kwargs}")

        try:
            exposuretime = float(args[-1])
            super(QMesh, self).__init__(exposuretime, env=env)
            for i in range(int((len(args) - 1) / 4)):
                self.motors.append(args[4 * i])
                self.limits.append(
                    [float(m) for m in args[4 * i + 1:4 * i + 3]])
                self.intervals.append(int(args[4 * i + 3]))
            self.n_positions = np.prod(np.array(self.intervals) + 1)
            assert all_are_motors(self.motors)
            assert (len(args) - 1) % 4 == 0
            
            self.description = {
                'scan' : 'mesh',
                'motors': [m.name for m in self.motors],
                'limits': [l for l in self.limits],
                'points': [i+1 for i in self.intervals],
                'exposure': exposuretime
            }
            self.description = json.dumps(self.description)
        except:
            raise MacroSyntaxError

    def _generate_positions(self):
        positions = []
        for i in range(len(self.motors)):
            positions.append(np.linspace(self.limits[i][0],
                                         self.limits[i][1],
                                         self.intervals[i] + 1))
        grids = np.meshgrid(*reversed(positions))
        grids = [l_ for l_ in reversed(grids)]  # fastest last

        if 'jitter' in self.kwargs.keys():
            print('[!] jittered grid postions by factor:',
                  self.kwargs['jitter'])
            if self.kwargs['jitter'] != 0:

                step_sizes = []
                for i, motor in enumerate(self.motors):
                    d = np.abs(self.limits[i][0] - self.limits[i][1])
                    n = self.intervals[i]
                    step_sizes.append(1. * d / n)

                rel_jitter = np.random.uniform(low=-.5 * self.kwargs['jitter'],
                                               high=.5 * self.kwargs['jitter'],
                                               size=np.shape(grids))
                for i, step_size in enumerate(step_sizes):
                    grids[i] += rel_jitter[i] * step_size

        for i in range(len(grids[0].flat)):
            yield {m.name: pos.flat[i] for (m, pos) in zip(self.motors, grids)}
            

class QMesh_fly(QSoftwareScan):
    """
    Sample stxm for softimax.
        
    QMesh_fly <fast motor> <start> <stop> <intervals> <slow motor> <start> <stop> <intervals> <exp time> <latency> <trigger axis X or Y>

    """

    SLEEP = .01
    PIEZO_SYNC_DELAY = 0.5 # delay in seconds to syncronize the piezo stage with the interferometer
    MARGIN = 1.
    PANDA = 'B318A-EA01/CTL/PandaPosTrig'
    
    progress = pyqtSignal(int)

    def __init__(self, *args, env=None, **kwargs):
        """
        Parse and check arguments.
        """
        (self.slow_motor, self.slow_begin, self.slow_end, self.slow_ints,
         self.fast_motor, self.fast_begin, self.fast_end, self.fast_ints,
         self.exptime, self.latency, self.trigger_axis) = args
        
        #super(QMesh_fly, self).__init__(self.exptime, env=env)
        QSoftwareScan.__init__(self, self.exptime, env=None, **kwargs)
        self.env = env
        
        if len(args) < 10:
            raise ValueError('Not enough parameters.')

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

        self.description = {
                'scan' : 'mesh_fly',
                'motors': [self.slow_motor.name, self.fast_motor.name],
                'limits': [[float(self.slow_begin), float(self.slow_end)], [float(self.fast_begin), float(self.fast_end)]],
                'points': [int(self.slow_ints)+1, int(self.fast_ints)+1],
                'exposure': self.exptime
            }
        self.description = json.dumps(self.description)
        
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

        snap = self.env.snapshot.capture()
        self.scan_started.emit(RecorderHeader(scannr=self.scannr,
                                       status='started',
                                       path=self.env.paths.directory,
                                       snapshot=snap,
                                       description=self.description))
        
        try:
            slow_positions = np.linspace(
                                 self.slow_begin, self.slow_end, self.slow_ints+1
                             )
            for y_i, y_val in enumerate(slow_positions):
                QCoreApplication.processEvents()
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
                dct['panda'] = {'x': self.panda.XPosOut,
                                'y': self.panda.YPosOut,
                                'PMT': self.panda.PMTOut,
                                'diode': self.panda.PDiodeOut}
                print(f'XPosOut length is: {len(self.panda.XPosOut)}')

                # pass data to recorders
                for r in active_recorders():
                    r.queue.put(dct)
                    
                # emit data for the gui part
                self.scan_data.emit(dct)

                # print spec-style info
                self.output(y_i, dct.copy())

                if self._macro_cancel:
                    self.group.stop()
                    print('\nScan #%d cancelled at %s' % (self.scannr, time.asctime()))

                    # emit the qt finished signal
                    self.scan_finished.emit(RecorderFooter(scannr=self.scannr,
                                                status='interrupted',
                                                path=self.env.paths.directory,
                                                snapshot=snap,
                                                description=self.description))

                    # tell the recorders that the scan was interrupted
                    for r in active_recorders():
                        r.queue.put(RecorderFooter(scannr=self.scannr,
                                                status='interrupted',
                                                path=self.env.paths.directory,
                                                snapshot=snap,
                                                description=self.description))

                    self.scan_finished.emit(RecorderFooter(scannr=self.scannr,
                                           status='interrupted',
                                           path=self.env.paths.directory,
                                           snapshot=snap,
                                           description=self.description))
                        
                    break

            print('\nScan #%d ending at %s' % (self.scannr, time.asctime()))
            self.scan_finished.emit(RecorderFooter(scannr=self.scannr,
                                           status='finished',
                                           path=self.env.paths.directory,
                                           snapshot=snap,
                                           description=self.description))

        except KeyboardInterrupt:
            for r in active_recorders():
                r.queue.put(RecorderFooter(scannr=self.scannr,
                                           status='cancelled',
                                           path=self.env.paths.directory))
            return

        for r in active_recorders():
            r.queue.put(RecorderFooter(scannr=self.scannr,
                                       status='finished',
                                       path=self.env.paths.directory))

    def _setup(self):
        # find and prepare the detectors
        self.group = Detector.get_active()
        self.group.prepare(self.exposuretime, self.scannr, self.slow_ints + 1, trials=10)
        self.t0 = time.time()
        # send a header to the recorders
        snap = self.env.snapshot.capture()
        for r in active_recorders():
            r.queue.put(RecorderHeader(scannr=self.scannr, 
                                       status='started',
                                       path=self.env.paths.directory,
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
        
        self.panda.ResetPointCntr()
        self.panda.DetTimePulseStep = exptime + latency
        self.panda.DetTimePulseWidth = exptime
        self.panda.DetTimePulseN = N_points
        self.panda.TimePulsesEnable = True

        # setting up trigger axis motor velocity
        vel = (abs(start - end)) / (N_points * (exptime + latency) * 1e-3)
        self._set_vel(vel)

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


