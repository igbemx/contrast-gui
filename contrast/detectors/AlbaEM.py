"""
This module contains an interface to the Alba electrometer, as well as
a contrast Detector class representing it.

Requires the so-called fastbuffer mode of the electrometer, with data
streaming, developed (but not yet formally delivered) by the MAX IV
hardware group in 2021 as described here,

https://gitlab.maxiv.lu.se/kits-maxiv/lib-cells-albaem2/-/tree/streaming-no-zeromq/nuc_SW/alin/extra.

For the old school data-polling version, see LegacyAlbaEM.py.
"""

if __name__ == '__main__':
    from contrast.detectors import Detector, LiveDetector, TriggeredDetector, BurstDetector
else:
    from .Detector import Detector, LiveDetector, TriggeredDetector, BurstDetector
import telnetlib
import numpy as np
import time
import socket, select
from threading import Thread

SOFT_TRIG_HACK = 0.10
BUF_SIZE = 1024
NUM_CHAN = 4
TIMEOUT = None
VALID_RANGE_STRINGS = ['1mA', '100nA', '10nA', '1nA', '100uA', '10uA', '1uA', '100pA']
VALID_FILTER_STRINGS = ['3200Hz', '100Hz', '10Hz', '1Hz', '0.5Hz']
BUSY_STATES = ('STATE_RUNNING', 'STATE_ACQUIRING')
IDLE_STATES = ('STATE_ON', )


class Stream(Thread):
    """
    Server which receives the electrometer's "fast buffer" stream.
    """
    def __init__(self, port, debug=False):
        super().__init__()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((socket.gethostname(), port))
        self.server.listen(1)
        self.data = []
        self.do_debug = debug

    def debug(self, *args):
        if self.do_debug:
            print(*args)

    def run(self):
        while True:
            client, address = self.server.accept()
            self.debug('client connected')
            data = b''
            while True:
                self.debug('server looping')
                new = client.recv(BUF_SIZE)
                if new == b'':
                    self.debug('client disconnected')
                    break
                self.debug('received:', new)
                data += new
                parts = data.split(b'\n')
                data = parts[-1] # empty if terminated with \n, otherwise dangling data
                for part in parts[:-1]:
                    self.data.append(list(map(float, part.split(b',')))[1:])
                    self.debug('now have %u data points' % len(self.data))


class Electrometer(object):
    """
    Interface to a 4-channel Alba electrometer.
    """

    def __init__(self, host='b-nanomax-em2-2', port=5025,
                 trig_source='DIO_1',
                 stream_host=None, stream_port=50001):
        self.em = telnetlib.Telnet(host, port)
        self.query('ACQU:MODE CURRENT')
        # the DIO channel used for triggering:
        self._trig_source = trig_source
        # require SW version 2.0.04, below that soft triggers are broken
        # and below 2.0.0 data indexing is wrong.
        assert self.version >= (2, 0, 4), "Requires on-board SW version 2.0.04 or higher."
        # Figure out host and port for the stream receiving server:
        if not stream_host:
            stream_host = socket.gethostbyname(socket.gethostname())
        ok = False
        while not ok:
            try:
                self.stream = Stream(port=stream_port, debug=False)
                self.stream.start()
                ok = True
            except OSError:
                stream_port += 1
                print('trying again with port %u' % stream_port)
        self.stream_host = stream_host
        self.stream_port = stream_port
        # Handle an annoying bug in soft triggering:
        self.last_soft_trig = time.time()

    def _flush(self):
        return self.em.read_eager().strip().decode('utf-8')

    def query(self, cmd):
        """
        Issue a command and read the answer.
        """
        # ctrl-c during a previous query can have left stuff to be read:
        self._flush()
        cmd = (cmd + '\n').encode('utf-8')
        self.em.write(cmd)
        reply = self.em.read_until(b'\r\n', timeout=TIMEOUT).strip().decode('utf-8')
        if reply:
            return reply
        else:
            return None

    def get_instant_current(self, ch):
        return float(self.query('CHAN%02u:INSC?'%ch))

    def get_current_range(self, ch):
        return self.query('CHAN%02u:CABO:RANG?'%ch)

    def set_current_range(self, ch, val):
        if not val in VALID_RANGE_STRINGS:
            return
        self.query('CHAN%02u:CABO:RANG %s'%(ch, val))

    def get_autorange(self, ch):
        lookup = {'On': True, 'Off': False}
        return lookup[self.query('CHAN%02u:CABO:ARNG?'%ch)]

    def set_autorange(self, ch, val):
        val = {True: 'On', False:'Off'}[bool(val)]
        self.query('CHAN%02u:CABO:ARNG %s'%(ch, val))

    def get_filter(self, ch):
        return self.query('CHAN%02u:CABO:FILT?'%ch)

    def set_filter(self, ch, val):
        if not val in VALID_FILTER_STRINGS:
            return
        self.query('CHAN%02u:CABO:FILT %s'%(ch, val))

    def state(self):
        st = self.query('ACQU:STAT?')
        return st

    def stop(self):
        self.query('ACQU:STOP True')

    def status(self):
        return self.query('ACQU:STUS?')

    def get_acqtime(self):
        return float(self.query('ACQU:TIME?')) * 1e-3

    def set_acqtime(self, val):
        val = val * 1000 # ms
        val = max(val, 0.320)
        self.query('ACQU:TIME %f'%val)

    def burst(self, period=1., n=1, latency=320e-6):
        """
        Take a series of measurements with internal timing. No
        triggering is possible, the series starts immediately.
        """
        latency = max(latency, .320e-3)
        acqtime = period - latency
        self.stream.data.clear()
        self.query('ACQU:TIME %f'%(acqtime*1000))
        self.query('ACQU:LOWT %f'%(latency*1000))
        self.query('TRIG:MODE AUTOTRIGGER')
        self.query('ACQU:NTRI %u'%n)
        self.query('ACQU:MODE fast_buffer')
        self.query('ACQU:STAR %s %u'%(self.stream_host, self.stream_port))

    def arm(self, acqtime=1., n=1, hw=False):
        """
        Prepare for hw- or sw-triggered acquisition, n x acqtime.
        """
        acqtime = acqtime * 1000 # ms
        acqtime = max(acqtime, 0.320)
        self.stream.data.clear()
        self.query('ACQU:TIME %f'%acqtime)
        self.query('TRIG:MODE %s' % ('HARDWARE' if hw else 'SOFTWARE'))
        self.query('TRIG:INPU %s'%self._trig_source)
        self.query('TRIG:DELA 0.0') # no delay
        self.query('ACQU:NTRI %u'%n)
        self.query('ACQU:MODE fast_buffer')
        self.query('ACQU:STAR %s %u'%(self.stream_host, self.stream_port))

    def soft_trigger(self):
        # This limits soft triggers to 10 Hz, to work around a bug in
        # the FAST_BUFFER mode, wherein the EM is blind to soft triggers
        # for a while. Hopefully the bug will be removed.
        time_passed = time.time() - self.last_soft_trig
        if time_passed < SOFT_TRIG_HACK:
            time.sleep(SOFT_TRIG_HACK - time_passed)
        self.last_soft_trig = time.time()
        ret = self.query('TRIG:SWSE True')
        assert ret == 'ACK'

    @property
    def ndata(self):
        """
        ACQU:NDAT? increments before the integration time is over, so
        cannot be used. Look at the stream instead.
        """
        return len(self.stream.data)

    @property
    def version(self):
        res = self.query('*IDN?')
        version = res.split(',')[-1].strip()
        return tuple(map(int, version.split('.')))

    def test_soft_triggers(self, N=1000):
        """
        The soft triggering wasn't reliable up until SW version 2.0.0,
        and seems to have started working from 2.0.04. This procedure
        typically halted after a few 100 points.
        """
        self.arm(n=N, acqtime=.001)
        for i in range(N):
            print('starting loop #%u'%(i+1))
            n = self.ndata
            while n < i:
                print('   only have %u, trying again'%n)
                time.sleep(.01)
                n = self.ndata
            print('   have %u, issuing trigger #%u'%(n, i+1))
            self.soft_trigger()


class AlbaEM(Detector, LiveDetector, TriggeredDetector, BurstDetector):
    """
    Contrast interface to the alba EM.

    The specifics of the EM enables these four cases, each of which
    causes a different triggering and readout behaviour below:

    1) HW triggered expecting one trigger per SW step -> arm at the top
    2) HW triggered expecting hw_trig_n triggers per SW step -> arm on every sw step
    3) Burst mode, burst_n > 1, uses a special EM command
    4) Software triggered mode, -> arm at the top

    Note that the electrometer itself
    (as of SW version 2.0.04) does not allow for triggered burst
    acquisition, as reflected in the code.
    """
    def __init__(self, name=None, debug=False, **kwargs):
        self.kwargs = kwargs
        self.do_debug = debug
        Detector.__init__(self, name=name)
        LiveDetector.__init__(self)
        TriggeredDetector.__init__(self)
        BurstDetector.__init__(self)

    def initialize(self):
        self.em = Electrometer(stream_port=50013, **self.kwargs)
        self.burst_latency = 320e-6
        self.n_started = 0
        self.channels = [ch+1 for ch in range(NUM_CHAN)]

    def prepare(self, acqtime, dataid, n_starts):
        BurstDetector.prepare(self, acqtime, dataid, n_starts)
        self.n_started = 0
        self.n_starts = n_starts
        if self.busy():
            raise Exception('%s is busy!' % self.name)
        msg = "The Alba EM cannot handle hardware-triggered burst acquisitions!"
        assert not (self.hw_trig and (self.burst_n > 1)), msg
        if self.global_arm:
            self.armed_so_far = 0
            self.em.arm(self.acqtime, self.n_starts, self.hw_trig)

    @property
    def global_arm(self):
        # This checks whether to arm the EM for several SW starts.
        # This corresponds to cases 1 and 4 (above).
        return (self.hw_trig and self.hw_trig_n==1) or (not self.hw_trig and self.burst_n==1)

    def start_live(self, acqtime=1.0):
        # The Alba EM:s are always in live mode, exposing the "instant current" values.
        pass

    def stop_live(self):
        # Nothing to do...
        pass

    def arm(self):
        if (self.hw_trig and self.hw_trig_n>1):
            self.em.arm(self.acqtime, self.hw_trig_n, True)

    def start(self):
        self.n_started += 1
        if self.hw_trig:
            return
        elif self.burst_n > 1:
            period = self.acqtime + self.burst_latency
            self.em.burst(period, self.burst_n, self.burst_latency)
        else:
            self.em.soft_trigger()

    def stop(self):
        self.em.stop()
        while self.busy():
            time.sleep(.01)

    def busy(self):
        st = self.em.state()
        if st in IDLE_STATES:
            return False
        if self.global_arm:
            # case 1 or 4 (above): expect one point per start on the stream
            return (self.em.ndata < self.n_started)
        else:
            # case 2 or 3 (see above): require an idle status
            return st in BUSY_STATES
        assert(False), "Should never get here!"

    def read(self):
        chs = self.channels
        keys = ['t',] + self.channels
        if self.global_arm:
            # case 1 or 4, which means read a single point each time
            data = np.array(self.em.stream.data[-1])
            data[1:] = data[1:] * 1e-6 # Amps
            return {k:v for k,v in zip(keys, data)}
        else:
            # case 2 or 3, return the last burst_n or hw_trig_n points
            chunk = self.hw_trig_n if self.hw_trig else self.burst_n
            assert (len(self.em.stream.data) == chunk), 'wrong number of points received'
            ret = {}
            data = np.array(self.em.stream.data)
            data[:, 1:] = data[:, 1:] * 1e-6 # Amps
            for i in range(len(keys)):
                ret[keys[i]] = data[:, i].reshape((1,-1))
            return ret


if __name__ == '__main__':
    # Example usage of the bare Electrometer class. Currents in uA.
    em = Electrometer(host='b-nanomax-em2-0')
    em.burst(period=.001, n=60000) # 1 minute, 1 kHz
    while em.ndata < 60000:
        print('Now have %u points' % len(em.stream.data))
        time.sleep(.1)
