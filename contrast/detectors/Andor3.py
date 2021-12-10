if __name__ == '__main__':
    from contrast.detectors.Detector import Detector, SoftwareLiveDetector, TriggeredDetector, BurstDetector
    from contrast.environment import env
    from contrast.recorders.Hdf5Recorder import Link
else:
    from .Detector import Detector, SoftwareLiveDetector, TriggeredDetector, BurstDetector
    from ..environment import env
    from ..recorders.Hdf5Recorder import Link
import os
import PyTango
import datetime

class Andor3(Detector, SoftwareLiveDetector, BurstDetector):
    """
    Provides an interface to the Andor3 Zyla streaming manager,

    https://github.com/maxiv-science/andor-streamer

    Note: hw trigger not yet implemented.
    """

    def __init__(self, device='zyla/test/1', name=None):
        SoftwareLiveDetector.__init__(self)
        BurstDetector.__init__(self)
        Detector.__init__(self, name=name) # last so that initialize() can overwrite parent defaults
        self.proxy = PyTango.DeviceProxy(device)

    def initialize(self):
        pass

    def prepare(self, acqtime, dataid, n_starts):
        if self.busy():
            raise Exception('%s is busy!' % self.name)

        # saving and paths
        if dataid is None:
            # no saving
            self.saving_file = ''
            self.proxy.filename = ''
        else:
            # saving
            path = env.paths.directory
            fn = 'scan_%06d_%s.hdf5' % (dataid, self.name)
            self.saving_file = os.path.join(path, fn)
            if os.path.exists(self.saving_file):
                print('%s: this hdf5 file exists, I am raising an error now'%self.name)
                raise Exception('%s hdf5 file already exists' % self.name)
            self.proxy.filename = self.saving_file

        # arming and numbers of frames
        self.proxy.exposure_time = acqtime
        if self.burst_n == 1:
            self.proxy.trigger_mode = 'Software'
            self.proxy.frame_count = n_starts
            self.proxy.start()
        else:
            self.proxy.trigger_mode = 'Internal'
            self.proxy.frame_count = self.burst_n
        self.frames_expected = 0

    def arm(self):
        pass

    def start(self):
        if self.burst_n == 1:
            self.proxy.software_trigger()
            self.frames_expected += 1
        else:
            self.proxy.start()
            self.frames_expected = self.burst_n

    def stop(self):
        self.proxy.stop()

    def busy(self):
        st = self.proxy.State()
        if st in (PyTango.DevState.STANDBY, PyTango.DevState.ON):
            return False
        elif st == PyTango.DevState.RUNNING:
            if self.proxy.acquired_frames == self.frames_expected:
                return False
        return True

    def read(self):
        if self.saving_file == '':
            return None
        else:
            return Link(self.saving_file , 'entry/instrument/andor', universal=True)

class AndorSofti(Detector, SoftwareLiveDetector, BurstDetector):
    """
    Provides an interface to the Andor Zyla Tango DS at SoftiMAX
    """

    def __init__(self, device='B318A-EA01/dia/andor-zyla-01', name=None, frames_n = None):
        SoftwareLiveDetector.__init__(self)
        BurstDetector.__init__(self)
        Detector.__init__(self, name=name) # last so that initialize() can overwrite parent defaults
        self.proxy = PyTango.DeviceProxy(device)
        self.frames_n = frames_n

    def initialize(self):
        pass

    def prepare(self, acqtime, dataid, n_starts):
        if self.busy():
            raise Exception('%s is busy!' % self.name)

        self.proxy.Stop()
        self.proxy.TriggerMode = 'SOFTWARE'

        if self.frames_n:
            self.burst_n = self.frames_n
        time_stamp = datetime.datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
        # saving and paths
        if dataid is None:
            # no saving
            self.proxy.nTriggers = n_starts
            self.saving_file = f'andor_ct_{time_stamp}.h5'
            #dest = '/data/staff/softimax/andor/ct'
            dest = env.paths.directory
            andor_dest_name = os.path.join(dest, self.saving_file)
            try:
                self.proxy.DestinationFilename = andor_dest_name
            except Exception as err:
                raise Exception('Faild to prepare Andor withe the error: %s' % err)
        else:
            # saving
            self.dataid = dataid
            if self.burst_n == 1:
                # saving
                path = env.paths.directory
                fn = 'scan_%06d_%s.h5' % (dataid, self.name)
                self.saving_file = os.path.join(path, fn)
                if os.path.exists(self.saving_file):
                    print('%s: this h5 file exists, I am raising an error now'%self.name)
                    raise Exception('%s h5 file already exists' % self.name)
                self.proxy.DestinationFilename = self.saving_file
            else:
                try:
                    self.cam_path = env.paths.directory + '/scan_%06d_%s' % (dataid, self.name)
                    os.mkdir(self.cam_path, 0o777)
                    print(f'Detector frames are written to: {self.cam_path}')
                    fn = 'scan_%06d_%s.h5' % (dataid, self.name)
                    self.saving_file = os.path.join(self.cam_path, fn)
                    if os.path.exists(self.saving_file):
                        print('%s: this h5 file exists, I am raising an error now'%self.name)
                        raise Exception('%s h5 file already exists' % self.name)
                    self.proxy.DestinationFilename = self.saving_file
                    self.file_apndx = 0
                except Exception as e:
                    print(e)

        # arming and numbers of frames
        self.proxy.ExposureTime = acqtime
        if self.burst_n == 1:
            self.proxy.TriggerMode = 'SOFTWARE'
            self.proxy.nTriggers = n_starts
            self.proxy.Arm()
        else:
            print(f'In internal, burst_n : {self.burst_n}')
            self.proxy.TriggerMode = 'INTERNAL'
            self.point_n_apndx = 0 
            self.proxy.nTriggers = self.burst_n
        self.frames_expected = 0
        #

    def arm(self):
        while self.busy():
            continue
        if self.burst_n != 1:
            pass
            

    def start(self):
        while self.busy():
            continue
        try:
            if self.burst_n == 1:
                self.proxy.SoftwareTrigger()
                self.frames_expected += 1
            else:
                #path = env.paths.directory
                self.cam_path = env.paths.directory + '/scan_%06d_%s' % (self.dataid, self.name)
                fn = 'scan_%06d_%s_%s.h5' % (self.dataid, self.name, self.file_apndx)
                self.saving_file = os.path.join(self.cam_path, fn)
                if os.path.exists(self.saving_file):
                    print('%s: this h5 file exists, I am raising an error now'%self.name)
                    raise Exception('%s h5 file already exists' % self.name)
                self.proxy.DestinationFilename = self.saving_file
                self.file_apndx += 1
                self.frames_expected += self.burst_n
                self.proxy.Arm()
                self.proxy.start()
        except Exception as e:
            print('Detector is running!!')

    def stop(self):
        self.proxy.Stop()

    def busy(self):
        st = self.proxy.State()
        if st == PyTango.DevState.RUNNING and self.burst_n != 1:
            return True
        if st in (PyTango.DevState.STANDBY, PyTango.DevState.ON):
            return False
        elif st == PyTango.DevState.RUNNING:
            if self.proxy.nFramesAcquired == self.frames_expected:
                return False
        else:
            return True

    def read(self):
        if self.saving_file == '':
            return None
        else:
            path, f_name = os.path.split(self.proxy.DestinationFilename)
            #print(f'Andor file path: {path}, filename: {f_name}')
            return Link(self.proxy.DestinationFilename, 'entry/data/data', universal=False)