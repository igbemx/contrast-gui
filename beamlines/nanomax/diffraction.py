"""
The diffraction endstation at NanoMAX.
"""

# need this main guard here because Process.start() (so our recorders)
# import __main__, and we don't want the subprocess to start new sub-
# processes etc.
if __name__ == '__main__':
    import contrast
    from contrast.environment import env, runCommand
    from contrast.environment.data import SdmPathFixer
    from contrast.environment.scheduling import MaxivScheduler
    from contrast.recorders import Hdf5Recorder, StreamRecorder, ScicatRecorder
    from contrast.motors import DummyMotor, MotorMemorizer
    from contrast.motors.LC400 import LC400Motor
    from contrast.detectors.LC400Buffer import LC400Buffer
    from contrast.motors.TangoMotor import TangoMotor
    from contrast.motors.TangoAttributeMotor import TangoAttributeMotor
    from contrast.motors.SmaractMotor import SmaractLinearMotor
    from contrast.motors.SmaractMotor import SmaractRotationMotor
    from contrast.motors.E727 import E727Motor
    from contrast.motors.KukaMotor import KukaRobot
    from contrast.detectors.Pilatus import Pilatus2, Pilatus3
    from contrast.detectors.Merlin import Merlin
    from contrast.detectors.Xspress3 import Xspress3
    from contrast.detectors.Andor3 import Andor3
    from contrast.detectors.Eiger import Eiger
    from contrast.detectors.AlbaEM import AlbaEM
    from contrast.detectors.PandaBox import PandaBox
    from contrast.detectors import Detector, PseudoDetector
    from contrast.detectors.DG645 import StanfordTriggerSource
    from contrast.detectors.Keysight import Keysight2985
    from contrast.detectors.BaslerCamera import BaslerCamera
    from contrast.detectors.TangoAttributeDetector import TangoAttributeDetector
    from contrast.scans import SoftwareScan, Ct
    import macros_common
    import macros_diff
    import os
    import time

    # warn if we are not nanomax-service with correct umask
    user = os.popen('whoami').read().strip()
    umask = os.popen('umask').read().strip()
    if not (user == 'nanomax-service' and umask =='0022'):
        print(
            '\033[91mWARNING! The correct way of running the beamline'
            ' is as nanomax-service with umask 022\033[0m'
        )

    # add a scheduler to pause scans when shutters close
    env.scheduler = MaxivScheduler(
        proxy_device='b303a/ctl/proxy-01',
        shutter_list=['B303A-FE/VAC/HA-01',
                      'B303A-FE/PSS/BS-01',
                      'B303A-O/PSS/BS-01',
                      'B303A-E/PSS/BS-01'],
        avoid_injections=False,
        respect_countdown=False
    )

    env.userLevel = 2
    # arbitrarily chosen these levels:
    # 1 - simple user
    # 2 - power user
    # 3 - optics
    # 4 - potentially dangerous

    # PI NanoCube 3-axis piezo. To be used in temporary setups
    # sx = E727Motor(device='B303A-EH/CTL/PZCU-02', axis=1, name='sx', userlevel=1, scaling=-1.0, dial_limits=(0,100), user_format='%.3f', dial_format='%.3f')
    # sy = E727Motor(device='B303A-EH/CTL/PZCU-02', axis=3, name='sy', userlevel=1, dial_limits=(0,100), user_format='%.3f', dial_format='%.3f')
    # sz = E727Motor(device='B303A-EH/CTL/PZCU-02', aos.popen('whoami').read().strip()xis=2, name='sz', userlevel=1, dial_limits=(0,100), user_format='%.3f', dial_format='%.3f')

    # sample piezos
    sx = LC400Motor(device='B303A/CTL/PZCU-LC400B', axis=2, name='sx', scaling=-1.0, dial_limits=(-50,50), user_format='%.3f')
    sy = LC400Motor(device='B303A/CTL/PZCU-LC400B', axis=3, name='sy', dial_limits=(-50,50), user_format='%.3f')
    sz = LC400Motor(device='B303A/CTL/PZCU-LC400B', axis=1, name='sz', scaling=-1.0, dial_limits=(-50,50), user_format='%.3f')

    # Xerion rotation stage
    # sr = TangoMotor(device='xeryon/test/ulfjoh', name='sr', userlevel=1)

    # base motors through sardana
    basex = TangoMotor(device='motor/icepap_ctrl_1_expert/16', name='basex', userlevel=1)
    basey = TangoMotor(device='motor/icepap_ctrl_1_expert/17', name='basey', userlevel=1)
    basez = TangoMotor(device='motor/icepap_ctrl_1_expert/18', name='basez', userlevel=1)

    # gap and taper via a proxy in the local pool
    ivu_gap = TangoMotor(device='motor/ivu_gap_ctrl/1', name='ivu_gap', userlevel=2, dial_limits=(4.5, 25), user_format='%.4f')
    ivu_taper = TangoMotor(device='motor/ivu_taper_ctrl/1', name='ivu_taper', userlevel=4, dial_limits=(-.05, .05), user_format='%.4f')

    # ring current via a local proxy
    ring_current = TangoAttributeDetector(device='b303a/ctl/proxy-01', attribute='r3current', name='ring_current')

    # Diamond filter motors, sitting in diagnostics module 1
    bl_filter_1 = TangoMotor(device='b303a-o/opt/flt-01-yml', name='bl_filter_1', userlevel=6, dial_limits=(-36.04, 36.77))
    bl_filter_2 = TangoMotor(device='b303a-o/opt/flt-02-yml', name='bl_filter_2', userlevel=6, dial_limits=(-36.24, 38.46))

    # Vertical focusing mirror motors
    vfm_x = TangoMotor(device='b303a-o/opt/mir-01-xml', name='vfm_x', userlevel=6, dial_limits=(-4.53, 1.2), user_format='%.3f')
    vfm_y = TangoMotor(device='b303a-o/opt/mir-01-yml', name='vfm_y', userlevel=6, dial_limits=(-15.24, 15.91), user_format='%.3f')
    vfm_pit = TangoMotor(device='b303a-o/opt/mir-01-pitml', name='vfm_pit', userlevel=6, dial_limits=(2.65, 2.85), user_format='%.3f')
    vfm_yaw = TangoMotor(device='b303a-o/opt/mir-01-yawml', name='vfm_yaw', userlevel=6, dial_limits=(-1.43, 1.42), user_format='%.3f')

    # Horizontal focusing mirror motors
    hfm_x = TangoMotor(device='b303a-o/opt/mir-02-xml', name='hfm_x', userlevel=6, dial_limits=(-2.05, 0.1), user_format='%.3f')
    hfm_y = TangoMotor(device='b303a-o/opt/mir-02-yml', name='hfm_y', userlevel=2, dial_limits=(-15.33, 14.71), user_format='%.3f')
    hfm_pit = TangoMotor(device='b303a-o/opt/mir-02-pitml', name='hfm_pit', userlevel=6, dial_limits=(2.65, 2.85), user_format='%.3f')
    hfm_bend = TangoMotor(device='b303a-o/opt/mir-02-bendml', name='hfm_bend', userlevel=6)

    # Monochromator motors
    mono_x = TangoMotor(device='b303a-o/opt/mono-xml', name='mono_x', userlevel=6, dial_limits=(-2.4, 3.87), user_format='%.3f')
    mono_bragg = TangoMotor(device='b303a-o/opt/MONO-BRAGML', name='mono_bragg', userlevel=4, dial_limits=(4.0, 27.46))
    mono_x2per = TangoMotor(device='b303a-o/opt/mono-perml', name='mono_x2per', userlevel=2, dial_limits=(-.1, .1), user_format='%.3f')
    mono_x2pit = TangoMotor(device='b303a-o/opt/mono-pitml', name='mono_x2pit', userlevel=4, dial_limits=(-1.21, 1.21), user_format='%.4f')
    mono_x2rol = TangoMotor(device='b303a-o/opt/mono-rolml', name='mono_x2rol', userlevel=4, dial_limits=(-0.8, 0.79), user_format='%.4f')
    mono_x2fpit = TangoMotor(device='B303A-O/CTL/PZCU-01', name='mono_x2fpit', userlevel=1, dial_limits=(0., 12.), user_format='%.2f')
    mono_x2frol = TangoMotor(device='B303A-O/CTL/PZCU-02', name='mono_x2frol', userlevel=1, dial_limits=(0., 12.), user_format='%.2f')

    # Nanobpm motor. Positions the bpm vertically in the beam. Almost never moved. Should be at 2.5 mm
    nanobpm_y = TangoMotor(device='b303a-o/dia/bpx-01', name='nanobpm_y', userlevel=6, dial_limits=(-0.1, 23.1))

    # buffered position detector - internal position recording is
    # not configured in the NpointFlyscan macro right now!
    # npoint_buff = LC400Buffer(device='B303A/CTL/FLYSCAN-02', name='npoint_buff', xaxis=2, yaxis=3, zaxis=1)
    # npoint_buff.active = False # this can be switched on from flyscanning macros when needed, although it does no harm.

    # smaracts
    # controller 1
    skb_top = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-03', axis=0, name='skb_top', userlevel=2, velocity=1000)
    skb_bottom = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-03', axis=1, name='skb_bottom', userlevel=2, velocity=1000)
    skb_left = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-03', axis=2, name='skb_left', userlevel=2, velocity=1000)
    skb_right = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-03', axis=3, name='skb_right', userlevel=2, velocity=1000)
    # kbfluox = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-03', axis=4, name='kbfluox', userlevel=3)
    # sr = SmaractRotationMotor(device='B303A-EH/CTL/PZCU-03', axis=5, name='sr', userlevel=1, user_format='%.3f', dial_format='%.3f')
    pinhole_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-03', axis=6, name='pinhole_x', userlevel=3, velocity=1000)
    pinhole_y = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-03', axis=7, name='pinhole_y', userlevel=3, velocity=1000)
    pinhole_z = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-03', axis=8, name='pinhole_z', userlevel=3, velocity=1000)

    # controller 2
    dbpm2_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=0, name='dbpm2_x', userlevel=3, velocity=1000)
    dbpm2_y = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=1, name='dbpm2_y', userlevel=3, velocity=1000)
    seh_top = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=2, name='seh_top', userlevel=3, velocity=1000)
    seh_bottom = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=3, name='seh_bottom', userlevel=3, velocity=1000)
    seh_left = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=4, name='seh_left', userlevel=3, velocity=1000)
    seh_right = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=5, name='seh_right', userlevel=3, velocity=1000)
    attenuator1_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=6, name='attenuator1_x', userlevel=2, velocity=5000)
    attenuator2_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=7, name='attenuator2_x', userlevel=2, velocity=5000)
    attenuator3_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=8, name='attenuator3_x', userlevel=2, velocity=5000)
    attenuator4_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=9, name='attenuator4_x', userlevel=2, velocity=5000)
    # fastshutter_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=10, name='fastshutter_x', userlevel=3)
    diode1_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=11, name='diode1_x', userlevel=3, velocity=5000)
    pol_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=12, name='pol_x', userlevel=2, velocity=1000)
    pol_y = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-04', axis=13, name='pol_y', userlevel=2, velocity=1000)
    pol_rot = SmaractRotationMotor(device='B303A-EH/CTL/PZCU-04', axis=14, name='pol_rot', userlevel=2, user_format='%.8f', dial_format='%.8f')
    
    ## controller 3
    # stages removed from DM4
    # xeol_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-05', axis=0, name='xeol_x', userlevel=1, velocity=1000)
    # xeol_y = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-05', axis=1, name='xeol_y', userlevel=1, scaling=-1, velocity=1000)
    # xeol_z = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-05', axis=2, name='xeol_z', userlevel=1, scaling=-1, velocity=1000)
    # we use ch0 on that controller for the long range sample motor for now
    # samplez = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-05', axis=0, name='samplez', userlevel=1)

    # controller 4 in OH2 for fast shutter and first diamondBPM
    fastshutter_y = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-07', axis=0, name='fastshutter_y', userlevel=3, velocity=1000)
    dbpm1_x = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-07', axis=1, name='dbpm1_x', userlevel=3, velocity=1000)
    dbpm1_y = SmaractLinearMotor(device='B303A-EH/CTL/PZCU-07', axis=2, name='dbpm1_y', userlevel=3, velocity=1000)

    # gap and offset pseudo motors for slits in DM4
    seh_posy = macros_common.SlitOffset([seh_top, seh_bottom], name="seh_posy", userlevel=4)
    seh_gapy = macros_common.SlitGap([seh_top, seh_bottom], name="seh_gapy", userlevel=4)
    seh_posx = macros_common.SlitOffset([seh_left, seh_right], name="seh_posx", userlevel=4)
    seh_gapx = macros_common.SlitGap([seh_left, seh_right], name="seh_gapx", userlevel=4)


    # gap and offset pseudo motors for slits in front of KB
    skb_posy = macros_common.SlitOffset([skb_top, skb_bottom], name="skb_posy", userlevel=4)
    skb_gapy = macros_common.SlitGap([skb_top, skb_bottom], name="skb_gapy", userlevel=4)
    skb_posx = macros_common.SlitOffset([skb_left, skb_right], name="skb_posx", userlevel=4)
    skb_gapx = macros_common.SlitGap([skb_left, skb_right], name="skb_gapx", userlevel=4)

    # KB mirror pitch piezos
    m1froll = E727Motor(device='B303A-EH/CTL/PZCU-01', axis=1, name='m1froll', userlevel=2, dial_limits=(0,30))
    m1fpitch = E727Motor(device='B303A-EH/CTL/PZCU-01', axis=2, name='m1fpitch', userlevel=2, dial_limits=(0,30))
    m2fpitch = E727Motor(device='B303A-EH/CTL/PZCU-01', axis=3, name='m2fpitch', userlevel=2, dial_limits=(0,30))

    # Robot
    # gamma, delta, radius = KukaRobot('B303-EH2/CTL/DM-02-ROBOT', names=['gamma', 'delta', 'radius'])

    # SSA through the Pool
    ssa_gapx = TangoMotor(device='B303A-O/opt/SLIT-01-GAPXPM', name='ssa_gapx', userlevel=2)
    ssa_gapy = TangoMotor(device='B303A-O/opt/SLIT-01-GAPYPM', name='ssa_gapy', userlevel=2)
    ssa_posx = TangoMotor(device='B303A-O/opt/SLIT-01-POSXPM', name='ssa_posx', userlevel=3)
    ssa_posy = TangoMotor(device='B303A-O/opt/SLIT-01-POSYPM', name='ssa_posy', userlevel=3)

    # microscope motors through the Pool
    oam_x = TangoMotor(device='b303a-e02/dia/om-01-x', name='oam_x', userlevel=4, user_format='%.4f', dial_format='%.4f')
    oam_y = TangoMotor(device='b303a-e02/dia/om-01-y', name='oam_y', userlevel=4, user_format='%.4f', dial_format='%.4f')
    oam_z = TangoMotor(device='b303a-e02/dia/om-01-z', name='oam_z', userlevel=4, user_format='%.4f', dial_format='%.4f')
    oam_zoom = TangoMotor(device='b303a-e02/dia/om-01-zoom', name='oam_zoom', userlevel=1)
    topm_x = TangoMotor(device='b303a-e02/dia/om-02-x', name='topm_x', userlevel=4, user_format='%.4f', dial_format='%.4f')
    topm_y = TangoMotor(device='b303a-e02/dia/om-02-y', name='topm_y', userlevel=4, user_format='%.4f', dial_format='%.4f')
    topm_z = TangoMotor(device='b303a-e02/dia/om-02-z', name='topm_z', userlevel=4, user_format='%.4f', dial_format='%.4f')
    topm_zoom = TangoMotor(device='b303a-e02/dia/om-02-zoom', name='topm_zoom', userlevel=1)

    # goniometer
    gontheta = TangoMotor(device='b303a-e02/dia/gon-01-theta', name='gontheta', userlevel=3, user_format='%.4f', dial_format='%.4f')
    gonphi = TangoMotor(device='b303a-e02/dia/gon-01-phi', name='gonphi', userlevel=3, user_format='%.4f', dial_format='%.4f')
    gonx1 = TangoMotor(device='b303a-e02/dia/gon-01-x1', name='gonx1', userlevel=4)
    gonx2 = TangoMotor(device='b303a-e02/dia/gon-01-x2', name='gonx2', userlevel=4)
    gonx3 = TangoMotor(device='b303a-e02/dia/gon-01-x3', name='gonx3', userlevel=4)
    gony1 = TangoMotor(device='b303a-e02/dia/gon-01-y1', name='gony1', userlevel=4)
    gony2 = TangoMotor(device='b303a-e02/dia/gon-01-y2', name='gony2', userlevel=4)
    gonz = TangoMotor(device='b303a-e02/dia/gon-01-z', name='gonz', userlevel=4)

    beamstop_x = TangoMotor(device='B303A-E02/DIA/SAMS-01-X', name='beamstop_x', userlevel=2, scaling=1.0)
    beamstop_y = TangoMotor(device='B303A-E02/DIA/SAMS-01-Y', name='beamstop_y', userlevel=2, scaling=1.0)
    # sams_z = TangoMotor(device='B303A-E02/DIA/SAMS-01-Z', name='sams_z', userlevel=4, scaling=-1.0)

    # xrf detector linear motion
    xrf_x = TangoMotor(device='B303A-E02/DIA/DMA-01-X', name='xrf_x', userlevel=4, scaling=1.25, dial_limits=(0, 75))

    # detector motors
    # detx = TangoMotor(device='motor/icepap_ctrl_1_expert/11', name='detx', userlevel=3, dial_limits=(0, 295))
    # dety = TangoMotor(device='motor/icepap_ctrl_1_expert/12', name='dety', userlevel=3, dial_limits=(0, 31))

    # table motors
    table_front_x = TangoMotor(device='b303a-e02/dia/tab-01-x1', name='table_front_x', userlevel=5, dial_limits=(-10,10))
    table_back_x = TangoMotor(device='b303a-e02/dia/tab-01-x2', name='table_back_x', userlevel=5, dial_limits=(-10,10))
    table_front_y = TangoMotor(device='b303a-e02/dia/tab-01-y1', name='table_front_y', userlevel=5, dial_limits=(-10,10))
    table_back_y = TangoMotor(device='b303a-e02/dia/tab-01-y2', name='table_back_y', userlevel=5, dial_limits=(-10,10))

    # some sardana pseudo motors - these are reimplemented but just need to be configured
    energy_raw = TangoMotor(device='pseudomotor/nanomaxenergy_ctrl/1', name='energy_raw')
    energy = TangoMotor(device='pseudomotor/nanomaxenergy_corr_ctrl/1', name='energy')

    # some dummy motors
    dummy1 = DummyMotor(name='dummy1', userlevel=2)
    dummy2 = DummyMotor(name='dummy2', userlevel=2)

    # The delay generator as a software source for hardware triggers
    # stanford = StanfordTriggerSource(name='stanford', device_name='B303A-A100380CAB03/CTL/DLY-01')

    # detectors
    #pilatus = Pilatus2(name='pilatus',
    #                 hostname='b-nanomax-mobile-ipc-01')
    pilatus = Pilatus3('b303a/dia/pilatus', name='pilatus3')
    merlin = Merlin(name='merlin', host='localhost')
    xspress3 = Xspress3(name='xspress3', device='staff/alebjo/xspress3')
    
    #andor = Andor3(name='andor', device='b303a-e01/dia/zyla')
    # settings for DESY Andor, needs to be changed for the NanoMAX Crytur Andor
    #andor.proxy.rotation=1
    #andor.proxy.flipud=False
    #andor.proxy.fliplr=False
    #andor.proxy.sensorcooling=True

    #eiger4m = Eiger(name='eiger4m', host='b-nanomax-eiger-dc-1')
    eiger1m = Eiger(name='eiger1m', host='b-nanomax-eiger-1m-0')
    # eiger500k = Eiger(name='eiger500k', host='b-nanomax-eiger-500k-0')
    # alba0 = AlbaEM(name='alba0', host='b-nanomax-em2-0')
    alba2 = AlbaEM(name='alba2', host='b-nanomax-em2-2')
    #E02_oam = BaslerCamera(name='oam', device='basler/on_axis_microscope/main')
    #E02_topm = BaslerCamera(name='topm', device='basler/top_microscope/main')
    #E01cam01 = BaslerCamera(name='E01cam01', device='basler/e01-cam-01/main')
    #E01cam02 = BaslerCamera(name='E01cam02', device='basler/e01-cam-02/main')
    #E01cam03 = BaslerCamera(name='E01cam03', device='basler/e01-cam-03/main')
    #E01cam04 = BaslerCamera(name='E01cam04', device='basler/e01-cam-04/main')

    # The pandabox and some related pseudodetectors
    panda0 = PandaBox(name='panda0', host='b-nanomax-pandabox-0')
    macros_common.NpointFlyscan.panda = panda0
    pseudo = PseudoDetector(name='pseudo',
                            variables={#'c1': 'panda0/INENC1.VAL_Mean',
                                       #'c2': 'panda0/INENC2.VAL_Mean',
                                       #'c3': 'panda0/INENC3.VAL_Mean',
                                       'adc1': 'panda0/FMC_IN.VAL1_Mean',
                                       'adc2': 'panda0/FMC_IN.VAL2_Mean',
                                       'adc3': 'panda0/FMC_IN.VAL3_Mean'},
                            expression={#'x': 'c2', 'y': 'c3', 'z': 'c1',
                                        'x': '-adc2*10', 'y': 'adc3*5', 'z': '-adc1*10'})
#                                        'analog_x': '-adc2*5*10/2**31', 'analog_y': 'adc3*5*10/2**31', 'analog_z': '-adc1*5*10/2**31'})

    # The keysight as both a detector (ammeter) and motor (bias voltage)
    # keysight = Keysight2985(name='keysight', device='B303A-EH/CTL/KEYSIGHT-01')
    # keysight_bias = TangoAttributeMotor(name='keysight_bias', device='B303A-EH/CTL/KEYSIGHT-01', attribute='bias_voltage')

    # the environment keeps track of where to write data
    env.paths = SdmPathFixer('B303A-E02/CTL/SDM-01')

    # an hdf5 recorder
    h5rec = Hdf5Recorder(name='h5rec')
    h5rec.start()

    # a zmq recorder
    zmqrec = StreamRecorder(name='zmqrec')
    zmqrec.start()

    # a scicat recorder - paused until further notice
    # scicatrec = ScicatRecorder(name='scicatrec')
    # scicatrec.start()

    # default detector selection
    for d in Detector.getinstances():
        d.active = False
    for d in [panda0, pseudo, alba2, ring_current]:  # eiger,
        d.active = True

    # define pre- and post-scan actions, per scan base class
    def pre_scan_stuff(slf):
        assert h5rec.is_alive(), 'hdf5 recorder is dead! this can''t be good. maybe restart contrast.'
        runCommand('stoplive')
        runCommand('fsopen')
        time.sleep(0.2)

    def post_scan_stuff(slf):
        runCommand('fsclose')
        pass

    SoftwareScan._before_scan = pre_scan_stuff
    SoftwareScan._after_scan = post_scan_stuff
    Ct._before_ct = pre_scan_stuff
    Ct._after_ct = post_scan_stuff

    contrast.wisdom()

    # find the latest scan number and initialize env.nextScanID
    try:
        l = os.listdir(env.paths.directory)
        last = max(
            [int(l_[:-3]) for l_ in l if (len(l_) == 9 and l_.endswith('.h5'))]
        )
        env.nextScanID = last + 1
        print(f'\nNote: inferring that the next scan number should be {last+1}')
    except:
        pass

    # add a memorizer so the motors keep their user positions and limits
    # after a restart note that this will overwrite the dial positions
    # set above! delete the file to generate it again.
    memorizer = MotorMemorizer(
        name='memorizer', filepath='/data/visitors/nanomax/common/.memorizer')

