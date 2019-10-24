import time
import numpy as np
import os, ast

from ..Gadget import Gadget
from ..environment import macro, env
from .. import utils

class Motor(Gadget):
    """
    General base class for motors.

    Motors have dial and user positions, which are related by an offset
    and a scaling factor. The user position is supposed to be used in
    all macros and movements. The dial position is hard coded in the Motor
    subclass to closely reflect what the underlying device quotes. ::

        user = dial * scaling + offset,
        dial = (user - offset) / scaling

    The user position can be changed, which updates the offset but not
    the scaling. In the same way, limits on the user position are internally
    converted to dial limits, such that setting the user position leaves
    the dial limits unchanged.
    """

    def __init__(self, scaling=1.0, offset=0.0, dial_limits=(None, None),
                 user_format='%.2f', dial_format='%.2f', **kwargs):
        """
        :param scaling: Scaling factor from dial to user position
        :param offset: Offset from scaled dial to user position
        :param dial_limits: Motor limits in dial positions
        :param user_format: Format string for presenting user positions
        :param dial_format: Format string for presenting dial positions
        :param ``**kwargs``: Passed on to base class constructor
        """
        super(Motor, self).__init__(**kwargs)
        self._lowlim, self._uplim = dial_limits
        self._scaling = scaling
        self._offset = offset
        self._uformat = user_format
        self._dformat = dial_format

    @property
    def user_position(self):
        return self.dial_position * self._scaling + self._offset

    @user_position.setter
    def user_position(self, pos):
        self._offset = pos - self.dial_position * self._scaling

    @property
    def user_limits(self):
        if None not in (self._uplim, self._lowlim):
            l1 = self._lowlim * self._scaling + self._offset
            l2 = self._uplim * self._scaling + self._offset
        else:
            low, up = None, None
        return tuple(sorted([l1, l2]))

    @user_limits.setter
    def user_limits(self, lims):
        l1 = (lims[0] - self._offset) / self._scaling
        l2 = (lims[1] - self._offset) / self._scaling
        self._lowlim, self._uplim = sorted([l1, l2])

    @property
    def dial_limits(self):
        return (self._lowlim, self._uplim)

    @dial_limits.setter
    def dial_limits(self, lims):
        self._lowlim, self._uplim = lims

    def position(self):
        return self.user_position

    def move(self, pos):
        if self.busy():
            raise Exception('Motor is busy')
        dial = (pos - self._offset) / self._scaling
        try:
            assert dial <= self._uplim
            assert dial >= self._lowlim
        except AssertionError:
            print('Trying to move %s outside its limits!' % self.name)
            return -1
        except TypeError:
            pass
        self.dial_position = dial

    @property
    def dial_position(self):
        """
        Override this property, which sets or gets the dial position.

        :rtype: float
        """
        raise NotImplementedError

    @dial_position.setter
    def dial_position(self, pos):
        raise NotImplementedError

    def busy(self):
        """
        Overrides this method, which reports on whether or not the motor
        is busy.

        :rtype: bool
        """
        raise NotImplementedError

    def stop(self):
        """
        Override this method, which stops the motor.
        """
        raise NotImplementedError


class DummyMotor(Motor):
    """
    Dummy motor which can be harmlessly moved with a velocity of 1 / s.
    """
    def __init__(self, *args, **kwargs):
        super(DummyMotor, self).__init__(*args, **kwargs)
        self._aim = 0.0
        self._oldpos = 0.0
        self._started = 0.0
        self.velocity = 1.0

    @property
    def dial_position(self):
        dpos = self._aim - self._oldpos
        dt = time.time() - self._started
        T = abs(dpos / self.velocity)
        if dt < T:
            return self._oldpos + dpos * self.velocity * dt / T
        else:
            return self._aim

    @dial_position.setter
    def dial_position(self, pos):
        self._oldpos = self.position()
        self._started = time.time()
        self._aim = pos

    def busy(self):
        return not np.isclose(self._aim, self.dial_position)

    def stop(self):
        self._aim = self.dial_position


class MotorMemorizer(Gadget):
    """
    Saves or loads motor scaling, offsets, and limits to or from file.
    """
    def __init__(self, filepath=None, **kwargs):
        """
        :param filepath: Path to file where motor information will be dumped
        :type filepath: str
        :param ``**kwargs``: Passed on to the base class
        """
        super(MotorMemorizer, self).__init__(**kwargs)
        self.filepath = filepath
        if filepath and os.path.exists(filepath):
            self.load()

    def load(self):
        """
        Loads memorized motor configurations from ``self.filepath``, and
        applies to matching motors.
        """
        try:
            motors = {m.name: m for m in Motor.getinstances()}
            with open(self.filepath, 'r') as fp:
                for row in fp:
                    dct = ast.literal_eval(row)
                    if dct['name'] in motors:
                        motors[dct['name']]._offset = dct['_offset']
                        if dct['_lowlim']:
                            motors[dct['name']]._lowlim = dct['_lowlim']
                        if dct['_uplim']:
                            motors[dct['name']]._uplim = dct['_uplim']
            print('Loaded motor states from %s' % self.filepath)
        except (FileNotFoundError,):
            print("Memorizer file %s doesn't exist" % self.filepath)

    def dump(self):
        """
        Dumps motor configurations for all motors to ``self.filepath``.
        """
        try:
            with open(self.filepath, 'w') as fp:
                for m in Motor.getinstances():
                    dct = {'name': m.name,
                           '_offset': m._offset,
                           '_lowlim': m._lowlim, '_uplim': m._uplim}
                    fp.write(str(dct) + '\n')
            print('Saved motor states to %s' % self.filepath)
        except (FileNotFoundError,):
            print("Cant write to %s, doesn't exist")

@macro
class Mv(object):
    """
    Move one or more motors. ::

        mvr <motor1> <position1> <motor2> <position2> ...

    """
    def __init__(self, *args):
        self.motors = args[::2]
        self.targets = np.array(args[1::2])

    def _run_while_waiting(self):
        pass
    
    def run(self):
        if max(m.userlevel for m in self.motors) > env.userLevel:
            print('You are trying to move motors above your user level!')
            return
        for m, pos in zip(self.motors, self.targets):
            m.move(pos)
        try:
            while True in [m.busy() for m in self.motors]:
                self._run_while_waiting()
                time.sleep(.01)
        except KeyboardInterrupt:
            for m in self.motors:
                m.stop()

@macro
class Mvd(object):
    """
    Move one or more motors to an abolute dial position. Not implemented.
    """
    def run(self):
        raise NotImplementedError

@macro
class Mvr(Mv):
    """
    Move one or more motors relative to their current positions. ::

        mvr <motor1> <position1> <motor2> <position2> ...

    """
    def __init__(self, *args):
        self.motors = args[::2]
        displacements = np.array(args[1::2])
        current = np.array([m.position() for m in self.motors])
        self.targets = current + displacements

@macro
class Umv(Mv):
    """
    Like mv, but prints the current position while moving, and returns
    when the move is complete.
    """
    def _run_while_waiting(self):
        l = ['%s: %s' % (m.name, m._uformat%m.user_position) for m in self.motors]
        s = '; '.join(l)
        print(s + '\r', end='')

    def run(self):
        # ensures the final position is printed too
        super(Umv, self).run()
        self._run_while_waiting()

@macro
class Umvr(Mvr, Umv):
    """
    Like umv, but in positions relative to the current ones.
    """
    pass # less is more

@macro
class Wm(object):
    """
    Print the positions of one or more motors. ::

        wm <motor1> <motor2> ...
    """
    def __init__(self, *args):
        self.motors = args
        self.out    = True
    def run(self, *args):
        titles = ['motor', 'user pos.', 'limits', 'dial pos.', 'limits']
        table = []
        for m in self.motors:
            try:
                upos = m.user_position
                ret = upos
                upos = m._uformat % upos
                dpos = m._dformat % m.dial_position
                if None in (m._uplim, m._lowlim):
                    ulims = '(None, None)'
                    dlims = '(None, None)'
                else:
                    ulims = ('(%s, %s)' % (2*(m._uformat,))) % m.user_limits
                    dlims = ('(%s, %s)' % (2*(m._dformat,))) % m.dial_limits
                table.append([m.name, upos, ulims, dpos, dlims])
            except:
                print('Could not get position of %s' % m.name)
                ret = None
        if self.out: 
            print(utils.list_to_table(lst=table, titles=titles, margins=[5,2,5,2,0]))
        return ret
@macro
class WmS(Wm):
    """
    Silent 'where motor'. Print the positions of one or more motors but do not print any output. ::

        wms <motor1> <motor2> ...
    """
    def __init__(self, *args):
        self.motors = args
        self.out = False

@macro
class Wa(Wm):
    """
    Print the positions of all motors available at the current user level.
    """
    def __init__(self):
        self.motors = [m for m in Motor.getinstances()
                       if m.userlevel <= env.userLevel]
        self.out = True

@macro
class LsM(object):
    """
    List available motors.
    """
    def run(self):
        dct = {m.name: m.__class__ for m in Motor.getinstances()
               if m.userlevel <= env.userLevel}
        print(utils.dict_to_table(dct, titles=('name', 'class'), sort=True))

@macro
class SetLim(object):
    """
    Set limits on motors. ::

        setlim <motor1> <lower 1> <upper 1> ...

    Also saves new limits to all available ``MotorMemorizer`` objects.
    """
    def __init__(self, *args):
        self.motors = args[::3]
        self.lowers = args[1::3]
        self.uppers = args[2::3]

    def run(self):
        for m, l, u in zip(self.motors, self.lowers, self.uppers):
            m.user_limits = (l, u)

        # memorize the new state
        for m in MotorMemorizer.getinstances():
            m.dump()

@macro
class SetPos(object):
    """
    Sets user position on motors. ::

        setpos <motor1> <pos1> ...

    Also saves new user positions to all available ``MotorMemorizer``
    objects.
    """
    def __init__(self, *args):
        self.motors = args[::2]
        self.positions = args[1::2]

    def run(self):
        for m, p in zip(self.motors, self.positions):
            m.user_position = p

        # memorize the new state
        for m in MotorMemorizer.getinstances():
            m.dump()
