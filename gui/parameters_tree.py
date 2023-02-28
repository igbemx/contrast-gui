"""
This example demonstrates the use of pyqtgraph's parametertree system. This provides
a simple way to generate user interfaces that control sets of parameters. The example
demonstrates a variety of different parameter types (int, float, list, etc.)
as well as some customized parameter types
"""

# `makeAllParamTypes` creates several parameters from a dictionary of config specs.
# This contains information about the options for each parameter so they can be directly
# inserted into the example parameter tree. To create your own parameters, simply follow
# the guidelines demonstrated by other parameters created here.
#from pyqtgraph._buildParamTypes import makeAllParamTypes

import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets

app = pg.mkQApp("Parameter Tree Example")
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree
import pyqtgraph.parametertree as pt
from softimax_beamline import *

# paramSpec = [
#         dict(name='bool', type='bool', readonly=True),
#         dict(name='color', type='color', readonly=True),
#     ]

paramSpec = []
for k,v in abs_x.__dict__.items():
    if isinstance(v, (float, int, str, list, dict, tuple)):
        paramSpec.append({'name':k, 'type':type(v).__name__, 'value':v})
    else:
        paramSpec.append({'name':k, 'type':'str', 'value':str(v)})

param = pt.Parameter.create(name='params', type='group', children=paramSpec)
tree = pt.ParameterTree()
tree.setParameters(param)


## Create two ParameterTree widgets, both accessing the same data
# t = ParameterTree()
# t.setParameters(p, showTop=False)
# t.setWindowTitle('pyqtgraph example: Parameter Tree')
# t2 = ParameterTree()
# t2.setParameters(p, showTop=False)

win = QtWidgets.QWidget()
layout = QtWidgets.QGridLayout()
win.setLayout(layout)
layout.addWidget(QtWidgets.QLabel("These are two views of the same data. They should always display the same values."), 0,  0, 1, 2)
# layout.addWidget(t, 1, 0, 1, 1)
# layout.addWidget(t2, 1, 1, 1, 1)
layout.addWidget(tree)
win.show()

# ## test save/restore
# state = p.saveState()
# p.restoreState(state)
# compareState = p.saveState()
# assert pg.eq(compareState, state)

if __name__ == '__main__':
    pg.exec()