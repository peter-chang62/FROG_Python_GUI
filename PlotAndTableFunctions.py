"""This file should handle all the customized plotting and table widgets"""

import pyqtgraph as pg
import PyQt5.QtWidgets as qt
import PyQt5.QtGui as qtg
from matplotlib import cm
import numpy as np
from scipy.misc import face


class PlotWidget(pg.PlotWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.setBackground('w')
        self.getAxis('left').setPen('k')
        self.getAxis('bottom').setPen('k')
        self.getAxis('left').setTextPen('k')
        self.getAxis('bottom').setTextPen('k')

        self.xmin, self.xmax = 0., 1.
        self.ymin, self.ymax = 0., 1.

    def set_xlabel(self, label):
        self.getAxis('bottom').setLabel(label)

    def set_ylabel(self, label):
        self.getAxis('left').setLabel(label)

    def set_xmin(self, xmin):
        self.setXRange(xmin, self.xmax)
        self.xmin = xmin

    def set_xmax(self, xmax):
        self.setXRange(self.xmin, xmax)
        self.xmax = xmax

    def set_ymin(self, ymin):
        self.setYRange(ymin, self.ymax)
        self.ymin = ymin

    def set_ymax(self, ymax):
        self.setYRange(self.ymin, ymax)
        self.ymax = ymax


def create_curve(color='b', width=2, x=None, y=None):
    curve = pg.PlotDataItem(pen=pg.mkPen(color=color, width=width))
    if (x is not None) and (y is not None):
        curve.setData(x, y)
    return curve


class PlotWindow:
    def __init__(self, le_wl_ll, le_wl_ul, le_ll, le_ul, plotwidget):
        le_wl_ll: qt.QLineEdit
        le_wl_ul: qt.QLineEdit
        le_ll: qt.QLineEdit
        le_ul: qt.QLineEdit
        plotwidget: PlotWidget

        self.plotwidget = plotwidget
        self.le_wl_ll = le_wl_ll
        self.le_wl_ul = le_wl_ul
        self.le_ll = le_ll
        self.le_ul = le_ul

        self.le_wl_ll.setValidator(qtg.QDoubleValidator())
        self.le_wl_ul.setValidator(qtg.QDoubleValidator())
        self.le_ll.setValidator(qtg.QDoubleValidator())
        self.le_ul.setValidator(qtg.QDoubleValidator())

        self.connect()

    @property
    def ymax(self):
        return self.plotwidget.ymax

    @ymax.setter
    def ymax(self, ymax):
        self.plotwidget.set_ymax(ymax)

    @property
    def ymin(self):
        return self.plotwidget.ymin

    @ymin.setter
    def ymin(self, ymin):
        self.plotwidget.set_ymin(ymin)

    @property
    def xmax(self):
        return self.plotwidget.xmax

    @xmax.setter
    def xmax(self, xmax):
        self.plotwidget.set_xmax(xmax)

    @property
    def xmin(self):
        return self.plotwidget.xmin

    @xmin.setter
    def xmin(self, xmin):
        self.plotwidget.set_xmin(xmin)

    def update_xmax(self):
        xmax = float(self.le_wl_ul.text())
        self.xmax = xmax

    def update_xmin(self):
        xmin = float(self.le_wl_ll.text())
        self.xmin = xmin

    def update_ymax(self):
        ymax = float(self.le_ul.text())
        self.ymax = ymax

    def update_ymin(self):
        ymin = float(self.le_ll.text())
        self.ymin = ymin

    def connect(self):
        self.le_ul.editingFinished.connect(self.update_ymax)
        self.le_ll.editingFinished.connect(self.update_ymin)
        self.le_wl_ul.editingFinished.connect(self.update_xmax)
        self.le_wl_ll.editingFinished.connect(self.update_xmin)

    def update_line_edits_to_properties(self):
        self.le_ll.setText('%.3f' % self.ymin)
        self.le_ul.setText('%.3f' % self.ymax)
        self.le_wl_ll.setText('%.3f' % self.xmin)
        self.le_wl_ul.setText('%.3f' % self.xmax)

    def format_to_current_viewBox(self):
        rect = self.plotwidget.viewRect()
        self.xmin, self.xmax = rect.left(), rect.right()
        self.ymin, self.ymax = rect.bottom(), rect.top()
        self.update_line_edits_to_properties()

    def format_to_curve(self, curve):
        curve: pg.PlotDataItem
        self.xmin, self.xmax = curve.xData[[0, -1]]
        self.ymin, self.ymax = curve.yData[[0, -1]]
        self.update_line_edits_to_properties()

    def format_to_xy_data(self, x, y):
        self.xmin, self.xmax = x[[0, -1]]
        self.ymin, self.ymax = y[[0, -1]]
        self.update_line_edits_to_properties()


def get_colormap(string):
    pos = np.linspace(0, 1, 300)
    lut = cm.get_cmap(string)(pos) * 255
    return pos, lut


# The following should also be able to be passed in as an argument to the
# init function of PlotWindow (in place of PlotWidget) However, note the
# format_to_current_viewBox method will format it to something really big. I
# don't think I'll use that method but if you do you should change
# plotwidget.viewRect() to plotwidget.PlotItem.viewRect()
class ImageWithAxisWidget(pg.GraphicsLayoutWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.show()
        self.setBackground('w')

        self.PlotItem = pg.PlotItem()
        self.addItem(self.PlotItem)

        self.PlotItem.getAxis('left').setPen('k')
        self.PlotItem.getAxis('bottom').setPen('k')
        self.PlotItem.getAxis('left').setTextPen('k')
        self.PlotItem.getAxis('bottom').setTextPen('k')

        self.ii = pg.ImageItem()
        self.PlotItem.addItem(self.ii)

        # self.plot_image()

        self.xmin, self.xmax = 0., 1.
        self.ymin, self.ymax = 0., 1.

    def set_xlabel(self, label):
        self.PlotItem.getAxis('bottom').setLabel(label)

    def set_ylabel(self, label):
        self.PlotItem.getAxis('left').setLabel(label)

    def set_xmin(self, xmin):
        self.PlotItem.setXRange(xmin, self.xmax)
        self.xmin = xmin

    def set_xmax(self, xmax):
        self.PlotItem.setXRange(self.xmin, xmax)
        self.xmax = xmax

    def set_ymin(self, ymin):
        self.PlotItem.setYRange(ymin, self.ymax)
        self.ymin = ymin

    def set_ymax(self, ymax):
        self.PlotItem.setYRange(self.ymin, ymax)
        self.ymax = ymax

    def set_cmap(self, cmap='nipy_spectral'):
        _, lut = get_colormap(cmap)
        self.ii.setLookupTable(lut)

    def scale_axes(self, x=np.array([0, 1]), y=np.array([0, 1]), format='xy'):

        # reset the transformation or else each time you collect a spectrogram
        # it shrinks the plot
        self.ii.resetTransform()

        xlims, ylims = x[[0, -1]], y[[0, -1]]
        x0, y0 = xlims[0], ylims[0]

        if format == 'ij':
            yscale, xscale = len(x), len(y)
            self.ii.translate(y0, x0)
            self.ii.scale(np.diff(ylims) / yscale, np.diff(xlims) / xscale)
        elif format == 'xy':
            xscale, yscale = len(x), len(y)
            self.ii.translate(x0, y0)
            self.ii.scale(np.diff(xlims) / xscale, np.diff(ylims) / yscale)
        else:
            raise ValueError("format should be 'ij' or 'xy'")

    def plot_image(self, data):
        self.ii.setImage(data)
