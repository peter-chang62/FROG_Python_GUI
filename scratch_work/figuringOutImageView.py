import pyqtgraph as pg
from matplotlib import cm
import PyQt5.QtWidgets as qt
import numpy as np
from scipy.misc import face
from PyQt5 import QtCore as qtc


def get_colormap(string):
    pos = np.linspace(0, 1, 300)
    lut = cm.get_cmap(string)(pos) * 255
    return pos, lut


class Image(pg.ImageView):
    def __init__(self):
        super().__init__()
        self.show()

        pos, lut = get_colormap('viridis')

        self.setColorMap(pg.ColorMap(pos, lut))
        self.setImage(face(True))
        self.ui.roiBtn.hide()
        self.ui.menuBtn.hide()
        self.ui.histogram.hide()


class Graphics(pg.GraphicsView):
    def __init__(self):
        super().__init__()
        self.show()

        pos, lut = get_colormap('afmhot')
        imagedata = face(True)
        ii = pg.ImageItem(imagedata)
        ii.setLookupTable(lut)
        ii.setRect(self.viewRect())

        self.addItem(ii)
        self.setBackground('w')


class GraphicsView(pg.GraphicsView):
    def __init__(self):
        super().__init__()
        self.show()

        self.setBackground('w')

    def plot_image(self, data=face(True), cmap='viridis'):
        ii = pg.ImageItem(data)
        _, lut = get_colormap(cmap)
        ii.setLookupTable(lut)
        ii.setRect(self.viewRect())

        self.addItem(ii)


# FINAL ONE
class GraphicsViewWithAxis(pg.GraphicsLayoutWidget):
    def __init__(self):
        super().__init__()
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

    def plot_image(self, data=face(True), cmap='viridis'):
        _, lut = get_colormap(cmap)
        self.ii.setImage(data)
        self.ii.setLookupTable(lut)


if __name__ == '__main__':
    app = qt.QApplication([])
    hey = GraphicsViewWithAxis()
    hey.plot_image()
    app.exec()
