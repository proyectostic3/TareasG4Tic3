import matplotlib
import random
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt6 import QtCore, QtGui, QtWidgets

class Ui_hum(object):
    def setupUi(self, hum):
        hum.setObjectName("hum")
        hum.resize(600, 600)
        self.title_hum = QtWidgets.QLabel(parent=hum)
        self.title_hum.setGeometry(QtCore.QRect(200, 40, 300, 40))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.title_hum.setFont(font)
        self.title_hum.setObjectName("title_hum")
        self.gridLayoutWidget = QtWidgets.QWidget(parent=hum)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(50, 90, 521, 441))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.graf_hum = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.graf_hum.setContentsMargins(0, 0, 0, 0)
        self.graf_hum.setObjectName("graf_hum")

        self.retranslateUi(hum)
        QtCore.QMetaObject.connectSlotsByName(hum)

    def retranslateUi(self, hum):
        _translate = QtCore.QCoreApplication.translate
        hum.setWindowTitle(_translate("hum", "Form"))
        self.title_hum.setText(_translate("hum", "Gráficos de humedad"))



class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)
        
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.sc = MplCanvas(self, width=5, height=4, dpi=100)
        self.xdata = [0,1,2,3,4]
        self.ydata = [10,1,20,3,40]
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.sc)
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()
    def update_plot(self):
        self.sc.axes.cla()
        rand_list=[]
        for i in range(5):
            rand_list.append(random.randint(0,10))
            self.ydata = [x + y for x, y in zip(self.ydata, rand_list)]
            self.sc.axes.plot(self.xdata, self.ydata)
            self.sc.draw()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    hum = QtWidgets.QWidget()
    ui = Ui_hum()
    ui.setupUi(hum)
    hum.show()
    sys.exit(app.exec())
