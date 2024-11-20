from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_temp(object):
    def setupUi(self, temp):
        temp.setObjectName("temp")
        temp.resize(615, 588)
        self.gridLayoutWidget = QtWidgets.QWidget(parent=temp)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(50, 100, 521, 431))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.graftemp = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.graftemp.setContentsMargins(0, 0, 0, 0)
        self.graftemp.setObjectName("graftemp")
        self.title_temp = QtWidgets.QLabel(parent=temp)
        self.title_temp.setGeometry(QtCore.QRect(120, 40, 391, 31))
        font = QtGui.QFont()
        font.setFamily("Segoe UI")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.title_temp.setFont(font)
        self.title_temp.setObjectName("title_temp")

        self.retranslateUi(temp)
        QtCore.QMetaObject.connectSlotsByName(temp)

    def retranslateUi(self, temp):
        _translate = QtCore.QCoreApplication.translate
        temp.setWindowTitle(_translate("temp", "Form"))
        self.title_temp.setText(_translate("temp", "Gráficos de temperatura"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    temp = QtWidgets.QWidget()
    ui = Ui_temp()
    ui.setupUi(temp)
    temp.show()
    sys.exit(app.exec())
