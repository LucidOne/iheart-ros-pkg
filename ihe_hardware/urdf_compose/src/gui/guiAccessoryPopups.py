import re
import yaml
from xml.dom.minidom import parse
from PySide import QtGui, QtCore
from aux import *

class AddAccessoryDialog(QtGui.QDialog):
    def __init__(self, parent, accName, accFile, accDFile, fname, pkg):
        super(AddAccessoryDialog, self).__init__()
        self.parent = parent
        self.accName = accName
        self.accFile = accFile
        self.accDFile = accDFile
        self.accData = None
        self.fname = fname
        self.pkg = pkg

        # if description file was provided
        if self.accDFile:
            with open(self.accDFile, 'r') as f:
                data = yaml.load(f)

            try:
                self.accData = data[self.accName]
            except KeyError:
                pass

        if self.accData:
            self.setupPrefilledDialog()
        else:
            self.setupFillParamsDialog()

        self.setWindowTitle("Add Accessory")
        self.setModal(True)
        self.show()

    def setupPrefilledDialog(self):
        line = QtGui.QLabel("<hr />")
        label_name = QtGui.QLabel(bold("Accessory"))
        label_description = QtGui.QLabel(bold("Description"))
        label_parent = QtGui.QLabel(bold("Parent"))
        label_position = QtGui.QLabel(bold("Position"))
        label_accName = QtGui.QLabel(setMargins(self.accName, 0, 10, 0, 10))
        label_accDescription = QtGui.QLabel(setMargins(self.accData['description'],
                                                       0, 10, 0, 10))

        label_accDescription.setWordWrap(True)
        label_accDescription.setTextInteractionFlags(QtCore.Qt.TextSelectableByKeyboard|
                                                     QtCore.Qt.TextSelectableByMouse)
        label_accDescription.setSizePolicy(QtGui.QSizePolicy.Minimum,
                                           QtGui.QSizePolicy.Minimum)

        src = resolvePkgPaths(self.accData['image'])
        img = QtGui.QImage(src)
        pixmap = QtGui.QPixmap.fromImage(img)

        img_acc = QtGui.QLabel()
        img_acc.setPixmap(pixmap)
        img_acc.setPalette(self.parent.palette)
        img_acc.setFrameShape(QtGui.QFrame.StyledPanel)
        img_acc.setAutoFillBackground(True)
        img_acc.setMinimumWidth(150)
        img_acc.setAlignment(QtCore.Qt.AlignCenter)

        self.btn_add = QtGui.QPushButton("&Add")
        btn_cancel = QtGui.QPushButton("&Cancel")

        self.btn_add.setEnabled(False)
        self.btn_add.clicked.connect(self.addCB)
        btn_cancel.clicked.connect(self.cancel)

        self.cb_parent = QtGui.QComboBox()
        self.cb_position = QtGui.QComboBox()

        sizePolicy = self.cb_parent.sizePolicy()
        sizePolicy.setHorizontalStretch(1)
        self.cb_parent.setSizePolicy(sizePolicy)
        self.cb_position.setSizePolicy(sizePolicy)

        self.cb_parent.setMinimumWidth(200)
        self.cb_position.setMinimumWidth(200)
        self.cb_position.setEnabled(False)

        self.cb_parent.addItem("")
        parents = self.accData['parent'].keys()
        parents.sort()
        self.cb_parent.addItems(parents)

        self.cb_parent.currentIndexChanged[str].connect(self.updatePosition)
        self.cb_position.currentIndexChanged[str].connect(self.setButtonStateCB)

        box_position = QtGui.QHBoxLayout()
        box_position.addWidget(label_position)
        box_position.addWidget(self.cb_position)

        box_buttons = QtGui.QHBoxLayout()
        box_buttons.addStretch(1)
        box_buttons.addWidget(self.btn_add)
        box_buttons.addWidget(btn_cancel)
        box_buttons.addStretch(1)

        box_accDescription = QtGui.QVBoxLayout()
        box_accDescription.addWidget(label_name)
        box_accDescription.addWidget(label_accName)
        box_accDescription.addWidget(label_description)
        box_accDescription.addWidget(label_accDescription)
        box_accDescription.addStretch(1)
        box_accDescription.addWidget(line)

        grid = QtGui.QGridLayout()
        grid.addLayout(box_accDescription, 0, 0, 1, 2)
        grid.addWidget(img_acc, 0, 2, 3, 1)
        grid.addWidget(label_parent, 1, 0)
        grid.addWidget(self.cb_parent, 1, 1)
        grid.addWidget(label_position, 2, 0)
        grid.addWidget(self.cb_position, 2, 1)
        grid.addLayout(box_buttons, 3, 0, 1, 3)

        self.setLayout(grid)

    def setupFillParamsDialog(self):
        label_name = QtGui.QLabel(bold("Accessory"))
        label_accName = QtGui.QLabel(self.accName)

        self.btn_add = QtGui.QPushButton("&Add")
        btn_cancel = QtGui.QPushButton("&Cancel")

        self.btn_add.setEnabled(False)
        self.btn_add.clicked.connect(self.addLE)
        btn_cancel.clicked.connect(self.cancel)

        with open(self.accFile, 'r') as f:
            data = parse(f)

        macros = data.getElementsByTagName("xacro:macro")
        macros.extend(data.getElementsByTagName("macro"))

        for macro in macros:
            if macro.getAttribute("name") == self.accName:
                break

        self.params = macro.getAttribute("params").split()
        labels = map(lambda x: QtGui.QLabel(bold(x)), self.params)
        self.line_edits = map(lambda x: QtGui.QLineEdit(), self.params)
        map(lambda x: x.setMinimumWidth(200), self.line_edits)
        map(lambda x: x.textChanged.connect(self.setButtonStateLE), self.line_edits)

        box_name = QtGui.QHBoxLayout()
        box_name.addWidget(label_name)
        box_name.addWidget(label_accName)
        box_name.addStretch(1)

        box_button = QtGui.QHBoxLayout()
        box_button.addStretch(1)
        box_button.addWidget(self.btn_add)
        box_button.addWidget(btn_cancel)
        box_button.addStretch(1)

        grid_params = QtGui.QGridLayout()

        if self.params:
            for i in xrange(len(self.params)):
                grid_params.addWidget(labels[i], i, 0)
                grid_params.addWidget(self.line_edits[i], i, 1)
        else:
            label_noParams = QtGui.QLabel("No parameters.")
            grid_params.addWidget(label_noParams, 0, 0)
            self.btn_add.setEnabled(True)

        style = QtGui.QStyleFactory.create("Cleanlooks")

        grp_params = QtGui.QGroupBox("Parameters")
        grp_params.setStyle(style)
        grp_params.setMinimumWidth(200)
        grp_params.setLayout(grid_params)

        container = QtGui.QVBoxLayout()
        container.addLayout(box_name)
        container.addWidget(grp_params)
        container.addLayout(box_button)

        self.setLayout(container)

    def updatePosition(self, parent):
        if parent:
            self.positions = self.accData['parent'][parent]
            positions = self.positions.keys()
            positions.sort()

            self.cb_position.clear()
            self.cb_position.addItem("")
            self.cb_position.addItems(positions)
            self.cb_position.setEnabled(True)
        else:
            self.cb_position.clear()
            self.cb_position.setEnabled(False)

    def setButtonStateCB(self, position):
        if position:
            self.btn_add.setEnabled(True)
        else:
            self.btn_add.setEnabled(False)

    def setButtonStateLE(self):
        for le in self.line_edits:
            if not le.text():
                self.btn_add.setEnabled(False)
                return

        self.btn_add.setEnabled(True)

    def addCB(self):
        position = self.positions[self.cb_position.currentText()]
        params = position['params']
        required = position['requires']
        result = {"part_name": self.accName, "params": params}
        accAdded = list()

        added = self.parent.addAccToRobot(result, self.pkg, self.fname)

        if added:
            accInfo = result['part_name'] + " (" + self.pkg + ":" + self.fname + ")"

            for k, v in result['params'].items():
                accInfo += ("\n    " + k + ": " + v)

            accAdded.append(accInfo)

        for fpath in required:
            for acc in required[fpath]:
                key = acc.keys()[0]
                result = {"part_name": key, "params": acc[key]}

                pkg, fname = re.sub("\$\(find|\)", " ", fpath).split()
                fname = re.sub("/[\w.-/]+/", "", fname)
                added = self.parent.addAccToRobot(result, pkg, fname, fpath)

                if added:
                    accInfo = result['part_name'] + " (" + pkg + ":" + fname + ")"

                    for k, v in result['params'].items():
                        accInfo += ("\n    " + k + ": " + v)

                    accAdded.append(accInfo)

        if accAdded:
            self.dialog = AddedAccessoryDialog(accAdded)
        else:
            self.dialog = AddErrorDialog()

    def addLE(self):
        params = dict()

        for i in xrange(len(self.params)):
            params[self.params[i]] = self.line_edits[i].text()

        result = {"part_name": self.accName, "params": params}
        roslib.packages.get_dir_pkg(self.accFile)

        added = self.parent.addAccToRobot(result, self.pkg, self.fname)

        if not added:
            self.dialog = AddErrorDialog()

    def cancel(self):
        self.close()

class AddedAccessoryDialog(QtGui.QMessageBox):
    def __init__(self, accessories):
        super(AddedAccessoryDialog, self).__init__()

        accNames = map(lambda x: x.split()[0], accessories)
        msg = "Added %d accessories:\t\n" % len(accessories)
        msg += ("\n    " + "\n    ".join(accNames) + '\n')
        details = "\n\n".join(accessories)

        self.setText(msg)
        self.setDetailedText(details)
        self.setIcon(self.Information)
        self.show()

class AddErrorDialog(QtGui.QMessageBox):
    def __init__(self):
        super(AddErrorDialog, self).__init__()

        msg = "This accessory has already been attached."

        self.setIcon(self.Critical)
        self.setText(msg)
        self.setWindowTitle("Error")
        self.show()
