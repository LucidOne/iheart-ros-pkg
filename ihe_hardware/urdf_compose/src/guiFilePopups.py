import re
import yaml
from xml.dom.minidom import parse
from PySide import QtGui, QtCore
from aux import *

class GetTwoFilesDialog(QtGui.QDialog):
    """
    Creates the following dialog:

        +-------------------------------------+
        | <f1> *  {     le_f1     }  [Browse] |     [] = button
        | <f2>    {     le_f2     }  [Browse] |     {} = line edit
        | * Required           [Add] [Cancel] |
        +-------------------------------------+

    <f1> and <f2> are text for labels. File 1 is required to enable the add
    button.
    """
    def __init__(self, parent=None, f1="File", f2="File"):
        super(GetTwoFilesDialog, self).__init__()
        self.parent = parent

        label_f1 = QtGui.QLabel(bold(f1)+" *")
        label_f2 = QtGui.QLabel(bold(f2))
        label_req = QtGui.QLabel("* Required")

        self.le_f1 = QtGui.QLineEdit()
        self.le_f2 = QtGui.QLineEdit()

        self.le_f1.setReadOnly(True)
        self.le_f2.setReadOnly(True)
        self.le_f1.setEnabled(False)
        self.le_f2.setEnabled(False)
        self.le_f1.setMinimumWidth(200)
        self.le_f2.setMinimumWidth(200)

        btn_f1 = QtGui.QPushButton("Browse")
        btn_f2 = QtGui.QPushButton("Browse")
        self.btn_add = QtGui.QPushButton("&Add")
        btn_cancel = QtGui.QPushButton("&Cancel")

        self.btn_add.setEnabled(False)

        btn_f1.clicked.connect(self.get_f1)
        btn_f2.clicked.connect(self.get_f2)
        self.btn_add.clicked.connect(self.add)
        btn_cancel.clicked.connect(self.cancel)

        buttons = QtGui.QHBoxLayout()
        buttons.addWidget(label_req)
        buttons.addStretch(1)
        buttons.addWidget(self.btn_add)
        buttons.addWidget(btn_cancel)

        grid = QtGui.QGridLayout()
        grid.addWidget(label_f1, 0, 0)
        grid.addWidget(self.le_f1, 0, 1)
        grid.addWidget(btn_f1, 0, 2)
        grid.addWidget(label_f2, 1, 0)
        grid.addWidget(self.le_f2, 1, 1)
        grid.addWidget(btn_f2, 1, 2)
        grid.addLayout(buttons, 2, 0, 1, 3)

        self.setLayout(grid)
        self.setModal(True)

    def getFile(self, title, directory, filter=""):
        path, _ = QtGui.QFileDialog.getOpenFileName(self, title, directory, filter)

        return path

    def get_f1(self):
        path = self.getFile("Select file", "/home/")

        if not path:
            return

        self.le_f1.setEnabled(True)
        self.le_f1.setText(path)
        self.btn_add.setEnabled(True)

    def get_f2(self):
        path = self.getFile("Select file", "/home/")

        if not path:
            return

        self.le_f2.setEnabled(True)
        self.le_f2.setText(path)

    def add(self):
        pass

    def cancel(self):
        self.close()

class AddRobotDialog(GetTwoFilesDialog):
    def __init__(self, parent):
        super(AddRobotDialog, self).__init__(parent, "Robot", "Image")
        self.setWindowTitle("Add Robot")
        self.show()

    def get_f1(self):
        title = "Select robot"
        filter = "URDF files (*.urdf *.xacro *.urdf.xacro)"
        fpath = self.getFile(title, self.parent.path, filter)

        if not fpath:
            return

        self.parent.path = re.findall(r"/[\w.\-/]+/", fpath)[0]
        self.le_f1.setEnabled(True)
        self.le_f1.setText(fpath)
        self.btn_add.setEnabled(True)

    def get_f2(self):
        title = "Select robot image"
        filter = "Image files (*.bmp *.png *.jpg)"
        image = self.getFile(title, self.parent.path, filter)

        if not image:
            return

        self.parent.path = re.findall(r"/[\w.\-/]+/", image)[0]
        self.le_f2.setEnabled(True)
        self.le_f2.setText(image)

    def add(self):
        file_path = getResolveString(self.le_f1.text())
        image_path = getResolveString(self.le_f2.text())
        result = {"file": file_path, "image": image_path}

        with open(self.le_f1.text(), 'r') as f:
            urdf = parse(f)
            robotTag = urdf.getElementsByTagName("robot")[0]
            robotName = robotTag.getAttribute("name")

        self.parent.addRobotToComboBox(robotName, result)

class AddAccessoryFileDialog(GetTwoFilesDialog):
    def __init__(self, parent):
        f1 = "Accessories"
        f2 = "Descriptions"

        super(AddAccessoryFileDialog, self).__init__(parent, f1, f2)
        self.setWindowTitle("Add Accessory File")
        self.show()

    def get_f1(self):
        title = "Select accessory file"
        filter = "URDF files (*.urdf *.xacro *.urdf.xacro)"
        fpath = self.getFile(title, self.parent.path, filter)

        if not fpath:
            return

        self.parent.path = re.findall(r"/[\w.\-/]+/", fpath)[0]
        self.le_f1.setEnabled(True)
        self.le_f1.setText(fpath)
        self.btn_add.setEnabled(True)

    def get_f2(self):
        title = "Select accessory descriptions"
        filter = "Description files (*.yaml)"
        description = self.getFile(title, self.parent.path, filter)

        if not description:
            return

        self.parent.path = re.findall(r"/[\w.\-/]+/", description)[0]
        self.le_f2.setEnabled(True)
        self.le_f2.setText(description)

    def add(self):
        file_path = getResolveString(self.le_f1.text())
        description_path = getResolveString(self.le_f2.text())
        result = {'file': file_path, 'description': description_path}

        directory = re.findall(r"/[\w.\-/]+/", self.le_f1.text())[0]
        fname = self.le_f1.text().replace(directory, "")
        _, pkg = roslib.packages.get_dir_pkg(self.le_f1.text())
        fname = fname + " (" + pkg + ")"

        self.parent.addFileToList(fname, result)
