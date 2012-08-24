#! /usr/bin/env python

import os
import sys
import subprocess
import re
import yaml
from xml.dom.minidom import parse, Document
from xml.parsers.expat import ExpatError
from PySide import QtGui, QtCore
from guiFilePopups import *
from guiAccessoryPopups import *
from aux import *

class MainGUI(QtGui.QWidget):
    def __init__(self):
        super(MainGUI, self).__init__()
        self.setWindowTitle("URDF Generator")

        self.attachedAccessories = list()
        self.reqFiles = list()

        self.initPath()
        self.initButtons()
        self.initComboBoxes()
        self.initImages()
        self.initLists()
        self.initLabels()
        self.setupLayout()
        self.centerFrame()
        self.show()

        title = "Launching rviz"
        msg = self.tr("To reactivate the GUI window after\n"
                      "launching rviz, close the opened\n"
                      "rviz window and press Ctrl+C in the\n"
                      "terminal where the GUI is running.")
        self.msgDialog = QtGui.QMessageBox(QtGui.QMessageBox.Information, title, msg)
        self.msgDialog.setModal(False)
        self.msgDialog.show()

    def initPath(self):
        """
        Setups initial path for QFileDialogs, i.e., first directory in
        ROS_PACKAGE_PATH. If ROS_PACKAGE_PATH does not exist, the default
        is the home directory. Gets path to package "urdf_compose".
        """
        try:
            self.pkg_path = os.environ["ROS_PACKAGE_PATH"].split(':')
            self.path = self.pkg_path[0]
            self.urdf_compose = roslib.packages.get_pkg_dir("urdf_compose")
        except KeyError:
            self.path = "/home/"
            print "Please setup ROS_PACKAGE_PATH:"
            print
            print "    export ROS_PACKAGE_PATH=<directory>"
            print
        except roslib.packages.InvalidROSPkgException:
            print "Could not get path to package \"urdf_compose\"."
            sys.exit(1)

    def initButtons(self):
        self.btn_addRobot = QtGui.QPushButton("Add &new robot")
        self.btn_rmRobot = QtGui.QPushButton("Remove r&obot")
        self.btn_addFile = QtGui.QPushButton("A&dd file")
        self.btn_rmFile = QtGui.QPushButton("Re&move file")
        self.btn_addAcc = QtGui.QPushButton("&Add accessory to robot")
        self.btn_rmAcc = QtGui.QPushButton("&Remove")
        self.btn_urdf = QtGui.QPushButton("&Create URDF file")
        self.btn_rviz = QtGui.QPushButton("&Launch rviz")

        self.btn_rmRobot.setEnabled(False)
        self.btn_rmFile.setEnabled(False)
        self.btn_addAcc.setEnabled(False)
        self.btn_rmAcc.setEnabled(False)
        self.btn_urdf.setEnabled(False)
        self.btn_rviz.setEnabled(False)

        self.btn_addRobot.clicked.connect(self.addRobot)
        self.btn_rmRobot.clicked.connect(self.rmRobot)
        self.btn_addFile.clicked.connect(self.addFile)
        self.btn_rmFile.clicked.connect(self.rmFile)
        self.btn_addAcc.clicked.connect(self.addAcc)
        self.btn_rmAcc.clicked.connect(self.rmAcc)
        self.btn_urdf.clicked.connect(self.create)
        self.btn_rviz.clicked.connect(self.launch)

    def initLabels(self):
        self.label_robot = QtGui.QLabel(bold("Select robot"))
        self.label_attAcc = QtGui.QLabel("Attached Accessories")
        self.label_files = QtGui.QLabel("Accessory Files")
        self.label_acc = QtGui.QLabel("Accessories")
        self.label_accName = QtGui.QLabel("Accessory")
        self.label_accDescript = QtGui.QLabel("Description")

        self.label_attAcc.setAlignment(QtCore.Qt.AlignHCenter)
        self.label_files.setAlignment(QtCore.Qt.AlignHCenter)
        self.label_acc.setAlignment(QtCore.Qt.AlignHCenter)

    def initImages(self):
        self.robotBG = QtGui.QColor(0, 1, 57, 255)

        brush = QtGui.QBrush(self.robotBG)
        brush.setStyle(QtCore.Qt.SolidPattern)

        self.palette = QtGui.QPalette()
        self.palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush);
        self.palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush);
        self.palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush);

        pixmap = QtGui.QPixmap(300, 500)
        pixmap.fill(self.robotBG)

        self.img_robot = QtGui.QLabel()
        self.img_robot.setPixmap(pixmap)
        self.img_robot.setPalette(self.palette)
        self.img_robot.setFrameShape(QtGui.QFrame.StyledPanel)
        self.img_robot.setAutoFillBackground(True)
        self.img_robot.setMinimumWidth(300)
        self.img_robot.setAlignment(QtCore.Qt.AlignCenter)

    def initComboBoxes(self):
        path = self.urdf_compose + "/src/gui/robots.yaml"

        try:
            self.robots = yaml.load(open(path, 'r'))
        except IOError, e:
            print "Could not open/missing file \"%s\"." % path
            sys.exit(1)
        except yaml.YAMLError, e:
            print e
            sys.exit(1)

        self.cb_robots = QtGui.QComboBox()

        sizePolicy = self.cb_robots.sizePolicy()
        sizePolicy.setHorizontalStretch(1)
        self.cb_robots.setSizePolicy(sizePolicy)

        keys = self.robots.keys()
        keys.sort()

        # populate combo box
        self.cb_robots.addItem("")
        self.cb_robots.addItems(keys)

        self.cb_robots.currentIndexChanged[str].connect(self.updateRobot)

    def initLists(self):
        path = self.urdf_compose + "/src/gui/files.yaml"

        try:
            self.files = yaml.load(open(path, 'r'))
        except IOError, e:
            print "Could not open/missing file \"%s\"." % path
            sys.exit(1)
        except yaml.YAMLError, e:
            print e
            sys.exit(1)

        self.list_attAcc = QtGui.QListWidget()
        self.list_files = QtGui.QListWidget()
        self.list_addAcc = QtGui.QListWidget()

        self.list_attAcc.setMinimumWidth(200)
        self.list_files.setMinimumWidth(200)
        self.list_addAcc.setMinimumWidth(200)
        self.list_attAcc.setSortingEnabled(True)
        self.list_files.setSortingEnabled(True)
        self.list_addAcc.setSortingEnabled(True)

        self.list_files.addItems(self.files.keys())
        self.list_addAcc.addItem("No accessory file selected.")

        if self.list_files.count() == 0:
            self.list_files.addItem("No accessory files.")
            self.list_files.setEnabled(False)

        self.list_addAcc.setEnabled(False)

        self.list_files.itemSelectionChanged.connect(self.updateAcc)
        self.list_addAcc.itemSelectionChanged.connect(self.selectAcc)
        self.list_attAcc.itemSelectionChanged.connect(self.selectAttAcc)
        self.list_attAcc.itemClicked.connect(self.selectAttAcc)

    def setupLayout(self):
        style = QtGui.QStyleFactory.create("Cleanlooks")

        box_selRobot = QtGui.QHBoxLayout()
        box_selRobot.addWidget(self.label_robot)
        box_selRobot.addWidget(self.cb_robots)
        box_selRobot.addWidget(self.btn_addRobot)
        box_selRobot.addWidget(self.btn_rmRobot)

        grid_robot = QtGui.QGridLayout()
        grid_robot.addWidget(self.img_robot, 0, 0, 2, 2)
        grid_robot.addWidget(self.label_attAcc, 0, 2, 1, 2)
        grid_robot.addWidget(self.list_attAcc, 1, 2, 1, 2)
        grid_robot.addWidget(self.btn_rviz, 2, 0)
        grid_robot.addWidget(self.btn_urdf, 2, 1)
        grid_robot.addWidget(self.btn_rmAcc, 2, 2, 1, 2)

        grp_robot = QtGui.QGroupBox("Robot")
        grp_robot.setStyle(style)
        grp_robot.setLayout(grid_robot)

        grid_addAcc = QtGui.QGridLayout()
        grid_addAcc.addWidget(self.label_files, 0, 0, 1, 2)
        grid_addAcc.addWidget(self.label_acc, 0, 2, 1, 2)
        grid_addAcc.addWidget(self.list_files, 1, 0, 1, 2)
        grid_addAcc.addWidget(self.list_addAcc, 1, 2, 1, 2)
        grid_addAcc.addWidget(self.btn_addFile, 2, 0)
        grid_addAcc.addWidget(self.btn_rmFile, 2, 1)
        grid_addAcc.addWidget(self.btn_addAcc, 2, 2, 1, 2)

        grp_addAcc = QtGui.QGroupBox("Add Accessories")
        grp_addAcc.setStyle(style)
        grp_addAcc.setLayout(grid_addAcc)

        box_main = QtGui.QHBoxLayout()
        box_main.addWidget(grp_robot)
        box_main.addWidget(grp_addAcc)

        layout = QtGui.QVBoxLayout()
        layout.addLayout(box_selRobot)
        layout.addLayout(box_main)
        self.setLayout(layout)

    def centerFrame(self):
        desktop = QtGui.QDesktopWidget().availableGeometry()
        frame = self.frameGeometry()
        center = (desktop.center()-frame.center())/2
        self.move(center)

    def addRobot(self):
        self.dialog = AddRobotDialog(self)

    def addRobotToComboBox(self, robotName, robotInfo):
        # if robot exists already
        if self.cb_robots.findText(robotName, QtCore.Qt.MatchExactly) == 1:
            msg = self.tr("Robot \"%s\" already exists.\n\n"
                          "Replace the existing \"%s\"?") % (robotName, robotName)
            reply = QtGui.QMessageBox.question(self, "Add Robot", msg,
                                               QtGui.QMessageBox.Yes,
                                               QtGui.QMessageBox.No)

            if reply == QtGui.QMessageBox.Yes:
                self.robots[robotName] = robotInfo
                self.dialog.close()

                if robotName == self.cb_robots.currentText():
                    self.updateRobot(self.cb_robots.currentText())

            return

        self.robots[robotName] = robotInfo
        self.cb_robots.addItem(robotName)
        self.dialog.close()

    def rmRobot(self):
        index = self.cb_robots.currentIndex()

        if index != 0:
            del self.robots[self.cb_robots.currentText()]
            self.cb_robots.removeItem(index)

    def updateRobot(self, robot):
        if robot:
            src = self.robots[robot]['image']
            src = resolvePkgPaths(src)

            if src:
                # display robot image
                img = QtGui.QImage(src)
            else:
                # no image
                path = self.urdf_compose + "/images/no_image.png"
                img = QtGui.QImage(path)

            img = img.scaledToWidth(300)
            pixmap = QtGui.QPixmap.fromImage(img)
            self.img_robot.setPixmap(pixmap)
            self.btn_rmRobot.setEnabled(True)
            self.btn_urdf.setEnabled(True)
            self.btn_rviz.setEnabled(True)

            if self.list_addAcc.currentItem():
                self.btn_addAcc.setEnabled(True)
        else:
            # solid color
            pixmap = QtGui.QPixmap(300, 500)
            pixmap.fill(self.robotBG)
            self.img_robot.setPixmap(pixmap)

            self.reqFiles = list()
            self.attachedAccessories = list()
            self.list_attAcc.clear()
            self.btn_rmRobot.setEnabled(False)
            self.btn_addAcc.setEnabled(False)
            self.btn_rmAcc.setEnabled(False)
            self.btn_urdf.setEnabled(False)
            self.btn_rviz.setEnabled(False)

    def addFile(self):
        self.dialog = AddAccessoryFileDialog(self)

    def addFileToList(self, fname, fileInfo):
        if fname in self.files:
            msg = self.tr("File name conflict.\n\n"
                          "Accessory file \"%s\"\n"
                          "already exists.") % fname.split()[0]
            self.msgDialog = QtGui.QMessageBox(QtGui.QMessageBox.Critical, "Error", msg)
            self.msgDialog.show()

            return

        self.files[fname] = fileInfo
        self.list_files.addItem(fname)
        self.dialog.close()

    def rmFile(self):
        selFile = self.list_files.currentRow()
        del self.files[self.list_files.currentItem().text()]
        self.list_files.takeItem(selFile)

        self.list_addAcc.clear()
        self.updateAcc()

    def addAcc(self):
        selFile = self.list_files.currentItem().text()
        fname, pkg = re.sub(r"\(|\)", "", selFile).split()
        accName = self.list_addAcc.currentItem().text()
        accFile = resolvePkgPaths(self.files[selFile]['file'])
        accDFile = resolvePkgPaths(self.files[selFile]['description'])

        self.dialog = AddAccessoryDialog(self, accName, accFile, accDFile, fname, pkg)

    def addAccToRobot(self, result, pkg, fname, fpath=""):
        if fpath and fpath not in self.reqFiles:
            self.reqFiles.append(fpath)

        accessory = [pkg, fname, result]

        if accessory in self.attachedAccessories:
            return False
        else:
            self.attachedAccessories.append(accessory)

        self.dialog.close()

        accInfo = result['part_name'] + " (" + pkg + ":" + fname + ")"

        for k, v in result['params'].items():
            accInfo += ("\n    " + k + ": " + v)

        self.list_attAcc.addItem(accInfo)

        return True

    def selectAcc(self):
        if self.cb_robots.currentIndex() != 0:
            self.btn_addAcc.setEnabled(True)

    def selectAttAcc(self):
        self.btn_rmAcc.setEnabled(True)

    def updateAcc(self):
        selFile = self.list_files.currentItem()

        if selFile:
            path = self.files[selFile.text()]['file']
            path = resolvePkgPaths(path)

            with open(path, 'r') as f:
                data = parse(f)

            macros = data.getElementsByTagName("xacro:macro")
            macros.extend(data.getElementsByTagName("macro"))
            macroNames = map(lambda x: x.getAttribute("name"), macros)

            self.list_addAcc.clear()
            self.list_addAcc.addItems(macroNames)

            self.btn_rmFile.setEnabled(True)
            self.btn_addAcc.setEnabled(False)
            self.list_addAcc.setEnabled(True)
        else:
            self.btn_rmFile.setEnabled(False)
            self.btn_addAcc.setEnabled(False)
            self.list_files.setEnabled(False)
            self.list_addAcc.setEnabled(False)

    def rmAcc(self):
        selAcc = self.list_attAcc.currentRow()
        accInfo = self.list_attAcc.currentItem().text().replace(' ', "")
        accInfo = accInfo.split('\n')
        params = dict()

        for param in accInfo[1:]:
            k, v = param.split(':')
            params[k] = v

        accName, pkg, fname = re.sub(r"[():]", ' ', accInfo[0]).split()
        acc = [pkg, fname, {"part_name": accName, "params": params}]

        self.attachedAccessories.remove(acc)
        self.list_attAcc.takeItem(selAcc)

        if not self.list_attAcc.count():
            self.btn_rmAcc.setEnabled(False)

    def create(self):
        title = "Save file as"
        filter = "URDF files (*.urdf *.xacro *.urdf.xacro)"
        path, _ = QtGui.QFileDialog.getSaveFileName(self, title, self.path, filter)

        if not path:
            return

        self.path = re.findall(r"/[\w.\-/]+/", path)[0]

        if not re.search(r"\.urdf|\.xacro", path):
            path += ".urdf.xacro"

        self.createFile(path)

        title = "URDF"
        msg = "File created at " + path
        self.msgDialog = QtGui.QMessageBox(QtGui.QMessageBox.Information, title, msg)
        self.msgDialog.show()

    def launch(self):
        path = self.urdf_compose + "/urdf/temp.urdf.xacro"
        self.createFile(path)

        cmd = ["roslaunch", "urdf_compose", "demo.launch", "model:=temp.urdf.xacro"]
        rviz = subprocess.call(cmd)

    def createFile(self, path):
        robotFile = resolvePkgPaths(self.robots[self.cb_robots.currentText()]['file'])
        files = list()

	self.attachedAccessories.sort()

        for acc in self.attachedAccessories:
            if not (acc[0], acc[1]) in files:   # pkg, fname
                files.append((acc[0], acc[1]))

        with open(robotFile, 'r') as f:
            robotData = parse(f)

        doc = Document()
        robot = robotData.firstChild
        robot_elements = robot.childNodes

        for node in robot_elements:
            if node.nodeType == node.TEXT_NODE:
                robot.removeChild(node)

        for f in files:
            key = f[1] + " (" + f[0] + ")"

            try:
                include = doc.createElement("include")
                include.setAttribute("filename", self.files[key]['file'])
                robot.appendChild(include)
            except KeyError:
                pass

        for f in self.reqFiles:
            include = doc.createElement("include")
            include.setAttribute("filename", f)
            robot.appendChild(include)

        for acc in self.attachedAccessories:
            accInfo = acc[2]
            accessory = doc.createElement(accInfo['part_name'])

            for k, v in accInfo['params'].items():
                accessory.setAttribute(k, v)

            robot.appendChild(accessory)

        doc.appendChild(robot)

        with open(path, 'w') as f:
            f.write(doc.toprettyxml("  "))

def main():
    app = QtGui.QApplication(sys.argv)
    gui = MainGUI()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
