#! /usr/bin/env python

# Generates an urdf file for the union of hardware pieces as described by 
# the input yaml file.

import sys
import getopt
import yaml
from xml.dom.minidom import Document

# Attachs <include> elements to the parent.
def appendInclude(parent, yaml_urdf, document=Document()):
    for pkg in yaml_urdf:
        for file in pkg['files']:
            filename = "$(find %s)/urdf/%s" % (pkg['package'], file['filename'])
            include = document.createElement("include")
            include.setAttribute("filename", filename)
            parent.appendChild(include)

# Attachs hardware elements (i.e., macros) to the parent.
def appendHardware(parent, yaml_urdf, document=Document()):
    for pkg in yaml_urdf:
        for file in pkg['files']:
            try:
                for hardware in file['parts']:      # macro
                    part = document.createElement(hardware['part_name'])
                    try:
                        # get parameters for macro
                        for k, v in hardware['params'].items():
                            part.setAttribute(k, v)
                    except:
                        # no parameters are defined
                        pass
                    parent.appendChild(part)
            except:
                # include file only
                pass

def usage(exit_code=0):
    print "Usage: compose.py <infile> [-o <outfile>] [-n <robot name>]"
    print
    print "  defaults: <outfile> = stdout"
    print "            <robot name> = turtlebot"
    print
    sys.exit(exit_code)

def main():
    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], "n:o:h")
    except getopt.GetoptError, err:
        print str(err)
        usage(2)

    robotName = "turtlebot"
    ofstream = sys.stdout

    for opt, arg in opts:
        if opt == '-h':
            usage()
        elif opt == '-o':
            ofstream = open(arg, 'w')
        elif opt == '-n':
            robotName = arg

    if len(args) < 1:
        print "No input provided."
        usage(2)

    try:
        urdf_struct = yaml.load(open(args[0], 'r'))
    except IOError, e:
        print "Could not open file \"%s\"" % args[0]
        sys.exit(1)
    except yaml.YAMLError, e:
        print e
        sys.exit(1)

    doc = Document()

    # Add <robot> element
    robot = doc.createElement("robot")
    robot.setAttribute("name", robotName)
    robot.setAttribute("xmlns:controller", "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller")
    robot.setAttribute("xmlns:interface", "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface")
    robot.setAttribute("xmlns:sensor", "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor")
    robot.setAttribute("xmlns:xacro", "http://ros.org/wiki/xacro")
    doc.appendChild(robot)

    # Add <include> elements
    appendInclude(robot, urdf_struct, doc)

    # Add hardware
    appendHardware(robot, urdf_struct, doc)

    ofstream.write(doc.toprettyxml(indent="  "))
    print
    ofstream.close()

if __name__ == "__main__":
    main()
