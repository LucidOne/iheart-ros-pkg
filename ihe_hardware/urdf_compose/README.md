Run GUI
-------

1. Build the turtlebot_body.

       roscd urdf_compose/urdf
       rosrun urdf_compose compose.py ../demo/turtlebot_body.yaml -o turtlebot_body.urdf.xacro

2. Run the GUI.

       rosrun urdf_compose gui.py

Only macros from added files will be displayed in the "Accessories" list.

You can view your robot with the attached accessories by clicking on the
"Launch rviz" button. To reactivate the GUI window after launching rviz, close
the opened rviz window and press Ctrl+C in the terminal where the GUI is
running.


Adding Your URDFs to the Compositor Automatically
-------------------------------------------------

In your manifest.xml, add the following export statement:

    <export>
        <urdf base="${prefix}/PATH/TO/FILE"/>
        <urdf plugin="${prefix}/PATH/TO/FILE"/>
    </export>

The base attribute is used for your robot base, i.e., the file is a complete
URDF (verify by running the check_urdf script in the urdf package). The
plugin attribute is used for accessories that are to be added onto the base,
i.e., the file contains macros.

Format for Descripiton Files (uraf)
-----------------------------------

### Bases

    -
      file: <path to urdf file>
      image: <path to image file>

### Plugins (Accessories)

    urdf: <path to urdf file>   # this line only appears once per file
    <accessory name/macro name>:
      image: <path to image>
      description: <description>
      parent:
        <link name>:
          <position name>:
            params:         # parameters for the accessory being described
              <parameter name>: <value>
                requires:
                  <path to file that contains the required accessory>
                    - <accessory name/macro name>
                        <parameter name>: <value>   # parameters for required piece

Examples are located in $(find urdf_compose)/urdf/kinect.uraf and
$(find turtlebot_xtion_top_description)/urdf/xtion.uraf


Format for File to be Processed by compose.py (yaml)
----------------------------------------------------

    - package: <package name>
      files:
        - filename: <filename>
          parts:
            - part_name: <macro name>
              params:
                <parameter name>: <parameter value>


Generate urdf File (compose.py)
-------------------------------

To generate an XML file, run:

    rosrun urdf_compose compose.py <yaml> [-o <outfile>] [-n <robot name>]

If -o is not specificed, the default is standard output. If -n is not
specified, the default is "turtlebot".

This script is to be used only if all parts listed are macros.


Run Demo (compose.py)
---------------------

1. Open the urdf directory.

       roscd urdf_compose/urdf

2. Generate urdf files. (yaml files are located in the demo directory.)

       rosrun urdf_compose compose.py ../demo/kinect.yaml -o kinect.urdf.xacro -n kinect
       rosrun urdf_compose compose.py ../demo/xtion.yaml -o xtion.urdf.xacro -n xtion
       rosrun urdf_compose compose.py ../demo/turtlebot_body.yaml -o turtlebot_body.urdf.xacro
       rosrun urdf_compose compose.py ../demo/turtlebot.yaml -o turtlebot.urdf.xacro

3. Launch rviz.

       roslaunch urdf_compose demo.launch model:=<model>

   <model> = { kinect.urdf.xacro,
               xtion.urdf.xacro,
               turtlebot.urdf.xacro,
               turtlebot_body.urdf.xacro }

Note: The turtlebot_body included in the package does not include the kinect
      standoffs like the body included in turtlebot_description.
