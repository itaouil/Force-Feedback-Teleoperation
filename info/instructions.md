# Instructions 

## Install & Run ur5_dummy

Steps to install ur5_dummy project:

* `git clone git@git.ais.uni-bonn.de:cogrob2020/ur5_dummy.git`
* `sudo apt install python-wstool`
* `wstool init`
* `wstool merge ur5_dummy/rosinstall`
* `wstool update`
* `catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS=-Wall`
* `sudo rosdep init`
* `rosdep update`
* `rosdep install --ignore-src -y --from-path src`
* `catkin build`
* `source devel/setup.sh`

Steps to run ur10e_dummy alone:

* `roscore` 
* `mon launch ur10e_dummy ur10e_dummy.launch`
* `roslaunch ur10e_dummy keyframe_editor.launch`

in case you want to launch both robot in the same space run instead:

* `roscore`
* `mon launch panda_ur10e_dummy panda_ur10e_dummy.launch`
* `roslaunch panda_ur10e_dummy keyframe_editor.launch`

## Run the project

Launch the package executables, mainly **franka_marker**, **franka_controller**, and **ur10e_controller** by launching the roslaunch file made available as follows:

roslaunch force_feedback_controller project.launch

This will make visible the **franka control marker** on the RVIz window that previously opened, as well as the collision shape.

### Franka Interactive Marker

If for any reason you want to change the settings of the franka controller marker, on rviz change the setting for the **franka_interactive_marker** object among the list.

In general, the default settings are the following:

* Fixed frame to '/base_link'.
* Update topic set to '/basic_controls/update'.
* Select "Interact" to move the marker around and see the effect.

### ur10e simulator

Similarly to above, the collision shapes are shown through the **shape_collision_marker** in rviz, and the default settings are:

* 'Update Topic' set to '/imarker-[name-of-the-geometry]/update'

The remaning objects at the end of the list are:

* 'Marker' whose 'Marker Topic' is set to '/interactive_robot_markers'
* 'MarkerArray' whose Marker Topic is set to '/interactive_robot_marray'

in order to visualize the contact points, as well as the relative contact points data (normal, pose, etc).

**Enjoy**, but do know it is not bug free :).
