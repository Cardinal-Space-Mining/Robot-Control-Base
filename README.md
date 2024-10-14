# Robot-Control-Base

TODO:
* The `robot_control_base` package requires that all packages that depend on it also depend on `tf2_geometry_msgs`. A current work around is to just include `tf2_geometry_msgs` in the package.xml and as something in `find_package` inside a CMAKE file