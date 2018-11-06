# image_transport_python

Trying to implement using image_transport from Python.

## On dealing with ROS messages

Exotica: https://github.com/ipab-slmc/exotica/blob/master/exotica_python/src/Exotica.cpp

Has generic code to transform **ANY** C++ ROS message into a Python message C++ -> Python.

pyrosmsg: https://github.com/dimatura/pyrosmsg

Has code to transform Images in both directions C++ <-> Python, but seems hardcoded.


## On dealing with ROS C++ node inside of Python node

I don't know how to deal with this.

If I find out how, I should make a nice tutorial as it's very useful.
