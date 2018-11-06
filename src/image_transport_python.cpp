/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2018, 
*  Sammy Pfeiffer, 
*  partial code from
*  the Exotica project from SLMC Group at University of Edinburgh, UK
*  and from the ROS tutorial on image_transport.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "image_transport/image_transport.h"
#include "image_transport/publisher_plugin.h"
// Taken from Exotica
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#if PY_MAJOR_VERSION < 3
#define PY_OLDSTANDARD
#endif

#ifndef PY_OLDSTANDARD
bool PyInt_Check(PyObject* value_py) { return PyLong_Check(value_py); }
long PyInt_AsLong(PyObject* value_py) { return PyLong_AsLong(value_py); }
#endif

bool isPyString(PyObject* value_py)
{
#ifndef PY_OLDSTANDARD
    return PyUnicode_Check(value_py);
#else
    return PyString_Check(value_py) || PyUnicode_Check(value_py);
#endif
}

std::string pyAsString(PyObject* value_py)
{
#ifndef PY_OLDSTANDARD
    PyObject* tmp = PyUnicode_AsASCIIString(value_py);
    std::string ret = std::string(PyBytes_AsString(tmp));
    Py_DECREF(tmp);
    return ret;
#else
    return std::string(PyString_AsString(value_py));
#endif
}

PyObject* stringAsPy(std::string value)
{
#ifndef PY_OLDSTANDARD
    return PyUnicode_DecodeASCII(value.c_str(), value.size(), "");
#else
    return PyString_FromString(value.c_str());
#endif
}

namespace py = pybind11;

std::map<std::string, Initializer> knownInitializers;

PyObject* createStringIOObject()
{
#if PY_MAJOR_VERSION <= 2
    PyObject* module = PyImport_ImportModule("StringIO");
    if (!module) throw_pretty("Can't load StringIO module.");
    PyObject* cls = PyObject_GetAttrString(module, "StringIO");
    if (!cls) throw_pretty("Can't load StringIO class.");
#else
    PyObject* module = PyImport_ImportModule("io");
    if (!module) throw_pretty("Can't load io module.");
    PyObject* cls = PyObject_GetAttrString(module, "BytesIO");
    if (!cls) throw_pretty("Can't load BytesIO class.");
#endif
    PyObject* stringio = PyObject_CallObject(cls, NULL);
    if (!stringio) throw_pretty("Can't create StringIO object.");
    Py_DECREF(module);
    Py_DECREF(cls);
    return stringio;
}

// This is the bomb! But how do we do it the other way around?
#define ROS_MESSAGE_WRAPPER(MessageType)                                        \
    namespace pybind11                                                          \
    {                                                                           \
    namespace detail                                                            \
    {                                                                           \
    template <>                                                                 \
    struct type_caster<MessageType>                                             \
    {                                                                           \
    public:                                                                     \
        PYBIND11_TYPE_CASTER(MessageType, _("genpy.Message"));                  \
                                                                                \
        bool load(handle src, bool)                                             \
        {                                                                       \
            PyObject* stringio = createStringIOObject();                        \
            if (!stringio) throw_pretty("Can't create StringIO instance.");     \
            PyObject* result =                                                  \
                PyObject_CallMethod(src.ptr(), "serialize", "O", stringio);     \
            if (!result) throw_pretty("Can't serialize.");                      \
            result = PyObject_CallMethod(stringio, "getvalue", nullptr);        \
            if (!result) throw_pretty("Can't get buffer.");                     \
            char* data = PyByteArray_AsString(PyByteArray_FromObject(result));  \
            int len = PyByteArray_Size(result);                                 \
            unsigned char* udata = new unsigned char[len];                      \
            for (int i = 0; i < len; i++)                                       \
                udata[i] = static_cast<unsigned char>(data[i]);                 \
            ros::serialization::IStream stream(udata, len);                     \
            ros::serialization::deserialize<MessageType>(stream, value);        \
            delete[] udata;                                                     \
            delete[] data;                                                      \
            Py_DECREF(stringio);                                                \
            Py_DECREF(result);                                                  \
            return !PyErr_Occurred();                                           \
        }                                                                       \
                                                                                \
        static handle cast(MessageType src,                                     \
                           return_value_policy /* policy /, handle / parent */) \
        {                                                                       \
            ros::message_traits::DataType<MessageType::Type> type;              \
            throw_pretty("Can't create python object from message of type '"    \
                         << type.value() << "'!");                              \
        }                                                                       \
    };                                                                          \
    }                                                                           \
    }


// Example lines on how they load some message
// #include <moveit_msgs/PlanningScene.h>
// #include <moveit_msgs/PlanningSceneWorld.h>
// ROS_MESSAGE_WRAPPER(moveit_msgs::PlanningScene);
// ROS_MESSAGE_WRAPPER(moveit_msgs::PlanningSceneWorld);

// I'd need to do 
// #include <sensor_msgs/Image.h>
// ROS_MESSAGE_WRAPPER(sensor_msgs::Image)

// We want to make

// image_transport::ImageTransport available to Python
// more precisely...

// image_transport::Publisher
// should accept sensor_msgs/Image Python object to publish
// so it does all the job of image_transport different kind of
// compressions available

// image_transport::Subscriber
// The callbacks should be given to Python in sensor_msgs/Image
// Python message

// I guess I'll need to have a C++ init
// ros::init("node_name_cpp");

// with that, a nodehandle, to be able to do anything
// ros::NodeHandle nh;

// image_transport::ImageTransport it(nh);
// image_transport::Subscriber sub;
  
// image_transport::Publisher pub = it.advertise("topic_that_will_have_all_image_transports", 1, out_transport);
// the method publish from pub should be wrapped to accept Python sensor_msgs/Image messages

// void imageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//    send to Python callback
//    https://pybind11.readthedocs.io/en/stable/advanced/cast/functional.html
// }


// The callback needs to be given to Python, giving it a Python sensor_msgs/Image
// we need to be able to set the transport hint
// sub = it.subscribe("topic_that_can_be_given_any_transport_hint", 1, imageCallback, in_transport);


// We need a way to make this spin, from Python too, I guess
