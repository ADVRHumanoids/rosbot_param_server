#include <ros/ros.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/detail/common.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <rosbot_param_server/parameter_manager.h>

namespace py = pybind11;

bool init (std::string name, std::list<std::string> args)
{
    if(ros::ok())
    {
        ROS_ERROR("Ros node already initialized with name %s",
                  ros::this_node::getName().c_str());
        return false;
    }

    std::vector<const char *> args_vec;
    for(auto& a : args)
    {
        args_vec.push_back(a.c_str());
    }

    int argc = args_vec.size();

    char ** argv = (char **)args_vec.data();

    name += "_cpp";

    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);

    ROS_INFO("Initialized roscpp under namespace %s with name %s",
             ros::this_node::getNamespace().c_str(),
             ros::this_node::getName().c_str()
            );

    return true;
}

bool shutdown()
{
    if(ros::ok())
    {
        ROS_INFO("Shutting down ros node");
        ros::shutdown();
        return true;
    }

    std::cerr << "Ros node not running" << std::endl;
    return false;
}

void ros_update()
{
    ros::spinOnce();
}


PYBIND11_MODULE(rosbot_param_server_py, m)
{
    py::class_<ParameterManager>(m, "ParameterManager")
        .def(py::init<>())
        .def("createParameter", &ParameterManager::createParameter<double>)
        .def("setMin", &ParameterManager::setMin)
        .def("setMax", &ParameterManager::setMax)
        .def("setMinMax", &ParameterManager::setMinMax);

    m.def("init", init);
    m.def("ros_update", ros_update);
    m.def("shutdown", shutdown);

}
