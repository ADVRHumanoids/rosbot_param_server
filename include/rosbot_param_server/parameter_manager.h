#ifndef PARAMETER_MANAGER_H
#define PARAMETER_MANAGER_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <rosbot_param_server/SetString.h>
#include <rosbot_param_server/GetParameterInfo.h>

class ParameterBase {
public:
    typedef std::shared_ptr<ParameterBase> Ptr;

    ParameterBase(std::string name):
        _name(name) {}

    std::string getName() const;

    virtual std::string getValue() const = 0;

    virtual void setValue(const YAML::Node v) = 0;

private:
    std::string _name;
};

template <class T>
class Parameter : public ParameterBase {
public:
    typedef std::shared_ptr<Parameter> Ptr;
    Parameter(std::string name, std::function<void(const T)> cb, const T initial_value):
        ParameterBase(name),
        _cb(cb),
        _value(initial_value) {}

    void setValue(const YAML::Node v) override
    {
        _value = v.as<T>();
        _cb(_value);
    }

    std::string getValue() const override
    {
        YAML::Node n;
        n = _value;
        YAML::Emitter em;
        em.SetMapFormat(YAML::Flow);
        em.SetSeqFormat(YAML::Flow);
        em << n;
        return em.c_str();
    }

private:
    T _value;
    std::function<void(const T)> _cb;
};

class ParameterManager {
public:
    ParameterManager();

    template <class T>
    void createParameter(std::string name, std::function<void(T)> cb, T initial_value)
    {
        if (_parameter_map.count(name) != 0)
        {
            throw std::runtime_error("Parameter " + name + " already inserted");
        }
        auto p = std::make_shared<Parameter<T>>(name, cb, initial_value);
        _parameter_map[name] = p;
    }

private:
    bool set_parameters(rosbot_param_server::SetStringRequest& req,
                        rosbot_param_server::SetStringResponse& res);

    bool get_parameters(rosbot_param_server::GetParameterInfoRequest& req,
                        rosbot_param_server::GetParameterInfoResponse& res);

    ros::NodeHandle _nh;
    ros::ServiceServer _set_srv, _get_srv;
    std::map<std::string, ParameterBase::Ptr> _parameter_map;
};

#endif // PARAMETER_MANAGER_H
