#include <rosbot_param_server/parameter_manager.h>


ParameterManager::ParameterManager():
_nh("horizon")
{
    _set_srv = _nh.advertiseService("set_parameters",
                                &ParameterManager::set_parameters,
                                this);

    _get_srv = _nh.advertiseService("get_parameter_info",
                                &ParameterManager::get_parameters,
                                this);
}


bool ParameterManager::set_parameters(rosbot_param_server::SetStringRequest& req,
                                      rosbot_param_server::SetStringResponse& res)
{
    auto y = YAML::Load(req.request);

    if(!y.IsMap())
    {
        res.message = "provided string must be a yaml map (param name -> value)";
        res.success = false;
        return true;
    }

    res.success = true;

    for(auto pair : y)
    {
        ParameterBase::Ptr p = _parameter_map[pair.first.as<std::string>()];

        if (!p)
        {
            res.message = "parameter " + pair.first.as<std::string>() + " does not exist";
            res.success = false;
            return true;
        }

        p->setValue(pair.second);
    }
    return true;
}

bool ParameterManager::get_parameters(rosbot_param_server::GetParameterInfoRequest& req,
                                      rosbot_param_server::GetParameterInfoResponse& res)
{
    for (auto pair : _parameter_map)
    {
        res.name.push_back(pair.first);
        res.value.push_back(pair.second->getValue());
        res.type.push_back("double");
        res.descriptor.push_back("{type: InRange, min: 0, max: 500}");
    }

    return true;
}

std::string ParameterBase::getName() const
{
    return _name;
}

