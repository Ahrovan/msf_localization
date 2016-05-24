
#include "msf_localization_core/robot_core.h"


#include "msf_localization_core/msf_storage_core.h"



RobotCore::RobotCore() :
    MsfElementCore()
{
    init();

    return;
}


RobotCore::RobotCore(const std::weak_ptr<MsfStorageCore> msf_storage_core_ptr) :
    MsfElementCore(msf_storage_core_ptr)
{
    init();

    return;
}

RobotCore::~RobotCore()
{

    return;
}

int RobotCore::init()
{
    // Dimensions
    dimension_state_=0;
    dimension_error_state_=0;
    dimension_parameters_=0;
    dimension_error_parameters_=0;
    dimension_noise_=0;

    // Element Type
    this->setMsfElementCoreType(MsfElementCoreTypes::robot);

    // Robot Type
    robot_core_type_=RobotCoreTypes::undefined;


    return 0;
}

int RobotCore::setRobotCoreType(RobotCoreTypes robot_core_type)
{
    this->robot_core_type_=robot_core_type;
    return 0;
}
RobotCoreTypes RobotCore::getRobotCoreType() const
{
    return robot_core_type_;
}

void RobotCore::setInputIds(const std::list<int>& input_ids)
{
    this->input_ids_=input_ids;
    return;
}

std::list<int> RobotCore::getInputIds() const
{
    return this->input_ids_;
}

int RobotCore::getNumInputs() const
{
    return this->input_ids_.size();
}

int RobotCore::getInputIdI(const int input_i) const
{
    if(input_i>=this->input_ids_.size())
        return -2;
    int search_input_i=0;
    for(std::list<int>::const_iterator it_input_ids=this->input_ids_.begin();
        it_input_ids!=this->input_ids_.end();
        ++it_input_ids, search_input_i++)
    {
        if(search_input_i == input_i)
        {
            return (*it_input_ids);
        }
    }
    return -1;
}

