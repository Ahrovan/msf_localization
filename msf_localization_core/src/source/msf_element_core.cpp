#include "msf_localization_core/msf_element_core.h"

//#include "msf_localization_core/msf_storage_core.h"


// State
#include "msf_localization_core/state_core.h"

// Measurement
#include "msf_localization_core/sensor_measurement_core.h"

// Input Command
#include "msf_localization_core/input_command_core.h"



MsfElementCore::MsfElementCore()
{
    init();

    return;
}

MsfElementCore::MsfElementCore(MsfLocalizationCore *msf_localization_core_ptr)
{
    //std::cout<<"MsfElementCore::MsfElementCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr)"<<std::endl;

    init();

    //this->msf_element_core_ptr_=msf_element_core_ptr;
    this->msf_localization_core_ptr_=msf_localization_core_ptr;

    return;
}

MsfElementCore::~MsfElementCore()
{
    destroy();

    return;
}

int MsfElementCore::init()
{
    // Init ptr to null
    msf_localization_core_ptr_=nullptr;

    // Type
    msf_element_core_type_=MsfElementCoreTypes::undefined;

    /// Dimensions
    dimension_state_=0;
    dimension_error_state_=0;
    dimension_parameters_=0;
    dimension_error_parameters_=0;
    dimension_noise_=0;

    /// LOG
    const char* env_p = std::getenv("FUSEON_STACK");
    log_path_=std::string(env_p)+"/logs/"+"logMsfElementCoreFile.txt";
    log_file_.open(log_path_);
    if(!log_file_.is_open())
    {
        std::cout<<"unable to open log file"<<std::endl;
    }

    return 0;
}

int MsfElementCore::destroy()
{
    /// Log
    if(log_file_.is_open())
    {
        log_file_.close();
    }

    return 0;
}

int MsfElementCore::setMsfElementCoreType(MsfElementCoreTypes msf_element_core_type)
{
    this->msf_element_core_type_=msf_element_core_type;
    return 0;
}

MsfElementCoreTypes MsfElementCore::getMsfElementCoreType() const
{
    return this->msf_element_core_type_;
}

int MsfElementCore::setMsfElementCorePtr(const std::weak_ptr<MsfElementCore> msf_element_core_ptr)
{
    this->msf_element_core_ptr_=msf_element_core_ptr;
    return 0;
}

std::weak_ptr<MsfElementCore> MsfElementCore::getMsfElementCoreWeakPtr() const
{
    return this->msf_element_core_ptr_;
}

std::shared_ptr<MsfElementCore> MsfElementCore::getMsfElementCoreSharedPtr() const
{
    return this->msf_element_core_ptr_.lock();
}

void MsfElementCore::setMsfLocalizationCorePtr(MsfLocalizationCore* msf_localization_core_ptr)
{
    this->msf_localization_core_ptr_=msf_localization_core_ptr;
    return;
}

MsfLocalizationCore* MsfElementCore::getMsfLocalizationCorePtr() const
{
    return this->msf_localization_core_ptr_;
}

bool MsfElementCore::isCorrect() const
{

    if(!this->msf_localization_core_ptr_)
    {
        std::cout<<"error in msf_localization_core_ptr_"<<std::endl;
        return false;
    }

    if(this->msf_element_core_ptr_.expired())
    {
        std::cout<<"error in msf_element_core_ptr_"<<std::endl;
        return false;
    }

    return true;
}


int MsfElementCore::getDimensionState() const
{
    return this->dimension_state_;
}

int MsfElementCore::setDimensionState(int dimension_state)
{
    this->dimension_state_=dimension_state;
    return 0;
}

int MsfElementCore::getDimensionErrorState() const
{
    return this->dimension_error_state_;
}

int MsfElementCore::setDimensionErrorState(int dimension_error_state)
{
    this->dimension_error_state_=dimension_error_state;
    return 0;
}

int MsfElementCore::getDimensionParameters() const
{
    return this->dimension_parameters_;
}

int MsfElementCore::setDimensionParameters(int dimension_parameters)
{
    this->dimension_parameters_=dimension_parameters;
    return 0;
}

int MsfElementCore::getDimensionErrorParameters() const
{
    return this->dimension_error_parameters_;
}

int MsfElementCore::setDimensionErrorParameters(int dimension_error_parameters)
{
    this->dimension_error_parameters_=dimension_error_parameters;
    return 0;
}

int MsfElementCore::getDimensionNoise() const
{
    return this->dimension_noise_;
}

int MsfElementCore::setDimensionNoise(int dimension_noise)
{
    this->dimension_noise_=dimension_noise;
    return 0;
}

int MsfElementCore::prepareCovarianceInitErrorState()
{
    this->covariance_init_error_state_.resize(dimension_error_state_, dimension_error_state_);
    this->covariance_init_error_state_.setZero();

    int error_prepare_covariance_init_error_state_specific=this->prepareCovarianceInitErrorStateSpecific();

    if(error_prepare_covariance_init_error_state_specific)
        return error_prepare_covariance_init_error_state_specific;

    return 0;
}

Eigen::MatrixXd MsfElementCore::getCovarianceInitErrorState() const
{
    return this->covariance_init_error_state_;
}

int MsfElementCore::log(std::string log_string)
{
    // Lock mutex
    log_file_mutex_.lock();

    // Write in file
    log_file_<<log_string;

    // Unlock mutex
    log_file_mutex_.unlock();

    return 0;
}

int MsfElementCore::predictErrorStateJacobianInit(// Past State
                                                  const std::shared_ptr<StateComponent> &past_state,
                                                  // Input Commands
                                                  const std::shared_ptr<InputCommandComponent> &input_commands,
                                                  // Predicted State
                                                  std::shared_ptr<StateCore> &predicted_state)
{

    //Checks
    if(!past_state)
        return -1;

    if(!predicted_state)
        return -10;


    // Dimension
    int dimension_error_state=this->getDimensionErrorState();


    /// Fx
    // World
    predicted_state->jacobian_error_state_.world.resize(dimension_error_state, past_state->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState());
    // Robot
    predicted_state->jacobian_error_state_.robot.resize(dimension_error_state, past_state->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState());
    // Inputs
    predicted_state->jacobian_error_state_.inputs.reserve(past_state->getNumberInputStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itInputStateCore=past_state->TheListInputStateCore.begin();
        itInputStateCore!=past_state->TheListInputStateCore.end();
        ++itInputStateCore
        )
    {
        predicted_state->jacobian_error_state_.inputs.push_back(Eigen::SparseMatrix<double>(dimension_error_state, (*itInputStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorState()));
    }
    // Sensors
    predicted_state->jacobian_error_state_.sensors.reserve(past_state->getNumberSensorStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itSensorStateCore=past_state->TheListSensorStateCore.begin();
        itSensorStateCore!=past_state->TheListSensorStateCore.end();
        ++itSensorStateCore
        )
    {
        predicted_state->jacobian_error_state_.sensors.push_back(Eigen::SparseMatrix<double>(dimension_error_state, (*itSensorStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorState()));
    }
    // Map elements
    predicted_state->jacobian_error_state_.map_elements.reserve(past_state->getNumberMapElementStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementStateCore=past_state->TheListMapElementStateCore.begin();
        itMapElementStateCore!=past_state->TheListMapElementStateCore.end();
        ++itMapElementStateCore
        )
    {
        predicted_state->jacobian_error_state_.map_elements.push_back(Eigen::SparseMatrix<double>(dimension_error_state, (*itMapElementStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorState()));
    }



    /// Fp
    // World
    predicted_state->jacobian_error_parameters_.world.resize(dimension_error_state, past_state->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters());
    // Robot
    predicted_state->jacobian_error_parameters_.robot.resize(dimension_error_state, past_state->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters());
    // Inputs
    predicted_state->jacobian_error_parameters_.inputs.reserve(past_state->getNumberInputStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itInputStateCore=past_state->TheListInputStateCore.begin();
        itInputStateCore!=past_state->TheListInputStateCore.end();
        ++itInputStateCore
        )
    {
        predicted_state->jacobian_error_parameters_.inputs.push_back(Eigen::SparseMatrix<double>(dimension_error_state, (*itInputStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters()));
    }
    // Sensors
    predicted_state->jacobian_error_parameters_.sensors.reserve(past_state->getNumberSensorStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itSensorStateCore=past_state->TheListSensorStateCore.begin();
        itSensorStateCore!=past_state->TheListSensorStateCore.end();
        ++itSensorStateCore
        )
    {
        predicted_state->jacobian_error_parameters_.sensors.push_back(Eigen::SparseMatrix<double>(dimension_error_state, (*itSensorStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters()));
    }
    // Map elements
    predicted_state->jacobian_error_parameters_.map_elements.reserve(past_state->getNumberMapElementStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementStateCore=past_state->TheListMapElementStateCore.begin();
        itMapElementStateCore!=past_state->TheListMapElementStateCore.end();
        ++itMapElementStateCore
        )
    {
        predicted_state->jacobian_error_parameters_.map_elements.push_back(Eigen::SparseMatrix<double>(dimension_error_state, (*itMapElementStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters()));
    }


    /// Fn
    predicted_state->jacobian_error_state_noise_.resize(dimension_error_state, this->getDimensionNoise());



    /// Fu
    predicted_state->jacobian_error_input_commands_.input_commands.reserve(input_commands->getNumberInputCommand());
    for(std::list< std::shared_ptr<InputCommandCore> >::iterator itInputCommands=input_commands->list_input_command_core_.begin();
        itInputCommands!=input_commands->list_input_command_core_.end();
        ++itInputCommands)
    {
        predicted_state->jacobian_error_input_commands_.input_commands.push_back(Eigen::SparseMatrix<double>(dimension_error_state, (*itInputCommands)->getInputCoreSharedPtr()->getDimensionErrorInputCommand()));
    }


    // End
    return 0;
}
