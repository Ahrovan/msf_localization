#include "msf_localization_core/msf_element_core.h"

#include "msf_localization_core/msf_storage_core.h"


MsfElementCore::MsfElementCore()
{
    init();

    return;
}

MsfElementCore::MsfElementCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr)
{
    //std::cout<<"MsfElementCore::MsfElementCore(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr)"<<std::endl;

    init();

    //this->msf_element_core_ptr_=msf_element_core_ptr;
    this->msf_storage_core_ptr_=msf_storage_core_ptr;

    return;
}

MsfElementCore::~MsfElementCore()
{
    destroy();

    return;
}

int MsfElementCore::init()
{
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

int MsfElementCore::setMsfElementCorePtr(std::weak_ptr<MsfElementCore> msf_element_core_ptr)
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
    std::shared_ptr<MsfElementCore> msf_element_core_ptr=this->msf_element_core_ptr_.lock();
    return msf_element_core_ptr;
}

int MsfElementCore::setMsfStorageCorePtr(std::weak_ptr<MsfStorageCore> msf_storage_core_ptr)
{
    this->msf_storage_core_ptr_=msf_storage_core_ptr;
    return 0;
}

std::weak_ptr<MsfStorageCore> MsfElementCore::getMsfStorageCoreWeakPtr() const
{
    return this->msf_storage_core_ptr_;
}

std::shared_ptr<MsfStorageCore> MsfElementCore::getMsfStorageCoreSharedPtr() const
{
    std::shared_ptr<MsfStorageCore> msf_storage_core_ptr=this->msf_storage_core_ptr_.lock();
    return msf_storage_core_ptr;
}

bool MsfElementCore::isCorrect()
{
    if(this->msf_storage_core_ptr_.expired())
    {
        std::cout<<"error in msf_storage_core_ptr_"<<std::endl;
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
