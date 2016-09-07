
#include "msf_localization_core/sensor_core.h"

//#include "state_estimation_core.h"

#include "msf_localization_core/msf_storage_core.h"
#include "msf_localization_core/msfLocalization.h"


SensorCore::SensorCore() :
    SensorBasics(),
    MsfElementCore()
{
    init();

    return;
}

SensorCore::SensorCore(MsfLocalizationCore* msf_localization_core_ptr) :
    SensorBasics(),
    MsfElementCore(msf_localization_core_ptr)
{

    init();

    // end
    return;
}

SensorCore::~SensorCore()
{

    return;
}

int SensorCore::init()
{
    // Dimensions
    dimension_state_=0;
    dimension_error_state_=0;
    dimension_parameters_=4+3;
    dimension_error_parameters_=3+3;
    dimension_measurement_=0;
    dimension_error_measurement_=0;
    dimension_noise_=0;

    // Element Type
    this->setMsfElementCoreType(MsfElementCoreTypes::sensor);

    // Sensor name
    //sensor_name_="sensor";

    // Flags
    flagEstimationAttitudeSensorWrtRobot=false;
    flagEstimationPositionSensorWrtRobot=false;


    // Noise
    noiseAttitudeSensorWrtRobot.setZero();
    noisePositionSensorWrtRobot.setZero();

    return 0;
}

std::weak_ptr<SensorCore> SensorCore::getSensorCoreWeakPtr() const
{
    //std::weak_ptr<SensorCore> msf_element_core_weak_ptr_;
    //std::shared_ptr<SensorCore> msf_element_core_ptr_=std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreWeakPtr());
    //msf_element_core_weak_ptr_=msf_element_core_ptr_;
    //return msf_element_core_weak_ptr_;
    return std::weak_ptr<SensorCore>(std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr()));
}

std::shared_ptr<SensorCore> SensorCore::getSensorCoreSharedPtr() const
{
    //std::shared_ptr<SensorCore> msf_element_core_ptr_=std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr());
    //return msf_element_core_ptr_;
    return std::dynamic_pointer_cast<SensorCore>(this->getMsfElementCoreSharedPtr());
}

int SensorCore::getDimensionMeasurement() const
{
    return this->dimension_measurement_;
}

int SensorCore::getDimensionErrorMeasurement() const
{
    return this->dimension_error_measurement_;
}

int SensorCore::setMeasurement(const TimeStamp& time_stamp, const std::shared_ptr<SensorMeasurementCore> sensor_measurement)
{
    if(!isSensorEnabled())
        return 0;

    if(!this->isCorrect())
    {
        std::cout<<"ERROR"<<std::endl;
        return -1;
    }

    // If it does not have any measurement, do not set measurement
    if(!sensor_measurement->isMeasurementSet())
    {
        std::cout<<"SensorCore::setMeasurement() no isMeasurementSet()"<<std::endl;
        return 0;
    }

    // check if it is correct
    if(!sensor_measurement->isCorrect())
    {
        std::cout<<"SensorCore::setMeasurement() no isCorrect()"<<std::endl;
        return 0;
    }

    // Set
    int error_set_measurement=this->getMsfLocalizationCorePtr()->setMeasurement(time_stamp, sensor_measurement);
    if(error_set_measurement > 0)
    {
        //std::cout<<"SensorCore::setMeasurement() error > 0 setMeasurement()"<<std::endl;
        return 2;
    }
    else if(error_set_measurement < 0)
    {
        std::cout<<"SensorCore::setMeasurement() error < 0 setMeasurement()"<<std::endl;
        return -2;
    }

    return 0;
}

int SensorCore::setMeasurementList(const TimeStamp& time_stamp, const std::list< std::shared_ptr<SensorMeasurementCore> >& sensor_measurement_list)
{
    if(!isSensorEnabled())
        return 0;

    // Check if there are measurements
    if(sensor_measurement_list.size() == 0)
        return 0;

    // check all the measurements from the list
    // TODO

    // Set
    int error_set_measurement=this->getMsfLocalizationCorePtr()->setMeasurementList(time_stamp, sensor_measurement_list);
    if(error_set_measurement > 0)
    {
        //std::cout<<"SensorCore::setMeasurementList() error setMeasurementList()"<<std::endl;
        return 1;
    }
    else if(error_set_measurement < 0)
    {
        std::cout<<"SensorCore::setMeasurementList() error < 0 setMeasurementList()"<<std::endl;
        return -1;
    }

    return 0;
}

bool SensorCore::isEstimationAttitudeSensorWrtRobotEnabled() const
{
    return this->flagEstimationAttitudeSensorWrtRobot;
}

int SensorCore::enableEstimationAttitudeSensorWrtRobot()
{
    if(!this->flagEstimationAttitudeSensorWrtRobot)
    {
        // Enable
        this->flagEstimationAttitudeSensorWrtRobot=true;
        // Update State Dimension
        this->dimension_state_+=4;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        // Update param
        this->dimension_parameters_-=4;
        // Update error param
        this->dimension_error_parameters_-=3;
    }
    return 0;
}

int SensorCore::enableParameterAttitudeSensorWrtRobot()
{
    if(this->flagEstimationAttitudeSensorWrtRobot)
    {
        // Enable
        this->flagEstimationAttitudeSensorWrtRobot=false;
        // Update State Dimension
        this->dimension_state_-=4;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        // Update param
        this->dimension_parameters_+=4;
        // Update error param
        this->dimension_error_parameters_+=3;

    }
    return 0;
}

Eigen::Matrix3d SensorCore::getNoiseAttitudeSensorWrtRobot() const
{
    return this->noiseAttitudeSensorWrtRobot;
}

int SensorCore::setNoiseAttitudeSensorWrtRobot(const Eigen::Matrix3d &noiseAttitudeSensorWrtRobot)
{
    this->noiseAttitudeSensorWrtRobot=noiseAttitudeSensorWrtRobot;
    return 0;
}



bool SensorCore::isEstimationPositionSensorWrtRobotEnabled() const
{
    return this->flagEstimationPositionSensorWrtRobot;
}

int SensorCore::enableEstimationPositionSensorWrtRobot()
{
    if(!this->flagEstimationPositionSensorWrtRobot)
    {
        // Enable
        this->flagEstimationPositionSensorWrtRobot=true;
        // Update State Dimension
        this->dimension_state_+=3;
        // Update Error State Dimension
        this->dimension_error_state_+=3;
        //
        this->dimension_parameters_-=3;
        //
        this->dimension_error_parameters_-=3;
    }
    return 0;
}

int SensorCore::enableParameterPositionSensorWrtRobot()
{
    if(this->flagEstimationPositionSensorWrtRobot)
    {
        // Enable
        this->flagEstimationPositionSensorWrtRobot=false;
        // Update State Dimension
        this->dimension_state_-=3;
        // Update Error State Dimension
        this->dimension_error_state_-=3;
        //
        this->dimension_parameters_+=3;
        //
        this->dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d SensorCore::getNoisePositionSensorWrtRobot() const
{
    return this->noisePositionSensorWrtRobot;
}

int SensorCore::setNoisePositionSensorWrtRobot(const Eigen::Matrix3d &noisePositionSensorWrtRobot)
{
    this->noisePositionSensorWrtRobot=noisePositionSensorWrtRobot;
    return 0;
}

Eigen::SparseMatrix<double> SensorCore::getCovarianceMeasurement()
{
    throw;
}

int SensorCore::predictErrorMeasurementJacobianInit(// Current State
                                                    const std::shared_ptr<StateComponent> &current_state,
                                                    // Sensor Measurement
                                                    const std::shared_ptr<SensorMeasurementCore> &sensor_measurement,
                                                    // Predicted Measurements
                                                    std::shared_ptr<SensorMeasurementCore> &predicted_measurement)
{

    //Checks
    if(!current_state)
        return -1;

    if(!sensor_measurement)
        return -11;

    if(!predicted_measurement)
        return -10;


    // Dimension
    int dimension_error_measurement=sensor_measurement->getDimensionErrorMeasurement();


    /// Hx
    // World
    predicted_measurement->jacobian_error_measurement_wrt_error_state_.world.resize(dimension_error_measurement, current_state->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState());
    // Robot
    predicted_measurement->jacobian_error_measurement_wrt_error_state_.robot.resize(dimension_error_measurement, current_state->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorState());
    // Inputs
    predicted_measurement->jacobian_error_measurement_wrt_error_state_.inputs.reserve(current_state->getNumberInputStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itInputStateCore=current_state->TheListInputStateCore.begin();
        itInputStateCore!=current_state->TheListInputStateCore.end();
        ++itInputStateCore
        )
    {
        predicted_measurement->jacobian_error_measurement_wrt_error_state_.inputs.push_back(Eigen::SparseMatrix<double>(dimension_error_measurement, (*itInputStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorState()));
    }
    // Sensors
    predicted_measurement->jacobian_error_measurement_wrt_error_state_.sensors.reserve(current_state->getNumberSensorStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itSensorStateCore=current_state->TheListSensorStateCore.begin();
        itSensorStateCore!=current_state->TheListSensorStateCore.end();
        ++itSensorStateCore
        )
    {
        predicted_measurement->jacobian_error_measurement_wrt_error_state_.sensors.push_back(Eigen::SparseMatrix<double>(dimension_error_measurement, (*itSensorStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorState()));
    }
    // Map elements
    predicted_measurement->jacobian_error_measurement_wrt_error_state_.map_elements.reserve(current_state->getNumberMapElementStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementStateCore=current_state->TheListMapElementStateCore.begin();
        itMapElementStateCore!=current_state->TheListMapElementStateCore.end();
        ++itMapElementStateCore
        )
    {
        predicted_measurement->jacobian_error_measurement_wrt_error_state_.map_elements.push_back(Eigen::SparseMatrix<double>(dimension_error_measurement, (*itMapElementStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorState()));
    }



    /// Hp
    // World
    predicted_measurement->jacobian_error_measurement_wrt_error_parameters_.world.resize(dimension_error_measurement, current_state->TheGlobalParametersStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters());
    // Robot
    predicted_measurement->jacobian_error_measurement_wrt_error_parameters_.robot.resize(dimension_error_measurement, current_state->TheRobotStateCore->getMsfElementCoreSharedPtr()->getDimensionErrorParameters());
    // Inputs
    predicted_measurement->jacobian_error_measurement_wrt_error_parameters_.inputs.reserve(current_state->getNumberInputStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itInputStateCore=current_state->TheListInputStateCore.begin();
        itInputStateCore!=current_state->TheListInputStateCore.end();
        ++itInputStateCore
        )
    {
        predicted_measurement->jacobian_error_measurement_wrt_error_parameters_.inputs.push_back(Eigen::SparseMatrix<double>(dimension_error_measurement, (*itInputStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters()));
    }
    // Sensors
    predicted_measurement->jacobian_error_measurement_wrt_error_parameters_.sensors.reserve(current_state->getNumberSensorStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itSensorStateCore=current_state->TheListSensorStateCore.begin();
        itSensorStateCore!=current_state->TheListSensorStateCore.end();
        ++itSensorStateCore
        )
    {
        predicted_measurement->jacobian_error_measurement_wrt_error_parameters_.sensors.push_back(Eigen::SparseMatrix<double>(dimension_error_measurement, (*itSensorStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters()));
    }
    // Map elements
    predicted_measurement->jacobian_error_measurement_wrt_error_parameters_.map_elements.reserve(current_state->getNumberMapElementStates());
    for(std::list< std::shared_ptr<StateCore> >::iterator itMapElementStateCore=current_state->TheListMapElementStateCore.begin();
        itMapElementStateCore!=current_state->TheListMapElementStateCore.end();
        ++itMapElementStateCore
        )
    {
        predicted_measurement->jacobian_error_measurement_wrt_error_parameters_.map_elements.push_back(Eigen::SparseMatrix<double>(dimension_error_measurement, (*itMapElementStateCore)->getMsfElementCoreSharedPtr()->getDimensionErrorParameters()));
    }


    /// Hn
    predicted_measurement->jacobian_error_measurement_wrt_error_measurement_.measurement.resize(dimension_error_measurement, dimension_error_measurement);


    // End
    return 0;
}

