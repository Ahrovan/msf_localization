
#include "msf_localization_core/px4flow_sensor_measurement_core.h"

#include "msf_localization_core/px4flow_sensor_core.h"


Px4FlowSensorMeasurementCore::Px4FlowSensorMeasurementCore() :
    SensorMeasurementCore()
{
    init();

    return;
}

Px4FlowSensorMeasurementCore::Px4FlowSensorMeasurementCore(const std::weak_ptr<SensorCore> sensor_core_ptr) :
    SensorMeasurementCore(sensor_core_ptr)
{
    init();

    return;
}

Px4FlowSensorMeasurementCore::~Px4FlowSensorMeasurementCore()
{
    return;
}

int Px4FlowSensorMeasurementCore::init()
{
    // Flags
    flag_velocity_set_=false;
    flag_ground_distance_set_=false;

    // Values
    this->velocity_.setZero();
    this->ground_distance_=-1;

    // Measurement type
    setMeasurementType(MeasurementTypes::px4flow);

    return 0;
}

bool Px4FlowSensorMeasurementCore::isMeasurementSet() const
{
    if(isVelocitySet())
        return true;
    if(isGroundDistanceSet())
        return true;
    return false;
}

int Px4FlowSensorMeasurementCore::getDimensionMeasurement() const
{
    int dimension_measurement=0;

    if(isVelocitySet())
    {
        dimension_measurement+=2;
    }
    if(isGroundDistanceSet())
    {
       dimension_measurement+=1;
    }

    return dimension_measurement;
}

int Px4FlowSensorMeasurementCore::getDimensionErrorMeasurement() const
{
    int dimension_error_measurement=0;

    if(isVelocitySet())
    {
        dimension_error_measurement+=2;
    }
    if(isGroundDistanceSet())
    {
       dimension_error_measurement+=1;
    }

    return dimension_error_measurement;
}

bool Px4FlowSensorMeasurementCore::isVelocitySet() const
{
    return this->flag_velocity_set_;
}

void Px4FlowSensorMeasurementCore::setVelocity(const Eigen::Vector2d& velocity)
{
    this->velocity_=velocity;
    this->flag_velocity_set_=true;
    return;
}

Eigen::Vector2d Px4FlowSensorMeasurementCore::getVelocity() const
{
    return this->velocity_;
}

bool Px4FlowSensorMeasurementCore::isGroundDistanceSet() const
{
    return this->flag_ground_distance_set_;
}

void Px4FlowSensorMeasurementCore::setGroundDistance(double ground_distance)
{
    this->ground_distance_=ground_distance;
    this->flag_ground_distance_set_=true;
    return;
}

double Px4FlowSensorMeasurementCore::getGroundDistance() const
{
    return this->ground_distance_;
}

Eigen::VectorXd Px4FlowSensorMeasurementCore::getInnovation(const std::shared_ptr<SensorMeasurementCore> &theMatchedMeasurementI, const std::shared_ptr<SensorMeasurementCore> &thePredictedMeasurementI)
{
    // Check
    if( theMatchedMeasurementI->getSensorCoreSharedPtr() == thePredictedMeasurementI->getSensorCoreSharedPtr() && thePredictedMeasurementI->getSensorCoreSharedPtr() == this->getSensorCoreSharedPtr() )
    {
        // Ok, do nothing
    }
    else
    {
        std::cout<<"Px4FlowSensorMeasurementCore::getInnovation() error"<<std::endl;
        throw;
    }

    // Create the Measurement
    Eigen::VectorXd inovation_measurement;

    // Available dimension error measurement
    int dimension_error_measurement_available=this->getDimensionErrorMeasurement();

    inovation_measurement.resize(dimension_error_measurement_available, 1);
    inovation_measurement.setZero();


    // Variables
    Eigen::VectorXd matched_measurement=theMatchedMeasurementI->getMeasurement();
    Eigen::VectorXd predicted_measurement=thePredictedMeasurementI->getMeasurement();


    // Fill
    unsigned int dimension=0;

    if(this->isVelocitySet())
    {
        inovation_measurement.block<2,1>(dimension,0)=matched_measurement.block<2,1>(dimension,0) - predicted_measurement.block<2,1>(dimension,0);
        dimension+=2;
    }

    if(this->isGroundDistanceSet())
    {
        inovation_measurement(dimension,0)=matched_measurement(dimension, 1) - predicted_measurement(dimension, 1);
        dimension+=1;
    }


    return inovation_measurement;
}

Eigen::VectorXd Px4FlowSensorMeasurementCore::getMeasurement()
{
    // Create the Measurement
    Eigen::VectorXd sensor_measurement;

    // Available dimension measurement
    int dimension_measurement_available=this->getDimensionMeasurement();

    // Resize sensor measurement
    sensor_measurement.resize(dimension_measurement_available, 1);
    sensor_measurement.setZero();


    // Fill
    unsigned int dimension=0;

    if(this->isVelocitySet())
    {
        sensor_measurement.block<2,1>(dimension,0)=getVelocity();
        dimension+=2;
    }

    if(this->isGroundDistanceSet())
    {
        sensor_measurement(dimension,0)=getGroundDistance();
        dimension+=1;
    }


    return sensor_measurement;
}

Eigen::SparseMatrix<double> Px4FlowSensorMeasurementCore::getCovarianceMeasurement()
{
    // Sensor Core
    std::shared_ptr<Px4FlowSensorCore> sensor_core=std::dynamic_pointer_cast<Px4FlowSensorCore>(this->getSensorCoreSharedPtr());

    Eigen::SparseMatrix<double> covariances_matrix;

    covariances_matrix.resize(this->getDimensionErrorMeasurement(), this->getDimensionErrorMeasurement());
    //covariances_matrix.setZero();

    std::vector<Eigen::Triplet<double> > triplets_covariance_measurement;

    unsigned int dimension=0;

    if(isVelocitySet())
    {
        Eigen::Matrix2d noise_measurement_velocity=sensor_core->getNoiseMeasurementVelocity();

        BlockMatrix::insertVectorEigenTripletFromEigenDense(triplets_covariance_measurement, noise_measurement_velocity, dimension, dimension);

        dimension+=3;
    }

    if(isGroundDistanceSet())
    {
        double noise_measurement_ground_distance=sensor_core->getNoiseMeasurementGroundDistance();

        triplets_covariance_measurement.push_back(Eigen::Triplet<double>(dimension, dimension, noise_measurement_ground_distance));

        dimension+=1;
    }

    // Set
    covariances_matrix.setFromTriplets(triplets_covariance_measurement.begin(), triplets_covariance_measurement.end());

    // End
    return covariances_matrix;

}
