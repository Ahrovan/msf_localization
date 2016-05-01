
#include "msf_localization_core/coded_visual_marker_landmark_core.h"

#include "msf_localization_core/map_element_state_core.h"
#include "msf_localization_core/coded_visual_marker_landmark_state_core.h"


CodedVisualMarkerLandmarkCore::CodedVisualMarkerLandmarkCore():
    MapElementCore()
{
    init();

    return;
}

CodedVisualMarkerLandmarkCore::CodedVisualMarkerLandmarkCore(std::weak_ptr<MsfStorageCore> TheMsfStorageCore) :
    MapElementCore(TheMsfStorageCore)
{
    init();

    return;
}

CodedVisualMarkerLandmarkCore::~CodedVisualMarkerLandmarkCore()
{
    return;
}

int CodedVisualMarkerLandmarkCore::init()
{
    // Dimensions
    dimension_state_=0;
    dimension_error_state_=0;

    dimension_parameters_+=3+4;
    dimension_error_parameters_+=3+3;

    // Flags
    flag_estimation_position_visual_marker_wrt_world=false;
    flag_estimation_attitude_visual_marker_wrt_world=false;

    // Noises
    covariancePositionVisualMarkerWrtWorld.setZero();
    covarianceAttitudeVisualMarkerWrtWorld.setZero();

    // Id
    id_=-1;

    // Map Type
    setMapElementType(MapElementTypes::coded_visual_marker);


    return 0;
}

int CodedVisualMarkerLandmarkCore::getId() const
{
    return this->id_;
}

int CodedVisualMarkerLandmarkCore::setId(int id)
{
    this->id_=id;
    return 0;
}

int CodedVisualMarkerLandmarkCore::readConfig(pugi::xml_node map_element, std::shared_ptr<CodedVisualMarkerLandmarkStateCore>& MapElementInitStateCore)
{
    // Map Element Core Pointer
    //std::shared_ptr<MapElementCore> TheMapElementCore(this);
    //std::shared_ptr<MapElementCore> TheMapElementCore=std::dynamic_pointer_cast<MapElementCore>(this->getMsfElementCoreSharedPtr());

    // Set pointer to the SensorCore
    //TheMapElementCore->setMsfElementCore(TheMapElementCore);

    // Create a class for the SensorStateCore
    if(!MapElementInitStateCore)
        MapElementInitStateCore=std::make_shared<CodedVisualMarkerLandmarkStateCore>(this->getMsfElementCoreSharedPtr());

    // Set pointer to the SensorCore
    //MapElementInitStateCore->setTheMapElementCore(TheMapElementCore);


    // Set the access to the Storage core
    //TheRosSensorImuInterface->setTheMsfStorageCore(std::make_shared<MsfStorageCore>(this->TheStateEstimationCore));
    //TheMapElementCore->setMsfStorageCore(TheMsfStorageCore);


    // Visual landmark
    pugi::xml_node visual_landmark=map_element.child("visual_landmark");


    // Auxiliar reading value
    std::string readingValue;


    /// id
    std::string idString=visual_landmark.child_value("id");
    this->setId(std::stoi(idString));


    /// Name
    this->setMapElementName("visual_marker_"+idString);


    //// Configs

    /// Pose of the map element wrt worl
    pugi::xml_node pose_in_world=visual_landmark.child("pose_in_world");

    // Position of the sensor wrt robot
    readingValue=pose_in_world.child("position").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationPositionVisualMarkerWrtWorld();

    // Attitude of the sensor wrt robot
    readingValue=pose_in_world.child("attitude").child_value("enabled");
    if(std::stoi(readingValue))
        this->enableEstimationAttitudeVisualMarkerWrtWorld();



    //// Init State

    /// Pose of the visual marker wrt world

    // Position of the visual marker wrt world
    readingValue=pose_in_world.child("position").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2];
        MapElementInitStateCore->setPosition(init_estimation);
    }

    // Attitude of the visual marker wrt world
    readingValue=pose_in_world.child("attitude").child_value("init_estimation");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector4d init_estimation;
        stm>>init_estimation[0]>>init_estimation[1]>>init_estimation[2]>>init_estimation[3];
        MapElementInitStateCore->setAttitude(init_estimation);
    }


    /// Parameters

    // None


    //// Init Variances


    /// Pose of the visual marker wrt world

    // Position of the visual marker wrt world
    readingValue=pose_in_world.child("position").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setCovariancePositionVisualMarkerWrtWorld(variance.asDiagonal());
    }


    // Attitude of the visual marker wrt world
    readingValue=pose_in_world.child("attitude").child_value("init_var");
    {
        std::istringstream stm(readingValue);
        Eigen::Vector3d variance;
        stm>>variance[0]>>variance[1]>>variance[2];
        this->setCovarianceAttitudeVisualMarkerWrtWorld(variance.asDiagonal());
    }



    /// Other Parameters

    // None


    // Noises in the estimation (if enabled)

    // None


    // Prepare covariance matrix
    this->prepareCovarianceInitErrorState();


    /// Finish


    // End
    return 0;
}

bool CodedVisualMarkerLandmarkCore::isEstimationPositionVisualMarkerWrtWorldEnabled()
{
    return flag_estimation_position_visual_marker_wrt_world;
}

int CodedVisualMarkerLandmarkCore::enableEstimationPositionVisualMarkerWrtWorld()
{
    if(!flag_estimation_position_visual_marker_wrt_world)
    {
        flag_estimation_position_visual_marker_wrt_world=true;
        dimension_state_+=3;
        dimension_error_state_+=3;
        dimension_parameters_-=3;
        dimension_error_parameters_-=3;
    }
    return 0;
}

int CodedVisualMarkerLandmarkCore::enableParameterPositionVisualMarkerWrtWorld()
{
    if(flag_estimation_position_visual_marker_wrt_world)
    {
        flag_estimation_position_visual_marker_wrt_world=false;
        dimension_state_-=3;
        dimension_error_state_-=3;
        dimension_parameters_+=3;
        dimension_error_parameters_+=3;
    }
    return 0;
}

Eigen::Matrix3d CodedVisualMarkerLandmarkCore::getCovariancePositionVisualMarkerWrtWorld() const
{
    return this->covariancePositionVisualMarkerWrtWorld;
}

int CodedVisualMarkerLandmarkCore::setCovariancePositionVisualMarkerWrtWorld(Eigen::Matrix3d covariancePositionVisualMarkerWrtWorld)
{
    this->covariancePositionVisualMarkerWrtWorld=covariancePositionVisualMarkerWrtWorld;
    return 0;
}


bool CodedVisualMarkerLandmarkCore::isEstimationAttitudeVisualMarkerWrtWorldEnabled()
{
    return this->flag_estimation_attitude_visual_marker_wrt_world;
}

int CodedVisualMarkerLandmarkCore::enableEstimationAttitudeVisualMarkerWrtWorld()
{
    if(!flag_estimation_attitude_visual_marker_wrt_world)
    {
        flag_estimation_attitude_visual_marker_wrt_world=true;
        dimension_state_+=4;
        dimension_error_state_+=3;
        dimension_parameters_-=4;
        dimension_error_parameters_-=3;
    }

    return 0;
}

int CodedVisualMarkerLandmarkCore::enableParameterAttitudeVisualMarkerWrtWorld()
{
    if(flag_estimation_attitude_visual_marker_wrt_world)
    {
        flag_estimation_attitude_visual_marker_wrt_world=false;
        dimension_state_-=4;
        dimension_error_state_-=3;
        dimension_parameters_+=4;
        dimension_error_parameters_+=3;
    }
    return 0;
}


Eigen::Matrix3d CodedVisualMarkerLandmarkCore::getCovarianceAttitudeVisualMarkerWrtWorld() const
{
    return this->covarianceAttitudeVisualMarkerWrtWorld;
}

int CodedVisualMarkerLandmarkCore::setCovarianceAttitudeVisualMarkerWrtWorld(Eigen::Matrix3d covarianceAttitudeVisualMarkerWrtWorld)
{
    this->covarianceAttitudeVisualMarkerWrtWorld=covarianceAttitudeVisualMarkerWrtWorld;
    return 0;
}




int CodedVisualMarkerLandmarkCore::prepareCovarianceInitErrorStateSpecific()
{
    int dimension_i=0;

    if(flag_estimation_position_visual_marker_wrt_world)
    {
        this->covariance_init_error_state_.block<3,3>(dimension_i, dimension_i)=covariancePositionVisualMarkerWrtWorld;
        dimension_i+=3;
    }

    if(flag_estimation_attitude_visual_marker_wrt_world)
    {
        this->covariance_init_error_state_.block<3,3>(dimension_i, dimension_i)=covarianceAttitudeVisualMarkerWrtWorld;
        dimension_i+=3;
    }

    return 0;
}

/*
Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkCore::getCovarianceMeasurement()
{
    Eigen::SparseMatrix<double> covariance;


    return covariance;
}
*/

Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkCore::getCovarianceParameters()
{
    Eigen::SparseMatrix<double> covariance;

    covariance.resize(this->getDimensionErrorParameters(), this->getDimensionErrorParameters());
    covariance.reserve(this->getDimensionErrorParameters());

    std::vector<Eigen::Triplet<double> > tripletCovarianceParameters;


    unsigned int dimension=0;
    if(!this->isEstimationPositionVisualMarkerWrtWorldEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,covariancePositionVisualMarkerWrtWorld(i,i)));

        dimension+=3;
    }
    if(!this->isEstimationAttitudeVisualMarkerWrtWorldEnabled())
    {
        for(int i=0; i<3; i++)
            tripletCovarianceParameters.push_back(Eigen::Triplet<double>(dimension+i,dimension+i,covarianceAttitudeVisualMarkerWrtWorld(i,i)));

        dimension+=3;
    }


    covariance.setFromTriplets(tripletCovarianceParameters.begin(), tripletCovarianceParameters.end());



    return covariance;
}


Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkCore::getCovarianceNoise(const TimeStamp deltaTimeStamp)
{
    Eigen::SparseMatrix<double> covariance;

    covariance.resize(0,0);

    // Do nothing

    return covariance;
}

int CodedVisualMarkerLandmarkCore::predictState(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, const std::shared_ptr<MapElementStateCore> pastStateI, std::shared_ptr<MapElementStateCore>& predictedStateI)
{
    //std::cout<<"CodedVisualMarkerLandmarkCore::predictState()"<<std::endl;

    // Poly
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState=std::static_pointer_cast<CodedVisualMarkerLandmarkStateCore>(pastStateI);
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> predictedState=std::static_pointer_cast<CodedVisualMarkerLandmarkStateCore>(predictedStateI);



    // Checks in the past state
    if(!pastState->isCorrect())
    {
        return -5;
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<CodedVisualMarkerLandmarkStateCore>(pastState->getMsfElementCoreWeakPtr());
    }

//    // Set The robot core if it doesn't exist
//    if(predictedState->getTheMapElementCoreWeak().expired())
//    {
//        predictedState->setTheMapElementCore(pastState->getTheMapElementCoreWeak());
//    }


    // Equations


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    double dt=DeltaTime.get_double();


    /// Position
    if(flag_estimation_position_visual_marker_wrt_world)
        predictedState->position_=pastState->position_;
    else
        predictedState->position_=pastState->position_;


    /// Attitude
    if(flag_estimation_attitude_visual_marker_wrt_world)
//        if(pastState->attitude_[0]<0)
//        {
//            predictedState->attitude_=-pastState->attitude_;
//            std::cout<<"CodedVisualMarkerLandmarkCore::predictState() quaternion!!"<<std::endl;
//        }
//        else
            predictedState->attitude_=pastState->attitude_;
    else
//        if(pastState->attitude_[0]<0)
//        {
//            predictedState->attitude_=-pastState->attitude_;
//            std::cout<<"CodedVisualMarkerLandmarkCore::predictState() quaternion!!"<<std::endl;
//        }
//        else
            predictedState->attitude_=pastState->attitude_;


    // Finish
    predictedStateI=predictedState;

    return 0;
}

// Jacobian
int CodedVisualMarkerLandmarkCore::predictErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<MapElementStateCore> pastStateI, std::shared_ptr<MapElementStateCore>& predictedStateI)
{
    // Poly
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> pastState=std::static_pointer_cast<CodedVisualMarkerLandmarkStateCore>(pastStateI);
    std::shared_ptr<CodedVisualMarkerLandmarkStateCore> predictedState=std::static_pointer_cast<CodedVisualMarkerLandmarkStateCore>(predictedStateI);


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        return 1;
    }


    //Delta Time
    TimeStamp DeltaTime=currentTimeStamp-previousTimeStamp;
    // delta time
    double dt=DeltaTime.get_double();



    //// Jacobian Error State

    // Jacobian Size
    predictedState->jacobian_error_state_.resize(dimension_error_state_, dimension_error_state_);
    predictedState->jacobian_error_state_.reserve(dimension_error_state_);

    std::vector<Eigen::Triplet<double> > tripletJacobianErrorState;



    /// Jacobian of the error: Linear Part

    // posi / posi
    if(flag_estimation_position_visual_marker_wrt_world)
    {
        //predictedState->error_state_jacobian_.linear.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
        for(int i=0; i<3; i++)
            tripletJacobianErrorState.push_back(Eigen::Triplet<double>(i,i,1));
    }



    /// Jacobian of the error -> Angular Part

    // att / att
    if(flag_estimation_attitude_visual_marker_wrt_world)
    {
        //predictedState->error_state_jacobian_.angular.block<3,3>(0,0)=Eigen::MatrixXd::Identity(3,3);
        for(int i=0; i<3; i++)
            tripletJacobianErrorState.push_back(Eigen::Triplet<double>(3+i,3+i,1));
    }


    /// Set
    predictedState->jacobian_error_state_.setFromTriplets(tripletJacobianErrorState.begin(), tripletJacobianErrorState.end());



    ///// Jacobian Error State Noise

    predictedState->jacobian_error_state_noise_.resize(0, 0);
    predictedState->jacobian_error_state_noise_.reserve(0);

    std::vector<Eigen::Triplet<double> > tripletJacobianErrorStateNoise;

    // Nothing to do



    // Finish
    predictedStateI=predictedState;

    return 0;
}

