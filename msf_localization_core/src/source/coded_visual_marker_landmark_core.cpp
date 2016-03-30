
#include "msf_localization_core/coded_visual_marker_landmark_core.h"

#include "msf_localization_core/map_element_state_core.h"
#include "msf_localization_core/coded_visual_marker_landmark_state_core.h"


CodedVisualMarkerLandmarkCore::CodedVisualMarkerLandmarkCore():
    MapElementCore()
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
    dimensionState=0;
    dimensionErrorState=0;

    dimensionParameters+=3+4;
    dimensionErrorParameters+=3+3;


    // Id
    id_=-1;


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

bool CodedVisualMarkerLandmarkCore::isEstimationPositionVisualMarkerWrtWorldEnabled()
{
    return flag_estimation_position_visual_marker_wrt_world;
}

int CodedVisualMarkerLandmarkCore::enableEstimationPositionVisualMarkerWrtWorld()
{
    if(!flag_estimation_position_visual_marker_wrt_world)
    {
        flag_estimation_position_visual_marker_wrt_world=true;
        dimensionState+=3;
        dimensionErrorState+=3;
        dimensionParameters-=3;
        dimensionErrorParameters-=3;
    }
    return 0;
}

int CodedVisualMarkerLandmarkCore::enableParameterPositionVisualMarkerWrtWorld()
{
    if(flag_estimation_position_visual_marker_wrt_world)
    {
        flag_estimation_position_visual_marker_wrt_world=false;
        dimensionState-=3;
        dimensionErrorState-=3;
        dimensionParameters+=3;
        dimensionErrorParameters+=3;
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
        dimensionState+=4;
        dimensionErrorState+=3;
        dimensionParameters-=4;
        dimensionErrorParameters-=3;
    }

    return 0;
}

int CodedVisualMarkerLandmarkCore::enableParameterAttitudeVisualMarkerWrtWorld()
{
    if(flag_estimation_attitude_visual_marker_wrt_world)
    {
        flag_estimation_attitude_visual_marker_wrt_world=false;
        dimensionState-=4;
        dimensionErrorState-=3;
        dimensionParameters+=4;
        dimensionErrorParameters+=3;
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




Eigen::MatrixXd CodedVisualMarkerLandmarkCore::getInitCovarianceErrorState()
{
    Eigen::MatrixXd covariance;

    covariance.resize(dimensionErrorState, dimensionErrorState);
    covariance.setZero();

    int dimension_i=0;

    if(flag_estimation_position_visual_marker_wrt_world)
    {
        covariance.block<3,3>(dimension_i, dimension_i)=covariancePositionVisualMarkerWrtWorld;
        dimension_i+=3;
    }

    if(flag_estimation_attitude_visual_marker_wrt_world)
    {
        covariance.block<3,3>(dimension_i, dimension_i)=covarianceAttitudeVisualMarkerWrtWorld;
        dimension_i+=3;
    }

    return covariance;
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


Eigen::SparseMatrix<double> CodedVisualMarkerLandmarkCore::getCovarianceNoise(const TimeStamp deltaTimeStamp) const
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
    if(!pastState->getTheMapElementCore())
    {
        return -5;
        std::cout<<"FreeModelRobotCore::predictState() error !pastState->getTheRobotCore()"<<std::endl;
    }


    // Create the predicted state if it doesn't exist
    if(!predictedState)
    {
        predictedState=std::make_shared<CodedVisualMarkerLandmarkStateCore>(pastState->getTheMapElementCoreWeak());
    }

    // Set The robot core if it doesn't exist
    if(predictedState->getTheMapElementCoreWeak().expired())
    {
        predictedState->setTheMapElementCore(pastState->getTheMapElementCoreWeak());
    }


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
        predictedState->attitude_=pastState->attitude_;
    else
        predictedState->attitude_=pastState->attitude_;


    // Finish
    predictedStateI=predictedState;

    return 0;
}

// Jacobian
int CodedVisualMarkerLandmarkCore::predictStateErrorStateJacobians(const TimeStamp previousTimeStamp, const TimeStamp currentTimeStamp, std::shared_ptr<MapElementStateCore> pastStateI, std::shared_ptr<MapElementStateCore>& predictedStateI)
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
    predictedState->jacobian_error_state_.resize(dimensionErrorState, dimensionErrorState);
    predictedState->jacobian_error_state_.reserve(dimensionErrorState);

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

