/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPRobotArm_Cuda.h
 * @author: ctaylor319@gatech.edu
 * @date: 11/10/2024
 * @brief: The CUDA-GVIMP optimizer to integrate the robot arm
 * into the rest of the automation framework.
 *
 */

#pragma once

#include <helpers/ExperimentParams.h>
#include <GaussianVI/ngd/NGDFactorizedBaseGH_Cuda.h>

namespace vimp{

using GHFunction = std::function<MatrixXd(const VectorXd&)>;

class GVIMPRobotArm_Cuda
{
public:

    // Constructors & Destructors
    GVIMPRobotArm_Cuda();
    ~GVIMPRobotArm_Cuda();

    /**
     * @brief: Read in the generated Gaussian-hermite weights (see README
     * for more info).
     */
    void initialize_GH_weights();

    /**
     * @brief: Find the best path for the robot to take at its current state.
     *
     * @param start_pos [in]: the current position of the robot.
     *
     * @param goal_pos [in]: the goal position of the robot.
     *
     * @param sdf [in]: The signed distance field of the environment.
     *
     * @param visualize [in]: Whether you want to use Matplotlib to visualize results. Default value is false.
     *
     * @return means and covariances of planned path
     */
    std::tuple<VectorXd, SpMat, std::optional<std::vector<VectorXd>>> findBestPath
    (
        VectorXd start_pos, VectorXd goal_pos, gpmp2::SignedDistanceField sdf, bool visualize=false
    );

    // Getter/Setter functions
    // RobotArm3D getRobotSDF();
    GVIMPParams getParams();
    int nlinks();
    int ndof();
    void setTotalTime(double totTime);
    void setSigObs(double sigObs);
    void setStepSize(double stepSize);
    void setInitPrecisionFactor(double initPrecFac);
    void setBoundaryPenalties(double pen);
    void setTemperature(double temp);
    void setHighTemperature(double highTemp);
    void setNumIter(int numIter);

private:

    VectorXd _mean;
    SpMat _precision;
    GVIMPParams _params;
    gpmp2::SignedDistanceField _sdf;
    int _nlinks;
    int _ndof;
    // RobotArm3D _robot_sdf;
    // QuadratureWeightsMap _nodes_weights_map;

    std::shared_ptr<SparseGaussHermite_Cuda<GHFunction>> _gh_ptr;
    std::shared_ptr<CudaOperation_3dArm> _cuda_ptr;
    std::shared_ptr<QuadratureWeightsMap> _nodes_weights_map_ptr;

    /**
     * @brief: Helper function for finding the best path
     */
    std::optional<std::vector<VectorXd>> optimize(bool visualize=false);
};

} //namespace vimp
