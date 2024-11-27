/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPMobileRobot.h
 * @author: ctaylor319@gatech.edu
 * @date: 11/24/2024
 * @brief: The GVIMP optimizer to integrate the mobile robot
 * into the rest of the automation framework.
 *
 */

#pragma once

#include <instances/gvimp/GVIMPPlanarPRSDF.h>
#include <GaussianVI/ngd/NGD-GH.h>

#include "MobileRobot.h"

namespace vimp{

class GVIMPMobileRobot
{
public:

    // Constructors & Destructors
    GVIMPMobileRobot();
    ~GVIMPMobileRobot();

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
        VectorXd start_pos, VectorXd goal_pos, gpmp2::PlanarSDF sdf, bool visualize=false
    );

    // Getter/Setter functions
    MobileRobot getRobotSDF();
    GVIMPParams getParams();
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
    QuadratureWeightsMap _nodes_weights_map;
    MobileRobot _robot_sdf;

    /**
     * @brief: Helper function for finding the best path
     */
    std::optional<std::vector<VectorXd>> optimize(bool visualize=false);
};

} //namespace vimp
