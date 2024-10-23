/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPRobotArm.h
 * @author: ctaylor319@gatech.edu
 * @date: 10/03/2024
 * @brief: The GVIMP optimizer to integrate the robot arm
 * into the rest of the automation framework.
 *
 */

#pragma once

#include <instances/gvimp/GVIMPRobotSDF.h>

#include "RobotArm3D.h"

namespace vimp{

using GVIMPRobotArm = GVIMPRobotSDF<gpmp2::ArmModel, RobotArm3D>;

class RobotArmMotionPlanner
{
public:

    // Constructors & Destructors
    RobotArmMotionPlanner();
    ~RobotArmMotionPlanner();

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
     * @return means and covariances of planned path
     */
    std::pair<VectorXd, SpMat> findBestPath(VectorXd start_pos, VectorXd goal_pos, gpmp2::SignedDistanceField sdf);

    // Getter functions
    RobotArm3D getRobotSDF();
    GVIMPParams getParams();

private:

    VectorXd _mean;
    SpMat _precision;
    GVIMPParams _params;
    QuadratureWeightsMap _nodes_weights_map;
    RobotArm3D _robot_sdf;

    /**
     * @brief: Helper function for finding the best path
     */
    void optimize();
};

} //namespace vimp
