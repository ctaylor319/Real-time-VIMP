/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPRobotArm.h
 * @author: ctaylor319@gatech.edu
 * @date: 10/03/2024
 * @brief: GVIMP optimizer for the arm
 *
 */

#pragma once

#include <instances/gvimp/GVIMPRobotSDF.h>

#include "RobotArm3D.h"

namespace vimp{
    using GVIMPRobotArm = GVIMPRobotSDF<gpmp2::ArmModel, RobotArm3D>;

    class RobotArmMotionPlanner {
    public:
        RobotArmMotionPlanner();
        RobotArmMotionPlanner(std::map<std::string, double> params);
        ~RobotArmMotionPlanner();
        void initialize_GH_weights();
        std::pair<VectorXd, SpMat> findBestPath(VectorXd start_pos, VectorXd goal_pos, gpmp2::SignedDistanceField sdf);
        void optimize();
        RobotArm3D getRobotSDF();
    private:
        VectorXd _mean;
        SpMat _precision;
        GVIMPParams _params;
        QuadratureWeightsMap _nodes_weights_map;
        RobotArm3D _robot_sdf;
    };
} //namespace vimp
