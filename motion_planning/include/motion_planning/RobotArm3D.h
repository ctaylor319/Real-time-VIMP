/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: 3DRobotArm.h
 * @author: ctaylor319@gatech.edu
 * @date: 10/03/2024
 * @brief: Defines the robot's physical model for GVIMP
 *
 */

#pragma once

#include <robots/RobotSDFBase.h>
#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>

namespace vimp{

using namespace gpmp2;
using Base = RobotSDFBase<gpmp2::ArmModel, gpmp2::SignedDistanceField, gpmp2::ObstacleSDFFactorArm>;

    class RobotArm3D : public Base
    {
    public:
        RobotArm3D();
        virtual ~RobotArm3D();
        RobotArm3D(double eps, double radius);
        RobotArm3D(double eps, double radius, const gpmp2::SignedDistanceField& sdf);
        inline void update_sdf(const gpmp2::SignedDistanceField& sdf);
        void generateArm();
        int ndof() const;
        int nlinks() const;
    private:
        double _eps, _radius;
    };

}// namespace