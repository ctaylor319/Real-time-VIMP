/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: MobileRobot.h
 * @author: ctaylor319@gatech.edu
 * @date: 11/24/2024
 * @brief: Defines the robot's physical model for GVIMP.
 *
 */

#pragma once

#include <robots/RobotSDFBase.h>
#include <gtsam/inference/Symbol.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
#include <gpmp2/kinematics/PointRobotModel.h>

namespace vimp{

using namespace gpmp2;
using Base = RobotSDFBase<gpmp2::PointRobotModel, gpmp2::PlanarSDF, gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>>;

class MobileRobot : public Base
{
public:

    // Constructors & Destructors
    MobileRobot();
    virtual ~MobileRobot();
    MobileRobot(double eps, double radius);
    MobileRobot(double eps, double radius, const gpmp2::PlanarSDF& sdf);

    /**
     * @brief: Update the SDF the robot is using
     *
     * @param sdf [in]
     */
    inline void update_sdf(const gpmp2::PlanarSDF& sdf);

    /**
     * @brief: Create a model of the arm that can be interpreted
     * by the motion planner.
     */
    void generateRobot();

    int ndof() const;
    int nlinks() const;
    
private:

    double _eps, _radius;
};

}// namespace gpmp2