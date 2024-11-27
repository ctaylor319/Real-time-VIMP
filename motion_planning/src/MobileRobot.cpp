/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: MobileRobot.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 11/24/2024
 *
 */

#pragma once

#include "MobileRobot.h"

using namespace vimp;

MobileRobot::MobileRobot()
{

}

MobileRobot::~MobileRobot()
{

}

MobileRobot::MobileRobot ( double eps, double radius ) :
 Base(2, 1),
 _eps(eps),
 _radius(radius)
{
    generateRobot();
}

MobileRobot::MobileRobot ( double eps, double radius, const gpmp2::PlanarSDF& sdf ) :
 Base(2, 1),
 _eps(eps),
 _radius(radius)
{
    Base::_sdf = sdf;
    Base::_psdf = std::make_shared<gpmp2::PlanarSDF>( Base::_sdf );
    Base::_psdf_factor = std::make_shared<gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>>
            ( gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>(gtsam::symbol('x', 0), 
                Base::_robot, Base::_sdf, 0.0, _eps) );
    generateRobot();
    
}

inline void MobileRobot::update_sdf ( const gpmp2::PlanarSDF& sdf )
{
    Base::_sdf = sdf;
    Base::_psdf = std::make_shared<gpmp2::PlanarSDF>(sdf);
    Base::_psdf_factor = std::make_shared<gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>>
            ( gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>( gtsam::symbol('x', 0), 
                Base::_robot, sdf, 0.0, _eps) );
}

void MobileRobot::generateRobot()
{
    gpmp2::PointRobot robot(4, 2);
    gpmp2::BodySphereVector body_spheres;
    body_spheres.push_back(gpmp2::BodySphere(0, _radius, Point3(0.0, 0.0, 0.0)));
    Base::_robot = gpmp2::PointRobotModel(robot, body_spheres);
}

int MobileRobot::ndof() const { return _ndof; }
int MobileRobot::nlinks() const { return _nlinks; }