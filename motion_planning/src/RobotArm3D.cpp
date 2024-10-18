/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: 3DRobotArm.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 10/03/2024
 *
 */

#include "RobotArm3D.h"

using namespace vimp;

RobotArm3D::RobotArm3D()
{
}

RobotArm3D::~RobotArm3D()
{
}

RobotArm3D::RobotArm3D ( double eps, double radius ) :
 Base(1, 6),
 _eps(eps),
 _radius(radius)
{
    generateArm();
}

RobotArm3D::RobotArm3D ( double eps, double radius, const gpmp2::SignedDistanceField& sdf ) :
 Base(1, 6),
 _eps(eps),
 _radius(radius)
{
    Base::_sdf = sdf;
    Base::_psdf = std::make_shared<gpmp2::SignedDistanceField>( Base::_sdf );
    generateArm();
    Base::_psdf_factor = std::make_shared<gpmp2::ObstacleSDFFactorArm>( gpmp2::ObstacleSDFFactorArm(gtsam::symbol('x', 0), Base::_robot, Base::_sdf, 0.0, _eps) );
}

inline void RobotArm3D::update_sdf ( const gpmp2::SignedDistanceField& sdf )
{
    Base::_sdf = sdf;
    Base::_psdf = std::make_shared<gpmp2::SignedDistanceField>(sdf);
    Base::_psdf_factor = std::make_shared<gpmp2::ObstacleSDFFactorArm>( gpmp2::ObstacleSDFFactorArm( gtsam::symbol('x', 0), Base::_robot, sdf, 0.0, _eps) );
}

void RobotArm3D::generateArm(){

    gtsam::Vector6 a = ( gtsam::Vector6() << 0.0, -0.1104, -0.096, 0.0, 0.0, 0.0436 ).finished();
    gtsam::Vector6 alpha = ( gtsam::Vector6() << -M_PI/2.0, 0.0, 0.0, M_PI/2.0, M_PI/2.0, 0.0 ).finished();
    gtsam::Vector6 d = ( gtsam::Vector6() << 0.13156, 0.0, 0.0, 0.06639, 0.07318, 0.0 ).finished();
    gtsam::Vector6 theta_bias = ( gtsam::Vector6() << M_PI/2.0, -M_PI/2.0, 0.0, -M_PI/2.0, M_PI/2.0, 0.0 ).finished();
    gtsam::Pose3 base_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0));
    
    Arm arm( 6, a, alpha, d, base_pose, theta_bias );
    
    // body spheres    
    BodySphereVector body_spheres;

    body_spheres.push_back(BodySphere(0, 0.06, gtsam::Point3(0.0,  -0.075,  0.0)));
    body_spheres.push_back(BodySphere(0, 0.06, gtsam::Point3(0.0,  0.0,  0.0)));

    body_spheres.push_back(BodySphere(1, 0.05, gtsam::Point3(0.1104,  0.0,  0.06639)));
    body_spheres.push_back(BodySphere(1, 0.05, gtsam::Point3(0.0,  0.0,  0.06639)));

    body_spheres.push_back(BodySphere(2, 0.04, gtsam::Point3(0.096,  0.0,  0.0)));
    body_spheres.push_back(BodySphere(2, 0.04, gtsam::Point3(0.0,  0.0,  0.0)));

    body_spheres.push_back(BodySphere(3, 0.03, gtsam::Point3(0.0,  0.0,  0.0)));

    body_spheres.push_back(BodySphere(4, 0.03, gtsam::Point3(0.0, 0.0,  0.0)));

    body_spheres.push_back(BodySphere(5, 0.02, gtsam::Point3(0.0, 0.0,  0.025)));

    Base::_robot = gpmp2::ArmModel{arm, body_spheres};
    std::cout << _robot.nr_body_spheres() << std::endl;
}

int RobotArm3D::ndof() const
{
    return _ndof;
}

int RobotArm3D::nlinks() const
{
    return _nlinks;
}