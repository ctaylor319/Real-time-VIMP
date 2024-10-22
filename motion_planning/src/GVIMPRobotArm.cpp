/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPRobotArm.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 10/03/2024
 *
 */

#include <helpers/ExperimentRunner.h>

#include "GVIMPRobotArm.h"
#include "GaussianVI/ngd/NGD-GH.h"

using namespace vimp;

RobotArmMotionPlanner::RobotArmMotionPlanner()
{
    double total_time = 5.0; int n_states = 10; double coeff_Qc = 1.0;
    double sig_obs = 0.02; double eps_sdf = 0.2; double radius = 0.0;
    double step_size = 0.7; double init_precision_factor = 10000.0;
    double boundary_penalties = 10000.0; double temperature = 0.01;
    double high_temperature = 0.2; int low_temp_iterations = 3;
    double stop_err = 1.0; int num_iter = 6; int max_n_backtracking = 5;
    int GH_deg = 3; int nx = 12; int nu = 6;
    _params = GVIMPParams( nx, nu, total_time, n_states, coeff_Qc, GH_deg, sig_obs, 
                            eps_sdf, radius, step_size, num_iter, init_precision_factor, 
                            boundary_penalties, temperature, high_temperature, low_temp_iterations, 
                            stop_err, max_n_backtracking, "map_bookshelf", "" );
    _robot_sdf = RobotArm3D( _params.eps_sdf(), _params.radius() );
    initialize_GH_weights();
}

RobotArmMotionPlanner::RobotArmMotionPlanner ( std::map<std::string, double> params )
{
    // Load in user specified parameters. Any that isn't specified is given a default value.
    double total_time = 5.0; int n_states = 5; double coeff_Qc = 1.0;
    double sig_obs = 0.02; double eps_sdf = 0.2; double radius = 0.0;
    double step_size = 0.7; double init_precision_factor = 10000.0;
    double boundary_penalties = 10000.0; double temperature = 0.01;
    double high_temperature = 0.2; int low_temp_iterations = 25;
    double stop_err = 1e-5; int num_iter = 50; int max_n_backtracking = 5;
    int GH_deg = 3; int nx = 12; int nu = 6;

    if ( params.count("total_time") ) {
        total_time = params["total_time"];
    }
    if ( params.count("n_states") ) {
        n_states = static_cast<int>( params["n_states"] );
    }
    if ( params.count("coeff_Qc") ) {
        coeff_Qc = params["coeff_Qc"];
    }
    if ( params.count("sig_obs") ) {
        sig_obs = params["sig_obs"];
    }
    if ( params.count("eps_sdf") ) {
        eps_sdf = params["eps_sdf"];
    }
    if ( params.count("radius") ) {
        radius = params["radius"];
    }
    if ( params.count("step_size") ) {
        step_size = params["step_size"];
    }
    if ( params.count("init_precision_factor") ) {
        init_precision_factor = params["init_precision_factor"];
    }
    if ( params.count("boundary_penalties") ) {
        boundary_penalties = params["boundary_penalties"];
    }
    if ( params.count("temperature") ) {
        temperature = params["temperature"];
    }
    if ( params.count("high_temperature") ) {
        high_temperature = params["high_temperature"];
    }
    if ( params.count("low_temp_iterations") ) {
        low_temp_iterations = static_cast<int>( params["low_temp_iterations"] );
    }
    if ( params.count("stop_err") ) {
        stop_err = params["stop_err"];
    }
    if ( params.count("num_iter") ) {
        num_iter = static_cast<int>( params["num_iter"] );
    }
    if ( params.count("max_n_backtracking") ) {
        max_n_backtracking = static_cast<int>( params["max_n_backtracking"] );
    }
    if ( params.count("GH_deg") ) {
        GH_deg = static_cast<int>( params["GH_deg"] );
    }
    if ( params.count("nx") ) {
        nx = static_cast<int>( params["nx"] );
    }
    if ( params.count("nu") ) {
        nu = static_cast<int>( params["nu"] );
    }

    _params = GVIMPParams( nx, nu, total_time, n_states, coeff_Qc, GH_deg, sig_obs, 
                            eps_sdf, radius, step_size, num_iter, init_precision_factor, 
                            boundary_penalties, temperature, high_temperature, low_temp_iterations, 
                            stop_err, max_n_backtracking, "map_bookshelf", "" );
    _robot_sdf = RobotArm3D( _params.eps_sdf(), _params.radius() );
    initialize_GH_weights();
}

RobotArmMotionPlanner::~RobotArmMotionPlanner()
{
}

// Source: ./VIMP/vimp/instances/gvimp/GVIMPRobotSDF.h
void RobotArmMotionPlanner::initialize_GH_weights()
{
    std::string GH_map_file{ source_root+"/GaussianVI/quadrature/SparseGHQuadratureWeights.bin" };
    try {
        std::ifstream ifs( GH_map_file, std::ios::binary );
        if ( !ifs.is_open() ) {
            std::string error_msg = "Failed to open file for GH weights reading in file: " + GH_map_file;
            throw std::runtime_error( error_msg );
        }

        std::cout << "Opening file for GH weights reading in file: " << GH_map_file << std::endl;
        boost::archive::binary_iarchive ia( ifs );
        ia >> _nodes_weights_map;

    } catch ( const boost::archive::archive_exception& e ) {
        std::cerr << "Boost archive exception: " << e.what() << std::endl;
    } catch ( const std::exception& e ) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
}

std::pair<VectorXd, SpMat> RobotArmMotionPlanner::findBestPath ( VectorXd start_pos, VectorXd goal_pos, gpmp2::SignedDistanceField sdf )
{
    // Parse start and goal pose (position+velocity) into parameters
    VectorXd m0( _params.nx()), mT(_params.nx() );
    VectorXd m0_pos(6); VectorXd mT_pos(6);
    m0_pos = start_pos; mT_pos = goal_pos;
    m0.block(0, 0, 6, 1) = m0_pos;
    m0.block(6, 0, 6, 1) = VectorXd::Zero(6);
    mT.block(0, 0, 6, 1) = mT_pos;
    mT.block(6, 0, 6, 1) = VectorXd::Zero(6);
    _params.set_m0( m0 );
    _params.set_mT( mT );

    // Update internal SDF model
    _robot_sdf.update_sdf( sdf ); 

    // Optimize over given parameters
    optimize();

    return std::pair<VectorXd, SpMat>( _mean, _precision );
}

// Source: ./VIMP/vimp/instances/gvimp/GVIMPRobotSDF.h
void RobotArmMotionPlanner::optimize()
{
    /// parameters
    int n_states = _params.nt();
    int N = n_states - 1;
    const int dim_conf = _robot_sdf.ndof() * _robot_sdf.nlinks();
    const int dim_state = 2 * dim_conf;

    /// joint dimension
    const int ndim = dim_state * n_states;

    VectorXd start_theta{ _params.m0() };
    VectorXd goal_theta{ _params.mT() };

    MatrixXd Qc{ MatrixXd::Identity(dim_conf, dim_conf)*_params.coeff_Qc() };
    MatrixXd K0_fixed{ MatrixXd::Identity(dim_state, dim_state)/_params.boundary_penalties() };

    /// Vector of base factored optimizers
    vector<std::shared_ptr<gvi::GVIFactorizedBase>> vec_factors;

    auto robot_model = _robot_sdf.RobotModel();
    auto sdf = _robot_sdf.sdf();
    double sig_obs = _params.sig_obs(), eps_sdf = _params.eps_sdf();
    double temperature = _params.temperature(), high_temperature = _params.high_temperature();

    /// initial values
    VectorXd joint_init_theta{ VectorXd::Zero(ndim) };
    VectorXd avg_vel{ (goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / _params.total_time() };

    /// prior 
    double delt_t = _params.total_time() / N;

    for ( int i=0; i<n_states; ++i) {

        // initial state
        VectorXd theta_i{ start_theta + double(i) * (goal_theta - start_theta) / N };

        // initial velocity: must have initial velocity for the fitst state??
        theta_i.segment(dim_conf, dim_conf) = avg_vel;
        joint_init_theta.segment(i*dim_state, dim_state) = std::move( theta_i );   

        MinimumAccGP lin_gp{ Qc, i, delt_t, start_theta };

        // fixed start and goal priors
        // Factor Order: [fixed_gp_0, lin_gp_1, obs_1, ..., lin_gp_(N-1), obs_(N-1), lin_gp_(N), fixed_gp_(N)] 
        if ( i==0 || i==n_states-1 ) {

            // lin GP factor for the first and the last support state
            if ( i==n_states-1 ) {
                vec_factors.emplace_back( new gvi::LinearGpPrior{2*dim_state, 
                                                            dim_state, 
                                                            cost_linear_gp, 
                                                            lin_gp, 
                                                            n_states, 
                                                            i-1, 
                                                            _params.temperature(), 
                                                            _params.high_temperature()} );
            }

            // Fixed gp factor
            FixedPriorGP fixed_gp{ K0_fixed, MatrixXd{theta_i} };
            vec_factors.emplace_back( new FixedGpPrior{dim_state, 
                                                        dim_state, 
                                                        cost_fixed_gp, 
                                                        fixed_gp, 
                                                        n_states, 
                                                        i,
                                                        _params.temperature(), 
                                                        _params.high_temperature()} );

        }else{
            // linear gp factors
            vec_factors.emplace_back( new gvi::LinearGpPrior{2*dim_state, 
                                                        dim_state, 
                                                        cost_linear_gp, 
                                                        lin_gp, 
                                                        n_states, 
                                                        i-1, 
                                                        _params.temperature(), 
                                                        _params.high_temperature()} );

            // collision factor
            vec_factors.emplace_back( new GVIFactorizedSDF<gpmp2::ArmModel>{dim_conf, 
                                                                dim_state, 
                                                                _params.GH_degree(),
                                                                cost_obstacle<gpmp2::ArmModel>, 
                                                                gpmp2::ObstacleSDFFactor<gpmp2::ArmModel>{gtsam::symbol('x', i), 
                                                                robot_model, 
                                                                sdf, 
                                                                1.0/sig_obs, 
                                                                eps_sdf}, 
                                                                n_states, 
                                                                i, 
                                                                temperature, 
                                                                high_temperature,
                                                                _nodes_weights_map} );    
        }
    }

    /// The joint optimizer
    gvi::NGDGH<gvi::GVIFactorizedBase> optimizer{ vec_factors, 
                                        dim_state, 
                                        n_states, 
                                        _params.max_iter(), 
                                        _params.temperature(), 
                                        _params.high_temperature() };

    optimizer.set_max_iter_backtrack( _params.max_n_backtrack() );
    optimizer.set_niter_low_temperature( _params.max_iter_lowtemp() );
    optimizer.set_stop_err( _params.stop_err() );

    // optimizer.update_file_names(_params.saving_prefix());
    optimizer.set_mu( joint_init_theta );

    optimizer.initilize_precision_matrix( _params.initial_precision_factor() );

    // optimizer.set_GH_degree(_params.GH_degree());
    optimizer.set_step_size_base( _params.step_size() ); // a local optima

    optimizer.optimize( true );

    _mean = optimizer.mean();
    _precision = optimizer.precision();
}

RobotArm3D RobotArmMotionPlanner::getRobotSDF() { return _robot_sdf; }
GVIMPParams RobotArmMotionPlanner::getParams() { return _params; }