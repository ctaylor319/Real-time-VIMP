/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPMobileRobot.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 11/24/2024
 *
 */

#include <helpers/ExperimentRunner.h>

#include <Poco/Util/AbstractConfiguration.h>
#include <Poco/Util/XMLConfiguration.h>
#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include "GVIMPMobileRobot.h"

using namespace vimp;

GVIMPMobileRobot::GVIMPMobileRobot()
{
    // default values
    double total_time = 5.0; int n_states = 10; double coeff_Qc = 1.0;
    double sig_obs = 0.02; double eps_sdf = 0.2; double radius = 0.22;
    double step_size = 0.7; double init_precision_factor = 10000.0;
    double boundary_penalties = 10000.0; double temperature = 0.01;
    double high_temperature = 0.2; int low_temp_iterations = 3;
    double stop_err = 1.0; int num_iter = 6; int max_n_backtracking = 5;
    int GH_deg = 3; int nx = 4; int nu = 2;

    // Initialize config file reader
    Poco::Util::AbstractConfiguration *cfg;
    std::string install_dir = XSTRING( INSTALL_DIR );
    try {
        cfg = new Poco::Util::XMLConfiguration( install_dir+"/lib/sim_config.xml" );
    } catch ( const std::exception& e ) {
        std::cout << "Unable to open configuration file: " << e.what() << std::endl;
    }

    // Read in params
    try {
        total_time = cfg->getDouble("Mobile.total_time");
        n_states = cfg->getInt("Mobile.n_states");
        coeff_Qc = cfg->getDouble("Mobile.coeff_Qc");
        sig_obs = cfg->getDouble("Mobile.sig_obs");
        eps_sdf = cfg->getDouble("Mobile.eps_sdf");
        radius = cfg->getDouble("Mobile.radius");
        step_size = cfg->getDouble("Mobile.step_size");
        init_precision_factor = cfg->getDouble("Mobile.init_precision_factor");
        boundary_penalties = cfg->getDouble("Mobile.boundary_penalties");
        temperature = cfg->getDouble("Mobile.temperature");
        high_temperature = cfg->getDouble("Mobile.high_temperature");
        low_temp_iterations = cfg->getInt("Mobile.low_temp_iterations");
        stop_err = cfg->getDouble("Mobile.stop_err");
        num_iter = cfg->getInt("Mobile.num_iter");
        max_n_backtracking = cfg->getInt("Mobile.max_n_backtracking");
        GH_deg = cfg->getInt("Mobile.GH_deg");
        nx = cfg->getInt("Mobile.nx");
        nu = cfg->getInt("Mobile.nu");
    } catch ( const std::exception& e ) {
        std::cout << "Unable to parse goal some/all parameters: " << e.what() << std::endl;
    }

    _params = GVIMPParams( nx, nu, total_time, n_states, coeff_Qc, GH_deg, sig_obs, 
                            eps_sdf, radius, step_size, num_iter, init_precision_factor, 
                            boundary_penalties, temperature, high_temperature, low_temp_iterations, 
                            stop_err, max_n_backtracking, "map_bookshelf", "" );
    _robot_sdf = MobileRobot( _params.eps_sdf(), _params.radius() );
    initialize_GH_weights();
}

GVIMPMobileRobot::~GVIMPMobileRobot()
{

}

void GVIMPMobileRobot::initialize_GH_weights()
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

std::tuple<VectorXd, SpMat, std::optional<std::vector<VectorXd>>> GVIMPMobileRobot::findBestPath 
( 
    VectorXd start_pos, VectorXd goal_pos, gpmp2::PlanarSDF sdf, bool visualize
)
{
    // Parse start and goal pose (position+velocity) into parameters
    _params.set_m0( start_pos );
    _params.set_mT( goal_pos );

    // Update internal SDF model
    _robot_sdf.update_sdf( sdf );

    // Optimize over given parameters
    auto full_path = optimize( visualize );
    std::tuple<VectorXd, SpMat, std::optional<std::vector<VectorXd>>> res = { _mean, _precision, full_path };
    return res;
}

std::optional<std::vector<VectorXd>> GVIMPMobileRobot::optimize ( bool visualize )
{
    // parameters
    int n_states = _params.nt();
    int N = n_states - 1;
    const int dim_conf = _robot_sdf.ndof() * _robot_sdf.nlinks();
    const int dim_state = 2 * dim_conf;

    // joint dimension
    const int ndim = dim_state * n_states;

    VectorXd start_theta{ _params.m0() };
    VectorXd goal_theta{ _params.mT() };

    MatrixXd Qc{ MatrixXd::Identity(dim_conf, dim_conf)*_params.coeff_Qc() };
    MatrixXd K0_fixed{ MatrixXd::Identity(dim_state, dim_state)/_params.boundary_penalties() };

    auto robot_model = _robot_sdf.RobotModel();
    auto sdf = _robot_sdf.sdf();
    double sig_obs = _params.sig_obs(), eps_sdf = _params.eps_sdf();
    double temperature = _params.temperature(), high_temperature = _params.high_temperature();

    // initial values
    VectorXd joint_init_theta{ VectorXd::Zero(ndim) };
    VectorXd avg_vel{ (goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / _params.total_time() };

    // prior 
    double delt_t = _params.total_time() / N;

    if ( visualize ) {
        
        std::vector<VectorXd> res;
        
        for ( int i=1; i<=_params.max_iter(); ++i) {

            // Vector of base factored optimizers
            vector<std::shared_ptr<gvi::GVIFactorizedBase>> vec_factors;
            _params.set_temperature(temperature);
            _params.set_high_temperature(high_temperature);

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
                    vec_factors.emplace_back( new NGDFactorizedPlanarSDF<gpmp2::PointRobotModel>{dim_conf, 
                                                                        dim_state, 
                                                                        _params.GH_degree(),
                                                                        cost_obstacle_planar<gpmp2::PointRobotModel>, 
                                                                        gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>{gtsam::symbol('x', i), 
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
            
            // The joint optimizer
            gvi::NGDGH<gvi::GVIFactorizedBase> optimizer{ vec_factors, 
                                                dim_state, 
                                                n_states, 
                                                i, 
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

            res.push_back( optimizer.mean() );

            if ( i == _params.max_iter() ) {
                _mean = optimizer.mean();
                _precision = optimizer.precision();
            }
        }
        return res;
    }
    else {

        // Vector of base factored optimizers
        vector<std::shared_ptr<gvi::GVIFactorizedBase>> vec_factors;

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
                vec_factors.emplace_back( new NGDFactorizedPlanarSDF<gpmp2::PointRobotModel>{dim_conf, 
                                                                    dim_state, 
                                                                    _params.GH_degree(),
                                                                    cost_obstacle_planar<gpmp2::PointRobotModel>, 
                                                                    gpmp2::ObstaclePlanarSDFFactor<gpmp2::PointRobotModel>{gtsam::symbol('x', i), 
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

        gvi::NGDGH<gvi::GVIFactorizedBase> optimizer{ vec_factors, 
                                        dim_state, 
                                        n_states, 
                                        _params.max_iter(), 
                                        _params.temperature(), 
                                        _params.high_temperature() };

        optimizer.set_max_iter_backtrack( _params.max_n_backtrack() );
        optimizer.set_niter_low_temperature( _params.max_iter_lowtemp() );
        optimizer.set_stop_err( _params.stop_err() );

        optimizer.set_mu( joint_init_theta );

        optimizer.initilize_precision_matrix( _params.initial_precision_factor() );

        optimizer.set_step_size_base( _params.step_size() ); // a local optima

        optimizer.optimize( true );

        _mean = optimizer.mean();
        _precision = optimizer.precision();
        return std::nullopt;
    }
}

MobileRobot GVIMPMobileRobot::getRobotSDF() { return _robot_sdf; }
GVIMPParams GVIMPMobileRobot::getParams() { return _params; }
void GVIMPMobileRobot::setTotalTime ( double totTime ) { _params.set_total_time( totTime ); }
void GVIMPMobileRobot::setSigObs ( double sigObs ) { _params.update_sig_obs( sigObs ); }
void GVIMPMobileRobot::setStepSize ( double stepSize ) { _params.update_step_size( stepSize ); }
void GVIMPMobileRobot::setInitPrecisionFactor ( double initPrecFac ) { _params.update_initial_precision_factor( initPrecFac ); }
void GVIMPMobileRobot::setBoundaryPenalties ( double pen ) { _params.update_boundary_penalties( pen ); }
void GVIMPMobileRobot::setTemperature ( double temp ) { _params.set_temperature( temp ); }
void GVIMPMobileRobot::setHighTemperature ( double highTemp ) { _params.set_high_temperature( highTemp ); }
void GVIMPMobileRobot::setNumIter ( int numIter ) { _params.update_max_iter( numIter ); }