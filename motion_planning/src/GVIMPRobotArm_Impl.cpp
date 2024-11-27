/**
 * @copyright Georgia Institute of Technology, 2024
 * @file: GVIMPRobotArm_Impl.cpp
 * @author: ctaylor319@gatech.edu
 * @date: 10/07/2024
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <gpmp2/obstacle/SignedDistanceField.h>

#include <Poco/Util/AbstractConfiguration.h>
#include <Poco/Util/XMLConfiguration.h>
#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include "GVIMPRobotArm_Impl.h"

GVIMPImpl::GVIMPImpl() : Node( "GVIMPImpl" ),
_state(RobotState::Idle),
_new_path_needed(false),
_reset_called(false)
{
    _gridmap_subscriber = create_subscription<grid_map_msgs::msg::GridMap>( "/gridmap_binary", 10,
        std::bind( &GVIMPImpl::gridMapCallback, this, std::placeholders::_1 ) );
    _robot_state_subscriber = create_subscription<control_msgs::msg::JointTrajectoryControllerState>( "/arm_controller/controller_state", 10,
        std::bind( &GVIMPImpl::stateCallback, this, std::placeholders::_1 ) );
    _occupied_cells_subscriber = create_subscription<visualization_msgs::msg::MarkerArray>("/occupied_cells_vis_array", 10,
        std::bind( &GVIMPImpl::cellsCallback, this, std::placeholders::_1 ) );
    _path_publisher = create_publisher<motion_planning_msgs::msg::WaypointPath>( "/vimp_path", 10 );
    _iterative_path_publisher = create_publisher<motion_planning_msgs::msg::VisualizedPath>( "/vis_path", 10 );
    _parameter_service = create_service<motion_planning_msgs::srv::RuntimeParameterInterface>( "robot_runtime_interface/parameter_tuning",
        std::bind(&GVIMPImpl::handleParameterRequest, this, std::placeholders::_1, std::placeholders::_2) );
    _path_service = create_service<motion_planning_msgs::srv::RuntimePathInterface>( "robot_runtime_interface/set_path",
        std::bind(&GVIMPImpl::handlePathRequest, this, std::placeholders::_1, std::placeholders::_2) );

    _path_planner = std::unique_ptr<vimp::GVIMPRobotArm>( new vimp::GVIMPRobotArm() ); // Instance of the motion planner

    this->declare_parameter("visualize", false);

    // Initialize config file reader
    Poco::Util::AbstractConfiguration *cfg;
    std::string install_dir = XSTRING( INSTALL_DIR );
    try {
        cfg = new Poco::Util::XMLConfiguration( install_dir+"/lib/sim_config.xml" );
    } catch ( const std::exception& e ) {
        std::cout << "Unable to open configuration file: " << e.what() << std::endl;
    }

    // Read in the goal position
    try {
        std::vector<double> goal_vec;
        Poco::StringTokenizer goal_str( cfg->getString("Arm.goal"), " " );
        for ( Poco::StringTokenizer::Iterator it = goal_str.begin(); it != goal_str.end(); ++it ) {
            goal_vec.push_back( Poco::NumberParser::parseFloat(*it) );
        }
        _goal_pos = Map<VectorXd, Unaligned>( goal_vec.data(), goal_vec.size() );
        _new_path_needed = true;
    } catch ( const std::exception& e ) {
        std::cout << "Unable to parse goal pose: " << e.what() << std::endl;
    }

    // If the number of joint positions you gave is different then how many the robot model has,
    // throw an exception
    if ( _goal_pos.size() != _path_planner->getRobotSDF().nlinks() ) {
        throw std::invalid_argument("Goal has incorrect number of joint positions");
    }
}

GVIMPImpl::~GVIMPImpl()
{

}

void GVIMPImpl::stateCallback ( control_msgs::msg::JointTrajectoryControllerState msg )
{
    trajectory_msgs::msg::JointTrajectoryPoint curr_state = msg.feedback;
    std::vector<double> start_pos = curr_state.positions;
    _start_pos = Map<VectorXd>(&start_pos[0], start_pos.size()); // Map the position to an Eigen object
    updateState();
}

void GVIMPImpl::gridMapCallback ( grid_map_msgs::msg::GridMap msg )
{
    // Check if a new path is needed and if the current pose is known
    if ( _state == RobotState::CreatePath ) {

        // Find new path
        _new_path_needed = false;
        gpmp2::SignedDistanceField sdf = generateSDF( msg );
        bool vis = this->get_parameter("visualize").as_bool();
        std::string vis_str = vis ? "Visualization mode ON. NOTE: algorithm will take longer to run" : "Visualization mode OFF.";
        RCLCPP_INFO(this->get_logger(), vis_str.c_str());
        std::tuple<VectorXd, SpMat, std::optional<std::vector<VectorXd>>> res = _path_planner->findBestPath( _start_pos, _goal_pos, sdf, vis );
        VectorXd means = std::get<0>(res);
        SpMat covs = std::get<1>(res);

        // Set message data
        _curr_path.waypoints.clear();
        _curr_path.waypoint_size = _path_planner->getParams().nx();
        std::vector<double> waypoint_entropy = extractLogEntropyFromJoint( covs );
        for ( int i=0; i<means.size(); ++i ) {
            _curr_path.waypoints.push_back( means[i] );
        }
        for ( int i=0; i<waypoint_entropy.size(); ++i ) {
            _curr_path.waypoint_entropies.push_back( waypoint_entropy[i] );
        }
        _path_publisher->publish( _curr_path );

        if ( std::get<2>(res).has_value() ) {
            std::vector<VectorXd> iter_path = std::get<2>(res).value();
            _vis_path.waypoints.clear();
            _vis_path.num_iter = _path_planner->getParams().max_iter();
            _vis_path.num_waypoints_per_path = _path_planner->getParams().nt();
            _vis_path.waypoint_size = _path_planner->getParams().nx();
            for ( int i=0; i<iter_path.size(); ++i ) {
                for ( int j=0; j<iter_path[i].size(); ++j )
                _vis_path.waypoints.push_back( iter_path[i][j] );
            }
            _vis_path.env_render = _occupied_cells;
            _iterative_path_publisher->publish( _vis_path );
        }
    }
    else if ( _state == RobotState::ResetArm ) {

        // Reverse the path the robot followed back to start
        _reset_called = false;
        std::vector<double> curr_path = _curr_path.waypoints;
        _curr_path.waypoints.clear();
        for ( int i=curr_path.size()-_path_planner->getParams().nx(); i>=0; i-=_path_planner->getParams().nx() ) {
            for ( int j=0; j<_path_planner->getParams().nx(); ++j ) {
                _curr_path.waypoints.push_back( curr_path[i+j] );
            }
        }
        _path_publisher->publish( _curr_path );
        _curr_path.waypoints.clear();
    }
    updateState();
}

void GVIMPImpl::cellsCallback ( visualization_msgs::msg::MarkerArray msg )
{
    std::vector<geometry_msgs::msg::Point> points;
    for ( auto it : msg.markers ) {
        if ( it.points.size() > 0 ) {
            points.insert( points.end(), it.points.begin(), it.points.end() );
        }
    }
    _occupied_cells = points;
}

void GVIMPImpl::handleParameterRequest ( const std::shared_ptr<motion_planning_msgs::srv::RuntimeParameterInterface::Request> request,
                                        std::shared_ptr<motion_planning_msgs::srv::RuntimeParameterInterface::Response> response )
{   
    std::string success_msg = "Received: {";
    if ( request->total_time > 0.0 ) {
        _path_planner->setTotalTime( request->total_time );
        success_msg.append(" total_time ");
    }
    if ( request->n_states > 0 ) {
        // _path_planner->getParams().set_nt( request->n_states );
        // success_msg.append(" n_states ");
    }
    if ( request->coeff_qc > 0.0 ) {
        // _path_planner->getParams().set_coeff_Qc( request->coeff_qc );
        // success_msg.append(" coeff_Qc ");
    }
    if ( request->sig_obs > 0.0 ) {
        _path_planner->setSigObs( request->sig_obs );
        success_msg.append(" sig_obs ");
    }
    if ( request->eps_sdf > 0.0 ) {
        // _path_planner->getParams().update_eps_sdf( request->eps_sdf );
        // success_msg.append(" eps_sdf ");
    }
    if ( request->radius > 0.0 ) {
        // _path_planner->getParams().update_radius( request->radius );
        // success_msg.append(" radius ");
    }
    if ( request->step_size > 0.0 ) {
        _path_planner->setStepSize( request->step_size );
        success_msg.append(" step_size ");
    }
    if ( request->init_precision_factor > 0.0 ) {
        _path_planner->setInitPrecisionFactor( request->init_precision_factor );
        success_msg.append(" init_precision_factor ");
    }
    if ( request->boundary_penalties > 0.0 ) {
        _path_planner->setBoundaryPenalties( request->boundary_penalties );
        success_msg.append(" boundary_penalties ");
    }
    if ( request->temperature > 0.0 ) {
        _path_planner->setTemperature( request->temperature );
        success_msg.append(" temperature ");
    }
    if ( request->high_temperature > 0.0 ) {
        _path_planner->setHighTemperature( request->high_temperature );
        success_msg.append(" high_temperature ");
    }
    if ( request->low_temp_iterations > 0 ) {
        // _path_planner->getParams().update_lowtemp_iter( request->low_temp_iterations );
        // success_msg.append(" low_temp_iterations ");
    }
    if ( request->stop_err > 0.0 ) {
        // _path_planner->getParams().update_stop_err( request->stop_err );
        // success_msg.append(" stop_err ");
    }
    if ( request->num_iter > 0 ) {
        _path_planner->setNumIter( request->num_iter );
        success_msg.append(" num_iter ");
    }
    if ( request->max_n_backtracking > 0 ) {
        // _path_planner->getParams().update_max_n_backtracking( request->max_n_backtracking );
        // success_msg.append(" max_n_backtracking ");
    }
    if ( request->gh_deg > 0 ) {
        // _path_planner->getParams().update_GH_deg( request->gh_deg );
        // success_msg.append(" gh_deg ");
    }
    if ( request->nx > 0 ) {
        // _path_planner->getParams().update_nx( request->nx );
        // success_msg.append(" nx ");
    }
    if ( request->nu > 0 ) {
        // _path_planner->getParams().update_nu( request->nu );
        // success_msg.append(" nu ");
    }

    response->success = true;
    auto now = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    response->message = success_msg + "} at time " + std::to_string(time);
    updateState();
}

void GVIMPImpl::handlePathRequest ( const std::shared_ptr<motion_planning_msgs::srv::RuntimePathInterface::Request> request,
                                        std::shared_ptr<motion_planning_msgs::srv::RuntimePathInterface::Response> response )
{
    if ( request->set_goal.compare("") != 0 ) {
        std::vector<double> goal_vec;
        Poco::StringTokenizer goal_str( request->set_goal, " " );
        for ( Poco::StringTokenizer::Iterator it = goal_str.begin(); it != goal_str.end(); ++it ) {
            goal_vec.push_back( Poco::NumberParser::parseFloat(*it) );
        }
        if ( goal_vec.size() == _path_planner->getRobotSDF().nlinks() ) {
            _goal_pos = Map<VectorXd, Unaligned>( goal_vec.data(), goal_vec.size() );
        }
    }
    if ( request->run_sim && !request->reset_path ) {
        _new_path_needed = true;
        response->success = true;
        response->message = "Processed request to run simulation";
    }
    else if ( request->reset_path && !request->run_sim ) {
        _reset_called = true;
        response->success = true;
        response->message = "Processed request to reset simulation";
    }
    else {
        response->success = false;
        response->message = "[error] Make sure only one command is called";
    }
    updateState();
}

gpmp2::SignedDistanceField GVIMPImpl::generateSDF ( grid_map_msgs::msg::GridMap msg )
{
    // Read in message data to a grid_map object
    std::string elevationLayer = msg.layers[0];
    grid_map::GridMap gridMap;
    std::vector<std::string> layers{ elevationLayer };
    grid_map::GridMapRosConverter::fromMessage( msg, gridMap, layers, false, false );
    auto& gm_data = gridMap.get( elevationLayer );

    // Replace NaNs with min value of map data
    if ( gm_data.hasNaN() ) {
        const float minMapVal{ gm_data.minCoeffOfFinites() };
        gm_data = gm_data.unaryExpr( [=](float v) { return std::isfinite(v)? v : minMapVal; } );
    }
    const float heightMargin = 0.1;
    const float minVal = gm_data.minCoeffOfFinites() - heightMargin;
    const float maxVal = gm_data.maxCoeffOfFinites() + heightMargin;
    grid_map::SignedDistanceField gm_sdf( gridMap, elevationLayer, minVal, maxVal );

    // Get SDF parameters
    double cell_size = gridMap.getResolution();
    grid_map::Size grid_size = gridMap.getSize();
    grid_map::Length len = gridMap.getLength();
    grid_map::Position pos = gridMap.getPosition();

    gtsam::Point3 origin( pos.x() - len.x()/2.0, pos.y() - len.y()/2.0, minVal );

    // Set up data parsing to a gpmp2 SDF
    int row = 0; int col = 0; int z = 0;
    int y_end = grid_size.y()-1;
    int x_end = grid_size.x()-1;
    std::vector<gtsam::Matrix> gpmp2_data;

    // Since grid_map stores 3D data in a single eigen matrix, we can't just cast it to a gtsam matrix
    // We can use their grid_map_sdf's iteration algorithm to extract the data
    gm_sdf.filterPoints( [&](const grid_map::Position3&, float sdfValue, const grid_map::SignedDistanceField::Derivative3) {
        if ( row == 0 && col == 0 ) {
            gtsam::Matrix temp = gtsam::Matrix::Zero( grid_size.y(), grid_size.x() );
            gpmp2_data.push_back( temp );
        }
        gpmp2_data[z](y_end-row, x_end-col) = static_cast<double>( sdfValue );
        ++col;
        if ( col == grid_size.x() ) {
            col = 0;
            ++row;
        }
        if ( row == grid_size.y() ) {
            row = 0;
            ++z;
        }
    } );
    
    // Parse grid_map_sdf into gpmp2_sdf
    gpmp2::SignedDistanceField gpmp2_sdf( origin, cell_size, gpmp2_data );

    return gpmp2_sdf;
}

void GVIMPImpl::updateState()
{
    double eps = 0.01;
    switch ( _state ) {
        case RobotState::Idle:
            if ( _reset_called ) {
                if ( _curr_path.waypoints.size() > 0 ){
                    _state = RobotState::ResetArm;
                }
            }
            if ( _new_path_needed && _start_pos.size() > 0 ) {
                _state = RobotState::CreatePath;
            }
            break;
        case RobotState::CreatePath:
            if ( !_new_path_needed ) {
                _state = RobotState::ExecutePath;
            }
            break;
        case RobotState::ExecutePath:
            if ( (_goal_pos - _start_pos).norm() <= eps ) {
                _state = RobotState::Idle;
            }
            break;
        case RobotState::ResetArm:
            if ( (_goal_pos - _start_pos).norm() <= eps ) {
                _state = RobotState::Idle;
            }
            break;
        default:
            _state = RobotState::Idle;
    }
}

std::vector<double> GVIMPImpl::extractLogEntropyFromJoint ( SpMat joint_precision )
{
    int nt = _path_planner->getParams().nt();
    int nx = _path_planner->getParams().nx();
    std::vector<double> log_entropy( nt );
    for ( int i=0; i<nt; ++i ) {
        SpMat fact_prec = joint_precision.middleRows(i, _path_planner->getParams().nx()).middleCols(i, _path_planner->getParams().nx());
        SparseLDLT ldlt(fact_prec);
        log_entropy[i] = log(ldlt.determinant());
    }

    return log_entropy;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GVIMPImpl>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}