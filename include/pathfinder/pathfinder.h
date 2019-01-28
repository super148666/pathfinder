//
// Created by chaoz on 26/07/18.
//

#ifndef PROJECT_PATHFINDER_H
#define PROJECT_PATHFINDER_H

#include <exception>

#include <pathfinder/cone_2d.h>
#include <pathfinder/range_1d_cluster.h>
#include <pathfinder/range_1d.h>
#include <pathfinder/enum_def.h>
#include <pathfinder/steering_range.h>
#include <string>
#include <eyebot++.h>


class pathfinder {
public:
    pathfinder();

    ~pathfinder() = default;

    bool Config(int argc, char **argv);

    bool PrintStatus();

    bool Start();

private:
    bool read_parameters(int argc, char **argv);

	bool drive();

    bool update_steering_ranges(cone_2d& cone);

    double evaluate_boundary_steering(double x, double y, double shift);

    void stop();

    void curve(double angle);

    void odom_update();

    void laser_update();

    bool find_cones();

    void sleep();

    int spin_rate;

    double desired_speed;
//    double applied_speed;
    double cur_x;
    double cur_y;
    double orientation;
    double cur_linear_velocity;
    double cur_rotate_velocity;

    double tar_linear_velocity;
    double tar_rotate_velocity;
    double tar_steering_angle;
    
    double fix_driving_speed;

    double dist_front_to_rear;

    std::list<cone_2d> cones;
    std::vector<cone_2d> valid_cones;

    steering_range initial_range;
    std::list<steering_range> valid_ranges;
    double clearance_radius;
    double max_steering;

    bool apply_median_filter;
    int filter_size;
    bool has_new_cone;
    bool has_new_laserscan;
    range_1d laserscan;
    double clustering_threshold;

	std::vector<double> r_meter;
	std::vector<double> angle_degree;

    int num_of_path_points;
    double time_interval;

    double processing_range;

    std::string frame_id;
};

#endif //PROJECT_PATHFINDER_H
