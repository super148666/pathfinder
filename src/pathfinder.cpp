//
// Created by chaoz on 26/07/18.
//

#include <pathfinder/pathfinder.h>
#include <functional>
#include <algorithm>
#include <cerrno>
#include <iostream>
#include <chrono>
#include <thread>
//extern "C"{
//#include <eyebot.h>
//}

#define INVALID_STEERING 9999.0
#define DEFAULT_SPIN_RATE 15
#define DEFAULT_CLUSTERING_THRESHOLD 0.2
#define DEFAULT_NUM_OF_PATH_POINTS 5
#define DEFAULT_TIME_INTERVAL (1.0/DEFAULT_NUM_OF_PATH_POINTS)
#define DEFAULT_MAX_STEERING 30
#define DEFAULT_PROCESSING_RANGE 4.5
#define DEFAULT_CLEARANCE_RADIUS 0.8
#define DEFAULT_APPLY_MEDIAN_FILTER false
#define DEFAULT_FILTER_SIZE 3
#define DEFAULT_DESIRED_SPEED 10
#define DEFAULT_DIST_FRONT_TO_REAR 2.5

/**
 * pathfinder::pathfinder - constructor
 *
 * @param n - ros node handle
 */
pathfinder::pathfinder() : spin_rate(15) {

    cur_rotate_velocity = 0.0;
    tar_linear_velocity = 0.0;
    tar_rotate_velocity = 0.0;
    cur_linear_velocity = 0.0;
//    applied_speed = 0.0;
    cur_x = 0.0;
    cur_y = 0.0;
    orientation = 0.0;
    has_new_cone = false;
    has_new_laserscan = false;

    VWSetPosition(0, 0, 0);
    VWSetSpeed(0, 0);
    LIDARSet(180, 0, 360);
}

/**
 * pathfinder::Config
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::Config(int argc, char **argv) {
    bool status = true;
    status &= read_parameters(argc, argv);

    return status;
}

/**
 * pathfinder::PrintStatus
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::PrintStatus() {
    static std::string s_tab = "    ";
    std::cout << "\npose:\n" << s_tab << "x:\n" << s_tab + s_tab << cur_x <<
              s_tab << "y:\n" << s_tab + s_tab << cur_y <<
              s_tab << "yaw:\n" << s_tab + s_tab << orientation <<
              "\nvelocity:\n" << s_tab << "current:\n" << s_tab + s_tab << "linear:\n"
              << s_tab + s_tab + s_tab << cur_linear_velocity <<
                                s_tab + s_tab << "angular:\n" << s_tab + s_tab + s_tab << cur_rotate_velocity <<
              s_tab << "target:\n" << s_tab + s_tab << "linear:\n" << s_tab + s_tab + s_tab
              << tar_linear_velocity <<
                                s_tab + s_tab << "angular:\n" << s_tab + s_tab + s_tab << tar_rotate_velocity <<
              "\ncones:\n" << s_tab << "number:\n" << s_tab + s_tab << cones.size() << '\n';
    return true;
}

/**
 * pathfinder::Start
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::Start() {
    // follow the path defined by cones until no more valid track is detected.
    bool status = true;
    int count = 0;
    while (status) {
        // attempt to read newest message
//        std::cout << "start odom update\n";
        odom_update();
//        std::cout << "start laser update\n";
        laser_update();

//        std::cout << "check new laserscan\n";
        // if laserscan is used, find cones
        if (has_new_laserscan) {
//            std::cout << "has new scan\n";
            has_new_laserscan = false;
            if (find_cones()) {
                std::cout << "has new cones\n";
                has_new_cone = true;
            }
        }


        // print status every 1 s
//        if (++count == spin_rate) {
//            status &= PrintStatus();
//            count = 0;
//        }

        if (has_new_cone) {
            has_new_cone = false;
//            std::cout << "start drive\n";
            status = drive();

//            std::cout << "clear cones and ranges\n";
            cones.clear();
            valid_ranges.clear();
        }

//        std::cout << "sleep\n";
        sleep();
    }

    return status;
}

/**
 * pathfinder::read_parameters
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::read_parameters(int argc, char **argv) {
    bool status = true;

    spin_rate = DEFAULT_SPIN_RATE;
    clustering_threshold = DEFAULT_CLUSTERING_THRESHOLD;
    num_of_path_points = DEFAULT_NUM_OF_PATH_POINTS;
    time_interval = DEFAULT_TIME_INTERVAL;
    max_steering = DEFAULT_MAX_STEERING / 180.0 * M_PI;
    processing_range = DEFAULT_PROCESSING_RANGE;
    clearance_radius = DEFAULT_CLEARANCE_RADIUS;
    apply_median_filter = DEFAULT_APPLY_MEDIAN_FILTER;
    filter_size = DEFAULT_FILTER_SIZE;
    desired_speed = DEFAULT_DESIRED_SPEED;
    dist_front_to_rear = DEFAULT_DIST_FRONT_TO_REAR;

    for (int i = 0; i < argc; i++) {
        if (i == 0 && strncmp(argv[i], "./", 2)==0) {
            continue;
        } else if (!strcmp(argv[i], "--spin_rate")) {
            i++;
            spin_rate = std::stoi(argv[i]);
        } else if (!strcmp(argv[i], "--clustering_threshold")) {
            i++;
            clustering_threshold = std::stod(argv[i]);
        } else if (!strcmp(argv[i], "--num_of_path_points")) {
            i++;
            num_of_path_points = std::stoi(argv[i]);
        } else if (!strcmp(argv[i], "--time_interval")) {
            i++;
            time_interval = std::stod(argv[i]);
        } else if (!strcmp(argv[i], "--max_steering")) {
            i++;
            max_steering = std::stod(argv[i]) / 180.0 * M_PI;
        } else if (!strcmp(argv[i], "--processing_range")) {
            i++;
            processing_range = std::stod(argv[i]);
        } else if (!strcmp(argv[i], "--clearance_radius")) {
            i++;
            clearance_radius = std::stod(argv[i]);
        } else if (!strcmp(argv[i], "--apply_median_filter")) {
            i++;
            if (!strcmp(argv[i], "false")) {
                apply_median_filter = false;
            } else if (!strcmp(argv[i], "true")) {
                apply_median_filter = true;
            }
        } else if (!strcmp(argv[i], "--filter_size")) {
            i++;
            filter_size = std::stoi(argv[i]);
        } else if (!strcmp(argv[i], "--desired_speed")) {
            i++;
            desired_speed = std::stod(argv[i]);
        } else if (!strcmp(argv[i], "--distance_between_front_rear_wheels")) {
            i++;
            dist_front_to_rear = std::stod(argv[i]);
        } else {
            std::cout << "Invalid input: " << argv[i] << "\n";
            std::cout << "Usage: ./demo <command> <value>\n" <<
                      "Available Commands:\n" <<
                      "    --spin_rate <int>    @define the spin rate of the program, default as " << DEFAULT_SPIN_RATE << ".\n" <<
                      "    --clustering_threshold <double>    @constant used for cone clustering, default as " << DEFAULT_CLUSTERING_THRESHOLD << ".\n" <<
                      "    --num_of_path_points <int>    @num of points for path visualisation, default as " << DEFAULT_NUM_OF_PATH_POINTS << ".\n" <<
                      "    --time_interval <double>    @time interval between each path point, default as " << DEFAULT_TIME_INTERVAL << ".\n" <<
                      "    --max_steering <double>    @max steering in degree (positive), default as " << DEFAULT_MAX_STEERING << ".\n" <<
                      "    --processing_range <double>    @in meter (drop all data outside this range), default as " << DEFAULT_PROCESSING_RANGE << ".\n" <<
                      "    --clearance_radius <double>    @in meter (collision free boundary), default as " << DEFAULT_CLEARANCE_RADIUS << ".\n" <<
                      "    --apply_median_filter <bool>    @whether to use median filter upon laserscan, default as " << (DEFAULT_APPLY_MEDIAN_FILTER?"true":"false") << ".\n" <<
                      "    --filter_size <int>    @filter length for median filter, default as " << DEFAULT_FILTER_SIZE << ".\n" <<
                      "    --desired_speed <double>    @target speed in m/s, default as " << DEFAULT_DESIRED_SPEED << ".\n" <<
                      "    --distance_between_front_rear_wheels <double>    @distance between front rear wheels in m, default as " << DEFAULT_DIST_FRONT_TO_REAR << ".\n";
            exit(-1);
        }
    }

    initial_range = steering_range(-2.0 * max_steering, 2.0 * max_steering, dist_front_to_rear);
    std::cout << "Configurations:\n" <<
              "spin rate: " << spin_rate << "\n" <<
              "clustering threshold: " << clustering_threshold << "\n" <<
              "num of path points: " << num_of_path_points << "\n" <<
              "time interval: " << time_interval << "\n" <<
              "max steering: " << max_steering << "\n" <<
              "processing range: " << processing_range << "\n" <<
              "clearance radius: " << clearance_radius << "\n" <<
              "apply median filter: " << apply_median_filter << "\n" <<
              "filter size: " << filter_size << "\n" <<
              "desired speed: " << desired_speed << "\n" <<
              std::endl;
    return status;
}

/**
 * pathfinder::odom_cb
 *
 * @param msg - nav_msgs::OdometryConstPtr
 */
void pathfinder::odom_update() {
    int x, y, phi, tspd, rspd;
    VWGetPosition(&x, &y, &phi);
    VWGetSpeed(&tspd, &rspd);

    // get x & y
    cur_x = x / 1000.0;
    cur_y = y / 1000.0;

    // get orientation
    orientation = phi / 100.0;

    // get velocity
    cur_linear_velocity = tspd / 1000.0;
    cur_rotate_velocity = rspd / 100.0;
}

/**
 * pathfinder::laser_cb
 *
 * @param msg - sensor_msgs::LaserScanConstPtr msg
 */
void pathfinder::laser_update() {
    int scan[360] = {0};
    LIDARGet(scan);
    LaserScan *msg = new LaserScan();

    msg->angle_increment = 0.5 / 180.0 * M_PI;
    msg->angle_max = 90.0 / 180.0 * M_PI;
    msg->angle_min = -90.0 / 180.0 * M_PI;
    msg->range_max = 20.0;

    for (int i = 0; i < 360; i++) {
        msg->ranges.push_back(scan[i] / 1000.0);
//        std::cout << " scan[" << i << "]: " << msg->ranges.back() << "\n";
    }

    // load ros sensor msg into range_1d
    has_new_laserscan = true;
    try {
        laserscan = range_1d(msg, 90.0, -90.0, processing_range);
    } catch (const std::exception &e) {
        std::cerr << e.what();
        exit(-1);
    }

    if(apply_median_filter) {
        laserscan.median_filter(filter_size);
    }
}

bool pathfinder::drive() {
	
    // init range list
//    std::cout << "init valid ranges\n";
    valid_ranges.clear();
    valid_ranges.push_back(initial_range);
    valid_cones.clear();
    bool stop_ = false;
    int count = 0;
//    std::cout << "start iterate through the cones and identify the collision free steering ranges\n";
    // iterate through the cones and identify the collision free steering ranges
    for (auto cones_iter = cones.begin(); cones_iter != cones.end(); cones_iter++) {
        // check if the cone is outside of the processing range
        double dist_to_car = cones_iter->get_dist_to_point(point_2d(0.0,0.0));
//        std::cout << "cone " << count++ << " dist to car: " << dist_to_car << "\n";
        if (dist_to_car < processing_range) {
//            std::cout << "inside of processing range\n";
            valid_cones.push_back(*cones_iter);
            // if the cone is inside the processing range
            // calculate the boundary of collision steering at given cone location
            // check existence of path
//            std::cout << "start update steering ranges for this cone\n";
            if (!update_steering_ranges(*cones_iter)) {
                std::cout << "no available path\n";
                // no available path
                // stop and return false
                stop_ = true;
                break;
            }
        }

    }
    
    if(stop_) {
        std::cout << "stop due to no available path\n";
        stop();
        return false;
	}


//    std::cout << "select the largest steering range (prefer as small change as possible to current steering)\n";
    // select the largest steering range (prefer as small change as possible to current steering)
    auto desired_range_iter = valid_ranges.begin();
    double max_size = 0;
    double min_change = 999999999;
    count = 0;
    std::cout << "dynamic limit: [" << -max_steering << ',' << max_steering << "]\n";
    bool has_valid_range = false;
    for (auto range_iter = valid_ranges.begin(); range_iter != valid_ranges.end(); range_iter++) {
        std::cout << "range[" << count++ << "]: [" << range_iter->get_min() << ',' << range_iter->get_max() << "]\n";
        if(range_iter->get_max() < -max_steering) {
            continue;
        }
        if(range_iter->get_min() > max_steering) {
            continue;
        }
        double this_size = range_iter->get_size();
        // check size first
        if (this_size > max_size) {
            // update with larger range
            max_size = this_size;
            desired_range_iter = range_iter;
            has_valid_range = true;
            min_change = fabs(range_iter->get_mean() - tar_steering_angle);
        } else if (this_size == max_size) {
            // if range is same
            double this_change = fabs(range_iter->get_mean() - tar_steering_angle);
            // check change in steering
            if (this_change < min_change) {
                // update with smaller change option
                min_change = this_change;
                desired_range_iter = range_iter;
                has_valid_range = true;
            }
        }
    }

    if (!has_valid_range) {
        std::cout << "no valid ranges\n";
        stop();
        return false;
    }

    std::cout << "desired range: [" << desired_range_iter->get_min() << ',' << desired_range_iter->get_max() << "]\n";
    double steering_angle = desired_range_iter->get_mean();
//    std::cout << "get initial steering angle decision: " << steering_angle << "\n";
    if(steering_angle < -max_steering) {
		if(desired_range_iter->get_max() < -max_steering) {
            std::cout << "the steering angle is outside of max steering\n";
            std::cout << "stop\n";
			stop();
			return false;
		}
        std::cout << "this steering angle is outside of max steering\n";
        std::cout << "but steering rang is still fall inside max steering range\n";
        steering_angle = -max_steering;
        std::cout << "update steering angle to max: " << steering_angle << "\n";
    }
    if(steering_angle > max_steering) {
        if(desired_range_iter->get_min() > max_steering) {
            std::cout << "the steering angle is outside of max steering\n";
            std::cout << "stop\n";
            stop();
			return false;
		}
        std::cout << "this steering angle is outside of max steering\n";
        std::cout << "but steering rang is still fall inside max steering range\n";
        steering_angle = max_steering;
        std::cout << "update steering angle to max: " << steering_angle << "\n";
    }
    // drive using mean steering for that range
    std::cout << "start curve based on steering angle: " << steering_angle << "\n";
    curve(-steering_angle);
    return true;
}

/**
 * pathfinder::stop
 *
 */
void pathfinder::stop() {
    // stop
    int tspd = 0;
    int rspd = 0;
    tar_linear_velocity = 0.0;
    std::cout << "linear velocity: " << tspd << "\n";
    std::cout << "rotational velocity: " << rspd << "\n";
    VWSetSpeed(tspd, rspd);
}

/**
 * pathfinder::curve
 *
 * @param angle - steering angle
 */
void pathfinder::curve(double angle) {
    // curve
    tar_linear_velocity = desired_speed;
    tar_rotate_velocity = tar_linear_velocity * sin(tar_steering_angle) / dist_front_to_rear;
    int tspd = (int) (tar_linear_velocity * 1000);
    int rspd = 0;
    if (desired_speed == 0.0) {
        rspd = 0;
        tar_steering_angle = 0;
    } else {
        rspd = (int) (tar_rotate_velocity * 100);
        tar_steering_angle = angle;
    }
    std::cout << "linear velocity: " << tspd << "\n";
    std::cout << "rotational velocity: " << rspd << "\n";
    VWSetSpeed(tspd, rspd);
}

/**
 * pathfinder::find_cones
 *
 * @return true if success; false otherwise.
 */
bool pathfinder::find_cones() {

    // clustering
    std::vector<range_1d_cluster> clusters;
    bool new_cluster = true;
    range_1d_cluster cluster;
    for (double current_angle = laserscan.get_min_angle();
         current_angle <= laserscan.get_max_angle();) {
        if (new_cluster) {
            cluster = range_1d_cluster(current_angle, laserscan[current_angle]);
            new_cluster = false;
        } else {
            if (!cluster.push(current_angle, laserscan[current_angle], clustering_threshold)) {
                if (cluster.getClosest_distance() >0.01) {//cluster.size() < clustering_threshold && cluster.size() > 0.05) {
                    clusters.push_back(cluster);
                }
                cluster = range_1d_cluster(current_angle, laserscan[current_angle]);
            }
        }
        current_angle += laserscan.get_resolution();
    }

    // put clusters into point_2d vector
    cones.clear();
    int count = 0;
    for (auto iter = clusters.begin(); iter != clusters.end(); iter++) {
        cones.emplace_back((*iter).get_x(), (*iter).get_y(), (*iter).size()/2.0);
        std::cout << "cone[" << count++ << "]: (" << cones.back().get_x() << "," << cones.back().get_y() << ")\n";
        std::cout << "cone[" << count++ << "]: dist to car is " << cones.back().get_dist_to_car() << "\n";
    }

    return !cones.empty();
}

bool pathfinder::update_steering_ranges(cone_2d &cone) {
    double min_steering = evaluate_boundary_steering(cone.get_x(), cone.get_y(), -clearance_radius-cone.get_half_diameter());
    double max_steering = evaluate_boundary_steering(cone.get_x(), cone.get_y(), clearance_radius+cone.get_half_diameter());
    
    if (min_steering == INVALID_STEERING && max_steering == INVALID_STEERING) {
        return false;
    }
    else if(min_steering == INVALID_STEERING) {
        for (auto ranges_iter = valid_ranges.begin(); ranges_iter != valid_ranges.end();) {

            // all good
            if (max_steering <= ranges_iter->get_min()) {
                ranges_iter++;
                continue;
            }

            // no common interval
            if (max_steering > ranges_iter->get_max()) {
                ranges_iter = valid_ranges.erase(ranges_iter);
                if (valid_ranges.empty()) {
                    return false;
                }
            }

            // update steering
            if (max_steering <= ranges_iter->get_max()) {
                ranges_iter->set_min(max_steering);
                return true;
            }
        }
    } else if(max_steering == INVALID_STEERING) {
        for (auto ranges_iter = valid_ranges.begin(); ranges_iter != valid_ranges.end();) {

            // all good
            if (min_steering >= ranges_iter->get_min()) {
                ranges_iter++;
                continue;
            }

            // no common interval
            if (min_steering < ranges_iter->get_min()) {
                ranges_iter = valid_ranges.erase(ranges_iter);
                if (valid_ranges.empty()) {
                    return false;
                }
            }

            // update steering
            if (min_steering <= ranges_iter->get_min()) {
                ranges_iter->set_max(min_steering);
                return true;
            }
        }
    }
    else {
    
        for (auto ranges_iter = valid_ranges.begin(); ranges_iter != valid_ranges.end();) {
            // all good
            if (min_steering >= ranges_iter->get_max()) {
                ranges_iter++;
                continue;
            }

            // all good
            if (max_steering <= ranges_iter->get_min()) {
                ranges_iter++;
                continue;
            }

            // no common interval
            if (min_steering < ranges_iter->get_min() && max_steering > ranges_iter->get_max()) {
                ranges_iter = valid_ranges.erase(ranges_iter);
                if (valid_ranges.empty()) {
                    return false;
                }
            }

            // split into two range
            if (min_steering >= ranges_iter->get_min() && max_steering <= ranges_iter->get_max()) {
                steering_range smaller_range(ranges_iter->get_min(), min_steering,
                                             ranges_iter->get_front_rear_distance());
                ranges_iter->set_min(max_steering);
                valid_ranges.insert(ranges_iter, smaller_range);
                return true;
            }

            // update steering
            if (max_steering <= ranges_iter->get_max()) {
                ranges_iter->set_min(max_steering);
            }

            if (min_steering >= ranges_iter->get_min()) {
                ranges_iter->set_max(min_steering);
            }
            ranges_iter++;
        }
    }
    return true;
}

double pathfinder::evaluate_boundary_steering(double x, double y, double shift) {
    double _y = y + shift;

    x = x + DIST_LIDAR_TO_CAR;
    if (_y == 0.0) {
        return 0.0;
    }

    double len = x / sin(2.0 * atan2(x, _y)) - shift;
    if(len == 0.0) {
        return 0.0;
    }
    if(shift > 0.0) {
        if(len < 0.0 && len > -shift) {
            std::cout << " shift:" << shift << ",len:" << len;
            return INVALID_STEERING;
        }
    } else if (shift < 0.0) {
        if(len > 0.0 && len < -shift) {
            std::cout << " shift:" << shift << ",len:" << len;
            return INVALID_STEERING;
        }
    }
    double rad = dist_front_to_rear / len;
    if (rad > 1.0) {
        rad = 1.0;
    }
    else if (rad < -1.0) {
        rad = -1.0;
    }

    return asin(rad);
}

void pathfinder::sleep() {
    static long long int max_time_interval_msec = 1000 / spin_rate;
    static auto last_time = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto this_time = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    auto duration = this_time - last_time;
    auto time_pass_msec = duration.count();
    long long int time_to_sleep_msec;
    if (time_pass_msec > max_time_interval_msec) {
        time_to_sleep_msec = max_time_interval_msec;
    } else {
        time_to_sleep_msec = max_time_interval_msec - time_pass_msec;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(time_to_sleep_msec));
    last_time = this_time;
}

