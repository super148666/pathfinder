//
// Created by chaoz on 2/08/18.
//

#include <deque>
#include "pathfinder/range_1d.h"
#include <cmath>
#include <iostream>

/**
 * range_1d::operator[]
 *
 * @param angle - desired angle
 * @return the laserscan reading at desired angle
 */
double &range_1d::operator[](double angle) {
    assert(angle <= max_angle && angle >= min_angle);
    return ranges[(int) ((angle - min_angle) / resolution)];
}

/**
 * range_1d::range_1d - contrustor
 *
 * @param ls_ptr - ros msg type pointer for laserscan
 */
range_1d::range_1d(LaserScan *ls_ptr) {
    max_range = ls_ptr->range_max;
    resolution = ls_ptr->angle_increment;
    if (resolution > 0) {
        max_angle = ls_ptr->angle_max * M_1_PI * 180.0;
        min_angle = ls_ptr->angle_min * M_1_PI * 180.0;
        int num_data = (int) ((max_angle - min_angle) / resolution) + 1;
        ranges.reserve(num_data);
        for (int i = 0; i < num_data; i++) {
            ranges.push_back(ls_ptr->ranges[i]);
        }

    } else if (resolution < 0) {
        resolution = -resolution;
        max_angle = ls_ptr->angle_min * M_1_PI * 180.0;
        min_angle = ls_ptr->angle_max * M_1_PI * 180.0;
        int num_data = (int) ((max_angle - min_angle) / resolution) + 1;
        ranges.reserve(num_data);
        for (int i = num_data - 1; i >= 0; i--) {
            ranges.push_back(ls_ptr->ranges[i]);
        }
    } else {
        throw std::runtime_error("failed to construct: resolution can not be zero!");
    }
}

/**
 * range_1d::range_1d - contrustor
 *
 * @param ls_ptr - ros msg type pointer for laserscan
 * @param desired_angle_max - maximum constrains in angle
 * @param desired_angle_min - minimum constrains in angle
 */
range_1d::range_1d(LaserScan *ls_ptr, double desired_angle_max, double desired_angle_min, double processing_range) {
    desired_angle_max = desired_angle_max / 180.0 *M_PI;
    desired_angle_min = desired_angle_min / 180.0 *M_PI;
    max_range = ls_ptr->range_max;
    resolution = ls_ptr->angle_increment;

    if (resolution > 0) {
        max_angle = ls_ptr->angle_max;
        min_angle = ls_ptr->angle_min;

//        std::cout << "max angle: " << max_angle << "\n" <<
//                     "desired angle max: " << desired_angle_max << "\n" <<
//                     "min angle: " << min_angle << "\n" <<
//                     "desired angle min: " << desired_angle_min << "\n" <<
//                     std::endl;

        assert(max_angle >= desired_angle_max);
        assert(min_angle <= desired_angle_min);

        int begin_index = (int) ((desired_angle_min - min_angle) / resolution);
        int end_index = (int) ((desired_angle_max - min_angle) / resolution) + 1;
        size_t num_points = end_index - begin_index;
        ranges.reserve(num_points);

        for (int i = begin_index; i != end_index; i++) {
            ranges.push_back(ls_ptr->ranges[i]);
            if (ranges[i] > processing_range) {
//                std::cout << "out side of processing range\n";
                ranges[i] = processing_range;
            }
//            std::cout << "ranges[" << i << "]: " << ranges.back() << "\n";
        }
        max_angle = desired_angle_max;
        min_angle = desired_angle_min;
    } else if (resolution < 0) {
        resolution = -resolution;
        max_angle = ls_ptr->angle_min;
        min_angle = ls_ptr->angle_max;
        assert(max_angle > desired_angle_max);
        assert(min_angle < desired_angle_min);
        int rbegin_index = (int) ((max_angle - desired_angle_min) / resolution);
        int rend_index = (int) ((max_angle - desired_angle_max) / resolution) - 1;

        size_t num_points = rbegin_index - rend_index;
        ranges.reserve(num_points);

        for (int i = rbegin_index; i > rend_index; i--) {
            ranges.push_back(ls_ptr->ranges[i]);
            if(ranges[i] > processing_range) {
//                std::cout << "out side of processing range\n";
                ranges[i] = processing_range;
            }
//            std::cout << "ranges[" << i << "]: " << ranges.back() << "\n";
        }

        max_angle = desired_angle_max;
        min_angle = desired_angle_min;
    } else {
        throw std::runtime_error("failed to construct: resolution can not be zero!");
    }
}

const std::vector<double> &range_1d::getRanges() const {
    return ranges;
}

double range_1d::get_max_range() const {
    return max_range;
}

double range_1d::get_min_angle() const {
    return min_angle;
}

double range_1d::get_max_angle() const {
    return max_angle;
}

double range_1d::get_resolution() const {
    return resolution;
}

void range_1d::set_ranges(const std::vector<double> &ranges) {
    range_1d::ranges = ranges;
}

void range_1d::set_max_range(double max_range) {
    range_1d::max_range = max_range;
}

void range_1d::set_min_angle(double min_angle) {
    range_1d::min_angle = min_angle;
}

void range_1d::set_max_angle(double max_angle) {
    range_1d::max_angle = max_angle;
}

void range_1d::set_resolution(double resolution) {
    range_1d::resolution = resolution;
}

void range_1d::median_filter(int size) {
    std::deque<double> queue1;
    std::deque<double> queue2;
    queue1.resize(size,ranges.front());

    if (size == 0) return;


    for(auto iter = ranges.begin(); iter != ranges.end(); iter++) {
        queue1.erase(queue1.begin());
        queue1.push_back(*iter);
        queue2 = queue1;
        std::sort(queue2.begin(),queue2.end());
        if (size % 2 == 0) {
            *iter = (queue1[size / 2 - 1] + queue1[size / 2] ) / 2.0;
        }
        else {
            *iter = queue1[size/2];
        }
    }

}
