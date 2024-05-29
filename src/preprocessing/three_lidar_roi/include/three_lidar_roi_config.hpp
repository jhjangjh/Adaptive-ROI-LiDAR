#ifndef __THREE_LIDAR_ROI_CONFIG_HPP__
#define __THREE_LIDAR_ROI_CONFIG_HPP__
#pragma once

typedef struct {
    double voxel_size;
    double ransac_threshold;
    int number_of_lanes;
} ThreeLidarRoiParameters;

#endif  // __THREE_LIDAR_ROI_CONFIG_HPP__