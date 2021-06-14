/*
 * Created on Mon Jan 06 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "bal_parser.h"

using namespace std;

BALParser::BALParser(const string& data_file_path)
{
    FILE* fp = fopen(data_file_path.c_str(), "r");
    if(fp == NULL)
    {
        std::cerr << "Error: unable to open file " << data_file_path;
        return;
    }

    fscanf(fp, "%d", &num_cameras_);
    fscanf(fp, "%d", &num_points_);
    fscanf(fp, "%d", &num_observation_);
    observations_.resize(num_observation_);

    for(int i = 0; i < num_observation_; i++)
    {
        int c_id, pt_id;
        double pt_x, pt_y;
        fscanf(fp, "%d", &c_id);
        fscanf(fp, "%d", &pt_id);
        fscanf(fp, "%lf", &pt_x);
        fscanf(fp, "%lf", &pt_y);

        observation ob(c_id, pt_id, pt_x, pt_y);
        observations_[i] = ob;
    }

}

void BALParser::Perturb(const double rotation_sigma,
                         const double translation_sigma,
                         const double point_sigma) {

    double *points = mutable_points();
    if (point_sigma > 0) {
        for (int i = 0; i < num_points_; ++i) {
            PerturbPoint3(point_sigma, points + 3 * i);
        }
    }

    for (int i = 0; i < num_cameras_; ++i) {
        double *camera = mutable_cameras() + camera_block_size() * i;

        double angle_axis[3];
        double center[3];
        // Perturb in the rotation of the camera in the angle-axis
        // representation
        CameraToAngelAxisAndCenter(camera, angle_axis, center);
        if (rotation_sigma > 0.0) {
            PerturbPoint3(rotation_sigma, angle_axis);
        }
        AngleAxisAndCenterToCamera(angle_axis, center, camera);

        if (translation_sigma > 0.0)
            PerturbPoint3(translation_sigma, camera + camera_block_size() - 6);
    }
}
