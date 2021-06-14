/*
 * Created on Mon Jan 06 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */
#ifndef BAL_PARSER_H_
#define BAL_PARSER_H_

#include <iostream>
#include <string>
#include <vector>

class BALParser
{
public:
    BALParser(const std::string& data_file);
    struct observation
    {
        observation(){};

        observation(int c_id, int pt_id, double pt_x, double pt_y)
        :cam_id_(c_id), pt_id_(pt_id), pt_x_(pt_x), pt_y_(pt_y){};
        int cam_id_;
        int pt_id_;
        double pt_x_;
        double pt_y_;
    };
    std::vector<observation>& Observation()
    {
        return observations_;
    };
    void Perturb(const double rotation_sigma,
                         const double translation_sigma,
                         const double point_sigma);
    int NumOfCam() {return num_cameras_;};
    int NumOfPoints() {return num_points_;};
    double* groundtrueth_;
private:
    int num_cameras_;
    int num_points_;
    int num_observation_;

    std::vector<observation> observations_;
};
#endif // 
