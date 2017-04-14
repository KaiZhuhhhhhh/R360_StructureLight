//
// Created by Öì¿­ on 17/4/4.
//

//#pragma once

#ifndef CALIBRATE_ZK_H
#define CALIBRATE_ZK_H

#include <highgui.h> 
#include "cv.h"
#include <iostream> 
#include "opencv2/opencv.hpp"

extern std::vector<CvMat> T_mat_4x4;					//Ðý×ª¾ØÕó

void inputCameraParam(CvMat * intrinsic_matrix1, CvMat * distortion_coeffs1);
void caculate_Tmat(CvMat * r_vec, CvMat * t_vec, CvMat * T_mat);
int find_rotation_mat();

#endif