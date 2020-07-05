/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

//这个Converter类提供了一系列的转换操作
class Converter
{
public:
    //把描述子矩阵的每一行，保存在vector中。cv::Mat->cv::Mat(0)+cv::Mat(1)+...
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    //将cv::Mat形式的位姿转换成李代数形式SE3Quat
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    //将以g2o::Sim3格式存储的位姿转换成为g2o::SE3Quat类型
    //为啥没有这个函数的定义呀？？
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    //将以g2o::SE3Quat格式存储的位姿转换成为cv::Mat格式
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    //把Sim3转换成cv::Mat
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    //把4x4的Eigen转换成cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    //把3x3的Eigen（旋转矩阵）转换成cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    //把3x1的Eigen（平移向量）转换成cv::Mat
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    //将给定的旋转矩阵和平移向量转换为以cv::Mat存储的李群SE3
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    //将cv::Mat类型数据转换成为3x1的Eigen矩阵
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    //将cv::Point3f转换成为Eigen中3x1的矩阵
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    //将一个3x3的cv::Mat矩阵转换成为Eigen中的矩阵
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    //把旋转矩阵M转换成四元数
    static std::vector<float> toQuaternion(const cv::Mat &M);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
