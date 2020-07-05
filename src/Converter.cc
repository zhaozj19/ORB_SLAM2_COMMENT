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


#include "Converter.h"

namespace ORB_SLAM2
{

////把描述子矩阵的每一行，保存在vector中。cv::Mat->cv::Mat(0)+cv::Mat(1)+...
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    //定义结果向量
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    //一共rows行，每一行都保存在vDesc中
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

//将cv::Mat形式的位姿转换成李代数形式SE3Quat
//SE3Quat的构造函数如下三种形式

//SE3Quat();//_r初始化为单位阵,_t初始化为0
//SE3Quat(const Matrix3D& R, const Vector3D& t):_r(Eigen::Quaterniond(R)),_t(t) 用旋转矩阵R和平移向量t来初始化_r、_t
//SE3Quat(const Eigen::Quaterniond& q, const Vector3D& t):_r(q),_t(t) 用四元数q和平移向量t来初始化_r、_t
g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    //定义旋转矩阵R，并初始化
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    //定义平移向量t，并初始化
    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    //构造李代数就好了
    return g2o::SE3Quat(R,t);
}

//将以g2o::SE3Quat格式存储的位姿转换成为cv::Mat格式
cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    //首先调用SE3的成员函数to_homogeneous_matrix生成4x4的Eigen
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    //然后转换成cv::Mat
    return toCvMat(eigMat);
}

//将仿射矩阵由g2o::Sim3->cv::Mat
cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    //得到仿射矩阵的旋转部分eigR
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    //得到仿射矩阵的平一部分eigt
    Eigen::Vector3d eigt = Sim3.translation();
    //获取仿射矩阵的缩放系数
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

//把4x4的Eigen转换成cv::Mat
cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    //就是一个个赋值
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    //用深拷贝函数返回计算结果
    return cvMat.clone();
}

//把3x3的Eigen（旋转矩阵）转换成cv::Mat
cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

//把3x1的Eigen（平移向量）转换成cv::Mat
cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

//将给定的旋转矩阵和平移向量转换为以cv::Mat存储的李群SE3
cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

//将cv::Mat类型数据转换成为3x1的Eigen矩阵
Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

//将cv::Point3f转换成为Eigen中3x1的矩阵
Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

//将一个3x3的cv::Mat矩阵转换成为Eigen中的矩阵
Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

//把旋转矩阵M转换成四元数（以vector的形式返回）
//这里没有做旋转矩阵的判断，要自己注意
std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    //顺序为xyzw
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

} //namespace ORB_SLAM
