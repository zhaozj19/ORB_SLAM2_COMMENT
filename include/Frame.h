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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
//定义一帧中有多少个网格，48行、64列
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    //默认的构造函数
    Frame();

    // Copy constructor.
    //拷贝构造函数
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    //双目相机的帧构造函数
    Frame(  const cv::Mat &imLeft,          //左图灰度图
            const cv::Mat &imRight,         //右图灰度图
            const double &timeStamp,        //时间戳
            ORBextractor* extractorLeft,    //左图灰度图ORB特征点提取器
            ORBextractor* extractorRight,   //右图灰度图ORB特征点提取器
            ORBVocabulary* voc,             //ORB字典
            cv::Mat &K,                     //双目相机内参
            cv::Mat &distCoef,              //双目相机去畸变参数
            const float &bf,                //双目相机基线和焦距(fx)的成绩
            const float &thDepth);          //远点和近点的阈值区分（什么叫远点和近点？？？）

    // Constructor for RGB-D cameras.
    //RGBD相机的帧构造函数
    Frame(  const cv::Mat &imGray,          //灰度图
            const cv::Mat &imDepth,         //灰度图对应的深度图
            const double &timeStamp,        //时间戳
            ORBextractor* extractor,        //灰度图ORB特征点提取器
            ORBVocabulary* voc,             //ORB字典
            cv::Mat &K,                     //RGBD相机的内参
            cv::Mat &distCoef,              //RGBD相机的去畸变参数
            const float &bf,                //RGBD相机应该用不到这个参数吧
            const float &thDepth);          //远点和近点的阈值区分

    // Constructor for Monocular cameras.
    //单目相机的帧构造函数
    Frame(  const cv::Mat &imGray,          //灰度图
            const double &timeStamp,        //时间戳
            ORBextractor* extractor,        //ORB特征点提取器
            ORBVocabulary* voc,             //ORB字典
            cv::Mat &K,                     //单目相机内参
            cv::Mat &distCoef,              //单目相机去畸变参数
            const float &bf,                //对单目相机来说，这个参数没用
            const float &thDepth);          //远点和近点的阈值区分

    // Extract ORB on the image. 0 for left image and 1 for right image.
    //从一张灰度图中提取ORB特征点
    //flag：0代表左图，flag：1代表右图
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    //计算当前帧的字典
    void ComputeBoW();

    // Set the camera pose.
    //用 Tcw 更新 mTcw 以及类中存储的一系列位姿
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    //计算和相机位姿有关的其他旋转矩阵和平移向量
    void UpdatePoseMatrices();

    // Returns the camera center.
    //返回相机在世界坐标系下的中心位置
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    //返回从当前相机坐标系到世界坐标系的旋转
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    //计算kp关键点，属于哪个cell，算出来之后以引用的方式得到cell的横坐标和纵坐标，如果失败就返回false
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    //相机内参矩阵
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    //原始左图像提取出的特征点（未校正），mvKeysRight代表右图（未校正）
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    //校正mvKeys后的特征点
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    //mvuRight存储左图特征点对应再右图中的横坐标（u代表横坐标，因为纵坐标是相同的）
    std::vector<float> mvuRight;
    //mvDepth存储特征点深度
    //对于单目相机来说，这两个值都为-1
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    //mBowVec是从std::map<WordId, WordValue>继承的，这里的WordId的取值范围即为生成词袋模型时word的数目(kn)。
    //这里存储的WordValue是 word 的 WordValue的累加值，当图像中的多个特征都与WordId对应的word相似时。
    DBoW2::BowVector mBowVec;
    //mFeatVec是被称为 direct index（直接索引） 的东西，用来支持图像与图像之间特征匹配的速度。
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    //图像帧里的关键点对应的地图点，如果没有对应的地图点，就赋值为空指针
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    //特征点对应的地图点属于离群点的标志
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    //这两个变量的作用是特征点坐标乘以以下两个变量就可以确定在哪个格子中
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    //这个向量中存储的是每个图像网格内特征点的id
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    //相机姿态 世界坐标系到相机坐标坐标系的变换矩阵
    cv::Mat mTcw;

    // Current and Next Frame id.
    
    //注意这个地方是类全局静态变量，在图像帧类中被初始化定义为0，每当有一个图像被送进来转换成帧的时候，这个变量都会+1
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    //未校正图像的边界，只需要计算一次，因为是类的静态成员变量
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    //标记是否进行了初始化操作
    //由于第一帧以及SLAM系统进行重新校正后的第一帧会有一些特殊的初始化处理操作，所以这里设置了这个变量
    //如果这个标志被置位，说明再下一帧的帧构造函数中要进行这个“特殊的初始化操作”，如果没有被置位则不用。
    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;       //相机位姿中的旋转矩阵
    cv::Mat mtcw;       //相机位姿中的平移向量
    cv::Mat mRwc;       //从相机坐标系到世界坐标系的旋转矩阵
    cv::Mat mOw; //==mtwc   //从相机坐标系到世界坐标系的平移向量
};

}// namespace ORB_SLAM

#endif // FRAME_H
