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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(   System* pSys,                   //SLAM系统指针
                ORBVocabulary* pVoc,            //ORB字典指针
                FrameDrawer* pFrameDrawer,      //帧绘制器
                MapDrawer* pMapDrawer,          //地图绘制器
                Map* pMap,                      //地图类
                KeyFrameDatabase* pKFDB,        //关键帧数据库
                const string &strSettingPath,   //配置文件路径
                const int sensor);              //传感器类型

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    //在一张图片读取进来之前，需要做一些准备工作，然后才能开始跟踪
    //包括把当前的图片转换成灰度图，然后把图片转换成帧数据，然后就能获取当前帧了，接下来才可以跟踪
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    //设置局部地图句柄
    void SetLocalMapper(LocalMapping* pLocalMapper);
    //设置回环检测器句柄
    void SetLoopClosing(LoopClosing* pLoopClosing);
    //设置可视窗口句柄
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    //更改校准？？？（没见过用到这个函数）
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    //设置SLAM是否处于仅定位模式
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,    // 系统没有准备好的状态,一般就是在启动后加载配置文件和词典文件时候的状态
        NO_IMAGES_YET=0,        // 当前无图像,图像复位过、或者第一次运行
        NOT_INITIALIZED=1,      // 有图像但是没有完成初始化
        OK=2,                   // 正常时候的工作状态
        LOST=3                  // 系统已经跟丢了的状态
    };

    eTrackingState mState;                  //当前的跟踪状态
    eTrackingState mLastProcessedState;     //上一帧的跟踪状态

    // Input sensor
    int mSensor;                            //传感器类型

    // Current Frame
    Frame mCurrentFrame;                    //当前帧
    cv::Mat mImGray;                        //灰度图（左图）

    // Initialization Variables (Monocular)
    //初始化的时候，牵涉到的前两帧的相关变量（参考帧和当前帧）
    std::vector<int> mvIniLastMatches;          //这个变量没用到。。。。
    std::vector<int> mvIniMatches;              //参考帧和当前帧的特征点匹配关系
    std::vector<cv::Point2f> mvbPrevMatched;    //参考帧的特征点
    std::vector<cv::Point3f> mvIniP3D;          //初始化过程中，匹配之后经过三角化的空间点
    Frame mInitialFrame;                        //初始化过程中的参考帧

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;     //所有的参考关键帧位姿
    list<KeyFrame*> mlpReferences;          //参考关键帧
    list<double> mlFrameTimes;              //参考关键帧的时间戳
    list<bool> mlbLost;                     //当前帧和参考帧的跟踪过程中，是否跟丢了的标志

    // True if local mapping is deactivated and we are performing only localization
    //是否仅定位的标志
    bool mbOnlyTracking;

    void Reset();   //整个系统进行复位操作

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();       //追踪函数

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
