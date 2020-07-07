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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);                         //构造函数

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);                //更新绘制线程的信息，新信息是由pTracker追踪线程送来的

    // Draw last processed frame.
    cv::Mat DrawFrame();                            //绘制最新送来的帧信息

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);        //绘制底部文本信息

    // Info of the frame to be drawn
    cv::Mat mIm;        //正在绘制的当前帧
    int N;              //当前帧的特征点数量
    vector<cv::KeyPoint> mvCurrentKeys;     //当前帧的特征点
    vector<bool> mvbMap, mvbVO;             //当前帧中的特征点是否在地图中的标记，后者是表示地图中没有出现,但是在当前帧中是第一次被观测得到的点
    bool mbOnlyTracking;                    //当前是否是只有追踪线程在工作;或者说,当前是处于定位模式还是处于SLAM模式
    int mnTracked, mnTrackedVO;             //当前帧中追踪到的特征点计数
    vector<cv::KeyPoint> mvIniKeys;         //参考帧中的特征点
    vector<int> mvIniMatches;               //当前帧特征点和参考帧特征点的匹配关系
    int mState;                             //当前SLAM系统的工作状态

    Map* mpMap;                             //地图指针

    std::mutex mMutex;                      //线程锁
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
