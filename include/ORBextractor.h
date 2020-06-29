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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{


//提取器节点
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    //节点分裂为4个子节点的函数
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;            //本节点中的特征点
    cv::Point2i UL, UR, BL, BR;                 //本节点的坐标范围（左上、右上、左下、右下）
    std::list<ExtractorNode>::iterator lit;     //list（双向链表）的迭代器
    bool bNoMore;       //不能够再分裂的标志（代码中判断这个这个节点的特征数是否为1，如果为1，则bNoMore为真。如果没有特征点的话，这个节点就会被删除）
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    //C++重载括号（调用）操作符
    //这里举一个C++Primer中的一个例子
    // struct absInt{
    //     int operator() (int val){
    //         return val < 0 ? -val : val;
    //     }
    // }
    //这个类定义了一个操作：函数调用操作符，该操作符有一个形参并返回形参的绝对值。
    // int i = -42;
    // absInt absObj;
    // unsigned int ui = absObj(i);
    //尽管absObj是一个对象而不是函数，我们仍然可以“调用”该对象，效果是运行由absObj对象定义的重载调用操作符
    //该操作符接受一个int值并返回它的绝对值。
    //函数调用操作符必须声明为成员函数。一个类可以定义函数调用操作符的多个版本，由形参的数目或类型加以区别。
    //定义了调用操作符的类，其对象常称为函数对象（function object），即它们是行为类似函数的对象。
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }


    //存储的是图像金字塔每一层的灰度图
    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    //pattern变量将用于描述子的计算
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    //记录图片金字塔每一层应该提取的特征数量，顺序是从低到高
    std::vector<int> mnFeaturesPerLevel;

    //用于 ORB特征 的方向向量
    //用来计算灰度质心法中的圆形区域每一行应有的像素数
    std::vector<int> umax;

    std::vector<float> mvScaleFactor;           //存储的是原始的缩放率，比如缩放率是1.2，那么存储的就是1.2
    std::vector<float> mvInvScaleFactor;        //存储的是缩放率的倒数，比如1 / 1.2
    std::vector<float> mvLevelSigma2;           //存储的是缩放因子的平方
    std::vector<float> mvInvLevelSigma2;        //存储缩放因子平方的倒数（这里为什么要定义缩放因子的平方？）
};

} //namespace ORB_SLAM

#endif

