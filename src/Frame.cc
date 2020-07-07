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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

//默认构造函数
Frame::Frame()
{}

//Copy Constructor
//拷贝构造函数
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

//双目模式的帧构造函数
Frame::Frame(const cv::Mat &imLeft,             //左目灰度图
            const cv::Mat &imRight,             //右目灰度图
            const double &timeStamp,            //时间戳
            ORBextractor* extractorLeft,        //左目灰度图ORB特征提取器
            ORBextractor* extractorRight,       //右目灰度图ORB特征提取器
            ORBVocabulary* voc,                 //ORB字典
            cv::Mat &K,                         //相机内参
            cv::Mat &distCoef,                  //去畸变参数
            const float &bf,                    //baseline*fx
            const float &thDepth)               //baseline的若干倍
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    //获取当前帧ID，并且更新下一帧ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();                        //获取图像金字塔层数
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();                   //获取金字塔缩放因子，配置文件是1.2
    mfLogScaleFactor = log(mfScaleFactor);                                  //金字塔缩放因子的对数
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();                 //获取金字塔的每层缩放因子
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();       //获取金字塔每层缩放因子的倒数
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();             //获取金字塔每层缩放因子的平方
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();   //获取金字塔每层缩放因子平方的倒数

    // ORB extraction
    //对左右两张图片进行ORB特征提取
    //首先定义了两个线程threadLeft和threadRight，
    thread threadLeft(  &Frame::ExtractORB,         //执行该线程的主函数
                        this,                       //父线程的指针
                        0,                          //0代表左图，1代表右图
                        imLeft);                    //灰度图
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();      //父进程等待子进程执行完毕之后再继续执行
    threadRight.join();

    //左目的特征点数量
    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    //对特征点去畸变
    UndistortKeyPoints();

    //计算双目图像之间的匹配关系，其实是根据左图特征点，来对右图特征点的x坐标进行微调，并且更新特征点的深度图
    ComputeStereoMatches();

    //先把所有的特征点对应的地图点都设置为NULL
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));  
    //并且默认这些地图点都不是离群点  
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    //只有第一帧的时候会初始化，或者在重定位之后
    /** 判断是否需要进行进行特殊初始化,这个过程一般是在第一帧或者是重定位之后进行.主要操作有:\n
     *      - 计算未校正图像的边界 Frame::ComputeImageBounds() 
     *      - 计算一个像素列相当于几个（<1）图像网格列
     *      - 给相机的内参数赋值
     *      - 标志复位
     */ 
    if(mbInitialComputations)
    {
        //计算去畸变之后图像的边界
        ComputeImageBounds(imLeft);

        //特征点坐标乘以以下两个变量就会确定特征点落在哪个格子
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        //焦距和图像坐标系中心点
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    //得到双目的基线
    mb = mbf/fx;

    //将提取出来的特征点分配到帧的网格中
    AssignFeaturesToGrid();
}

//RGBD模式的帧构造函数
Frame::Frame(const cv::Mat &imGray,             //灰度图
            const cv::Mat &imDepth,             //深度图
            const double &timeStamp,            //时间戳
            ORBextractor* extractor,            //ORB特征提取器
            ORBVocabulary* voc,                 //ORB字典
            cv::Mat &K,                         //相机内参
            cv::Mat &distCoef,                  //相机的去畸变参数
            const float &bf,                    //baseline*fx
            const float &thDepth)               //baseline的若干倍
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    //获取本帧ID，并且更新下一帧ID
    mnId=nNextId++;

    // Scale Level Info 
    mnScaleLevels = mpORBextractorLeft->GetLevels();                        //获取图像金字塔层数
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();                   //获取金字塔缩放因子，配置文件是1.2
    mfLogScaleFactor = log(mfScaleFactor);                                  //金字塔缩放因子的对数
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();                 //获取金字塔的每层缩放因子
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();       //获取金字塔每层缩放因子的倒数
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();             //获取金字塔每层缩放因子的平方
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();   //获取金字塔每层缩放因子平方的倒数

    // ORB extraction
    //提取灰度图特征点
    ExtractORB(0,imGray);

    //获取特征点数量N
    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    //对特征点去畸变
    UndistortKeyPoints();

    //通过深度图，来计算特征点在右图中的x坐标
    ComputeStereoFromRGBD(imDepth);

    //先把所有的特征点对应的地图点都设置为NULL
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    //并且默认这些地图点都不是离群点
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    //只有第一帧的时候会初始化，或者在重定位之后
    /** 判断是否需要进行进行特殊初始化,这个过程一般是在第一帧或者是重定位之后进行.主要操作有:\n
     *      - 计算未校正图像的边界 Frame::ComputeImageBounds() 
     *      - 计算一个像素列相当于几个（<1）图像网格列
     *      - 给相机的内参数赋值
     *      - 标志复位
     */ 
    if(mbInitialComputations)
    {
        //计算去畸变之后图像的边界
        ComputeImageBounds(imGray);

        //特征点坐标乘以以下两个变量就会确定特征点落在哪个格子
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        //焦距和图像坐标系中心点
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    //将提取出来的特征点分配到帧的网格中
    AssignFeaturesToGrid();
}


//单目模式的帧构造函数
Frame::Frame(   const cv::Mat &imGray,      //灰度图
                const double &timeStamp,    //时间戳
                ORBextractor* extractor,    //ORB特征提取器
                ORBVocabulary* voc,         //词典
                cv::Mat &K,                 //相机内参
                cv::Mat &distCoef,          //畸变系数矩阵
                const float &bf,            //x focal * baseline
                const float &thDepth)       //baseline的若干倍
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    //---------------Step1：获取帧ID----------------
    mnId=nNextId++;

    // Scale Level Info
    //---------------Step2：获取图像金字塔相关参数---------------
    //获取图像金字塔的层数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    //获取图像金字塔的缩放因子，就是配置文件里的1.2
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    //计算每层缩放因子的自然对数
    mfLogScaleFactor = log(mfScaleFactor);
    //获取图像金字塔每层各自的缩放因子，比如第一层是1，第二层是1.2，第三层是1.2*1.2，。。。
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    //获取图像金字塔每层各自的缩放因子的倒数
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    //获取图像金字塔每层各自缩放因子的平方
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    //获取图像金字塔每层各自缩放因子的平方的倒数
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    //-------------Step3：提取图像的ORB特征-----------------
    //提取ORB特征，0代表提取左图的特征点，1代表右图（单目用不到）
    ExtractORB(0,imGray);

    //N代表特征点个数
    N = mvKeys.size();

    //如果当前图像没有成功提取出来特征点，就return
    if(mvKeys.empty())
        return;

    //--------------Step4：对特征点进行矫正-----------------
    UndistortKeyPoints();

    // Set no stereo information
    //由于单目相机只有左图，所以这里，把mvuRight和mvDepth都设为-1
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    //先把所有特征点对应的地图点都设置为NULL
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    //先假设他们都不是离群点
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    //只有第一帧的时候会初始化，或者在重定位之后
    /** 判断是否需要进行进行特殊初始化,这个过程一般是在第一帧或者是重定位之后进行.主要操作有:\n
     *      - 计算未校正图像的边界 Frame::ComputeImageBounds() 
     *      - 计算一个像素列相当于几个（<1）图像网格列
     *      - 给相机的内参数赋值
     *      - 标志复位
     */ 
    if(mbInitialComputations)
    {
        //计算去畸变之后图像的边界
        ComputeImageBounds(imGray);

        //特征点坐标乘以以下两个变量就会确定特征点落在哪个格子
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        //焦距和图像坐标系中心点
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;


        //以后就不在做上述操作了
        mbInitialComputations=false;
    }

    //mb为基线（这里是单目，应该没用吧mb）
    mb = mbf/fx;

    //将提取出来的特征点分配到帧的网格中
    AssignFeaturesToGrid();
}


//将提取出来的特征点分配到帧的网格中，其实每一个网格存储的是特征点们的id
void Frame::AssignFeaturesToGrid()
{
    //先假设每个网格可以存储平均值的一半（至于为什么是一半，可能是经验吧）
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        //nGridPosX和nGridPosY 以引用的方式传递参数，如果求出来特征点所在格子的横纵坐标之后，就返回true
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            //把特征点id放入格子中
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

//提取灰度图的特征点
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
    //这里用到了ORB类中的（）操作符
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

//设置当前相机位姿（意思是从世界坐标系到相机坐标系的变换矩阵T）
void Frame::SetPose(cv::Mat Tcw)
{
    //深复制；a.copyTo(b)这种也是深复制
    //copyTo函数、clone函数拷贝的不仅仅是信息头，还有矩阵本身，而“= ”运算符与拷贝构造函数仅仅拷贝了信息头，他们指向的其实是一个矩阵
    mTcw = Tcw.clone();
    //更新和相机位姿相关的
    UpdatePoseMatrices();
}

//更新和相机位姿相关的变量
void Frame::UpdatePoseMatrices()
{ 
    //得到旋转矩阵（从世界坐标系到相机坐标系）
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    //取逆代表相反的旋转（因为旋转矩阵肯定是正交矩阵，正交矩阵的转置就是逆，所以这里直接转置就可以了）
    mRwc = mRcw.t();
    //从变换矩阵中取出平移向量（前三行的第四列）
    mtcw = mTcw.rowRange(0,3).col(3);
    //光心在世界坐标系下的坐标
    mOw = -mRcw.t()*mtcw;
}

// 判断路标点是否在视野中
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}


//获取指定区域内的特征点ID
//区域中心x坐标
//区域中心y坐标
//区域的半径r
//搜索的图像金字塔层数的下限,也可以理解为最小尺度
//搜索的图像金字塔层数的上限，同样可以理解为最大尺度
//返回包含有这个区域内所有特征点的向量，该向量中存储的的是特征点的序号
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    //定义结果向量，预先分配N个大小
    vector<size_t> vIndices;
    vIndices.reserve(N);

    //检查圆形区域是否在图像中，具体做法是分别求圆形搜索区域的上下左右四个边界是否能够满足图像的边界条件。
    //这里的边界条件以圆的左边界为例，就是首先求出左边界所在的图像网格列，然后判断这个网格列位置是否超过了图像网格的上限。

    //下面的这段计算的代码其实可以这样理解：
	//首先(mnMaxX-mnMinX)/FRAME_GRID_COLS表示每列网格可以平均分得几个像素坐标的列
	//那么它的倒数，就可以表示每个像素列相当于多少（<1）个网格的列
	//而前面的(x-mnMinX-r)，可以看做是从图像的左边界到半径r的圆的左边界区域占的像素列数
	//两者相乘，就是求出那个半径为r的圆的左侧边界在那个网格列中。这个变量的名其实也是这个意思
    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

//计算kp关键点，属于哪个cell，算出来之后以引用的方式得到cell的横坐标和纵坐标，如果失败就返回false
bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


//计算当前帧的字典
void Frame::ComputeBoW()
{
    //只有当当前帧没有被计算字典时，才进行计算
    if(mBowVec.empty())
    {
        //要写入词袋信息,将以OpenCV格式存储的描述子 Frame::mDescriptors 转换成为vector<cv::Mat>存储
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        //将特征点的描述子转换成为当前帧的词袋
        //4代表字典树中的从叶子节点向上数的第4层
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

//对特征点去畸变
void Frame::UndistortKeyPoints()
{
    /*-----------Step1：首先判断图像是否已经去畸变-------------*/
    //如果mDistCoef内存储的畸变参数为0，那么说明配置文件里的配置就是0，也就是说图像已经经过去畸变了，这里直接赋值返回就行
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    /*-----------Step2：如果图像没有经过去畸变，那么就对特征点进行矫正-------------*/
    //首先用mat来保存特征点的坐标
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    // 调整mat的通道为2，矩阵的行列形状不变
    //这个函数的原型是：cv::Mat::reshape(int cn,int rows=0) const
    //其中cn为更改后的通道数，rows=0表示这个行将保持原来的参数不变
    //不过根据手册发现这里的修改通道只是在逻辑上修改，并没有真正地操作数据
    //这里调整通道的目的应该是这样的，下面的undistortPoints()函数接收的mat认为是2通道的，两个通道的数据正好组成了一个点的两个坐标
    mat=mat.reshape(2);
    cv::undistortPoints(        // 用cv的函数进行失真校正
        mat,                    //输入的特征点坐标
        mat,                    //输出的特征点坐标，也就是校正后的特征点坐标
        mK,                     //相机的内参数矩阵
        mDistCoef,              //保存相机畸变参数的变量
        cv::Mat(),              //一个空的cv::Mat()类型，对应为函数原型中的R。Opencv的文档中说如果是单目相机，这里可以是空矩阵
        mK);                    //相机的内参数矩阵，对应为函数原型中的P

    //然后调整回只有一个通道，回归我们正常的处理方式
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector

    //*-----------Step3：把处理过后的特征点坐标赋值给mvKeysUn-------------*/
    //申请空间，并初始化为0
    mvKeysUn.resize(N);
    //for循环遍历每一个特征点

    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

//计算去畸变后图片的边界
void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    //首先判断是否已经做过去畸变
    if(mDistCoef.at<float>(0)!=0.0)
    {
        //创建一个4行2列的矩阵（主要是用来保存图像边界4个坐标点）
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;                 //左上
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;         //右上
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;         //左下
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows; //右下

        // Undistort corners
        //这里和前面的操作是一样的
        // 调整mat的通道为2，矩阵的行列形状不变
        //这个函数的原型是：cv::Mat::reshape(int cn,int rows=0) const
        //其中cn为更改后的通道数，rows=0表示这个行将保持原来的参数不变
        //不过根据手册发现这里的修改通道只是在逻辑上修改，并没有真正地操作数据
        //这里调整通道的目的应该是这样的，下面的undistortPoints()函数接收的mat认为是2通道的，两个通道的数据正好组成了一个点的两个坐标
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        //校正后的四个边界点已经不能够围成一个严格的矩形，因此在这个四边形的外侧加边框作为坐标的边界
        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));    //左上和左下横坐标最小的
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));    //右上和右下横坐标最大的
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));    //左上和右上纵坐标最小的
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));    //左下和右下纵坐标最小的

    }
    else
    {
        //如果之前矫正过，直接赋值
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

//计算双目图像之间的匹配关系
//首先是根据左图提取出来的特征点
void Frame::ComputeStereoMatches()
{
    //右图存在的是左图特征点在右图中的横坐标
    mvuRight = vector<float>(N,-1.0f);
    //深度图存放的是左图特征点的深度（可以计算出深度的特征点）
    mvDepth = vector<float>(N,-1.0f);

    //这个是ORB描述子之间的阈值，如果小于这个阈值就认为是成功的匹配
    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    //得到完整图像的行数
    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    //size_t表示C++容器中的元素个数，std::size_t的优点是不需要判断小于0
    //vRowIndices这个变量的目的是为了减小特征点搜索范围
    //举个例子，如果说左图的大小为200*200，在这张图上检测出来50个特征点，那么如何在右图的某一部分区域来找到对应的特征点呐？
    //是这样的，把左图某一个特征点对应的y，然后把y-2,y-1,y,y+1,y+2这些值存储在vRowIndices中的某一个vector中
    //然后在就这么多行中去搜索对应的特征点，这样就可以起到减小特征点搜索范围的目的
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    //为每一行的vector分配200个大小（这里应该是一个经验值）
    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    //右图图像中特征点的个数
    const int Nr = mvKeysRight.size();

    //初始化上面定义的vRowIndices变量
    for(int iR=0; iR<Nr; iR++)
    {
        //首先得到右图中每一个特征点的y坐标
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        //然后根据这个特征点所在的金字塔层数，来动态地得到r地值
        //如果层数较低，则不确定性较小，那么r也较小；如果层数较高，那么不确定性较大，r也较大。
        //如果特征点在金字塔第一层，则搜索范围为:正负2（其实金字塔越往上，一个像素就代表了好几个像素）
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        //这里的minr有可能小于0吗？
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;               //minD代表最小视差，这里设置为0也就代表了最大深度
    const float maxD = mbf/minZ;        //maxD代表最大视差，也就是最小深度；这个地方mbf/minZ得到地是fx焦距呀。。。。。（这都是经验吗？？？）

    // For each left keypoint search a match in the right image
    //vDistIdx保存每一个左图特征点在特征匹配的过程中，得到的最佳距离 bestDist (SAD匹配最小匹配偏差)和左图特征点id
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    //开始对每一个左图特征点进行匹配
    for(int iL=0; iL<N; iL++)
    {
        //得到左图特征点kpL
        const cv::KeyPoint &kpL = mvKeys[iL];
        //得到kpL的金字塔层级
        const int &levelL = kpL.octave;
        //y坐标和x坐标
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        //获取这个特征点所在行，在右图中可能的匹配点，注意这里 vRowIndices 存储的是右图的特征点索引
        const vector<size_t> &vCandidates = vRowIndices[vL];

        //如果左图的特征点在右图中没有对应的特征点，就跳过左图这个特征点
        if(vCandidates.empty())
            continue;

        
        const float minU = uL-maxD;         //minU代表匹配出来的右图特征点的最小x坐标
        const float maxU = uL-minD;         //maxU代表匹配出来的右图特征点的最大x坐标，其实这么结果还是uL，因为minD被设置为了0

        //如果最大匹配范围小于0，就返回
        if(maxU<0)
            continue;

        //设置ORB描述子之间的最佳距离；ORBmatcher::TH_HIGH，被定义为100
        int bestDist = ORBmatcher::TH_HIGH;
        //左图特征点对应的右图特征点的id（被定义为size_t类型，都是细节啊。）
        size_t bestIdxR = 0;

        //左图特征点的描述子
        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        //遍历右目所有可能的匹配点，找出最佳匹配点（描述子距离最小）
        //vCandidates存储的是左图特征点所在的行在右图中对应的特征点，如果这一行有5个特征点，那么用这5个特征点和左图特征点分别比较
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            //获取右图对应的特征点id
            const size_t iR = vCandidates[iC];
            //得到右图特征点
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            //如果左右特征点的金字塔层级相差在2以上，就认为是不可靠的
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            //得到右图特征点的x坐标（也就是u坐标）
            const float &uR = kpR.pt.x;

            //如果符合最小和最大坐标的范围，就计算描述子之间的汉明距离
            if(uR>=minU && uR<=maxU)
            {   
                //得到右图特征点对应的描述子
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                //计算它们之间的距离
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);


                //距离还是越小越好的（所以ORBmatcher::TH_HIGH还是一个阈值，并不是最好的值）
                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;      //同时把右图最优特征点id也记录下来
                }
            }
        }

        // Subpixel match by correlation 
        //下面开始SAD算法，对右图特征点进行亚像素级别的调整
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            
            //这里特征点在金字塔图层中缩放后的坐标，是为了下一步SAD算法计算滑动窗口做准备
            const float uR0 = mvKeysRight[bestIdxR].pt.x;               //得到原始右图特征点横坐标
            const float scaleFactor = mvInvScaleFactors[kpL.octave];    //左图特征点所在金字塔的缩放因子的倒数
            const float scaleduL = round(kpL.pt.x*scaleFactor);         //左图特征点所在金字塔图层的横坐标
            const float scaledvL = round(kpL.pt.y*scaleFactor);         //左图特征点所在金字塔图层的纵坐标
            const float scaleduR0 = round(uR0*scaleFactor);             //右图特征点所在金字塔图层的横坐标

            // sliding window search
            //w为SAD算法滑动窗口的移动范围
            //滑动窗口是以左图特征点为中心，在缩放后的图片上，扩展出11*11的窗口，这也是为什么w=5的原因
            const int w = 5;
            //得到以左图特征点为中心的小窗口
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            //类型设置为浮点型
            IL.convertTo(IL,CV_32F);
            //将小窗口内所有像素的像素值减去中心点的像素值
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            //用来保存在SAD匹配中得到的最优的SAD距离，这里先默认为INT_MAX
            int bestDist = INT_MAX;
            //用来保存最优的SAD匹配所对应的修正量
            int bestincR = 0;
            //滑动窗口的范围是（-L，+L）
            const int L = 5;
            //存储的是SAD比较出来的距离
            vector<float> vDists;
            vDists.resize(2*L+1);

            //滑动的边缘检测
            //iniu是，以右图特征点为中心构造出来的小窗口，scaleduR0为原本的右图特征点对应的x坐标，
            //L为特征点的偏移量（因为在比较SAD距离的时候，右图特征点横坐标是不断变化的，这样才能找到最优的SAD，而这时候的L也就是那个修正量）
            //w就是用来构造边界了，因为边界一直距离中心特征点是5，w才是死的，下面的for循环时按照L的范围来的
            //下面的if判断就是看看 构造出来的小窗口是不是边界出离了 这个金字塔图层的边界，出离了的话，就不要这个了
            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            //这里开始判断左图小窗口和右图小窗口之间的SAD距离（对应像素差的绝对值之和）
            //注意啦！在比较的过程中，左图小窗口时不变的，右图小窗口是不断往右移动的
            for(int incR=-L; incR<=+L; incR++)
            {
                //获取右图小窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                //转换为浮点型
                IR.convertTo(IR,CV_32F);
                //和上面一样，所有像素的像素值全部减去中心特征点的像素值
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                //用opencv函数来计算SAD距离，就是一范数，计算差的绝对值之和
                float dist = cv::norm(IL,IR,cv::NORM_L1);
                //更新最优SAD距离，以及最优的修正量（右图特征点的x坐标的偏移量）
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                //这里之所以用L+incR的形式作为下标，我想是因为想让下表从0-2L，一共2L+1个下标
                //这样画出来函数图像的话，应该是一个开口向上的抛物线，在下标为L的地方，SAD距离最小（按道理是这样的，也不绝对）
                vDists[L+incR] = dist;
            }

            //如果偏偏 小窗口在最极端的两侧的话，就认为是失败的匹配？
            //应该时因为 按照常理最优的SAD不应该在两端出现，否则就认为是失败，也就是说修正量应该是很小的数，不可能太大
            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            //这里做抛物线拟合，找到谷底的亚像素最优值
            //也就是说，假如下标5，6，7，按道理来说6为谷底，但是因为这是一个不完全的抛物线，最优值存在于6.2或者5.8
            //就是这个意思
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];


            //这里的deltaR为什么这样计算，我还不知道。。
            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            //抛物线拟合得到的亚像素偏移不能超过一个像素，否则放弃求该特征点的深度
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            //计算右图特征点修正之后，在原始图的横坐标
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            //计算左右特征点的视差
            float disparity = (uL-bestuR);

            //判断视差是否合理
            if(disparity>=minD && disparity<maxD)
            {   
                //若视差为负,则说明视差本身非常低小以至于计算机在进行数值计算时出现了误差
                //就给它设置为0.01
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;          //记录深度值
                mvuRight[iL] = bestuR;              //记录特征点在右图中对应的横坐标
                vDistIdx.push_back(pair<int,int>(bestDist,iL));     //记录左图特征点的最优SAD距离，以及特征点id
            }
        }
    }

    //按照SAD距离，从大到小排序
    sort(vDistIdx.begin(),vDistIdx.end());
    //取一个中间值作为阈值
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;//又是经验吧。。

    //因为是要剔除SAD过于大的特征点，所以倒着来
    //（为什么要设置这么多条件，一定要经过层层筛选，才能计算出来良好的配对特征点吗？？不怕最后搞得没有多少配对成功的特征点了嘛）
    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        //只要有一开始小于阈值，那么 往后的就都小于阈值了
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            //还是设置为-1
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


//计算RGBD图像的立体深度信息
//更新depth和右图特征点的x坐标
void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    //首先初始化
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        //分别得到未矫正的特征点和矫正之后的特征点
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];


        //获取其横纵坐标，注意 NOTICE 是校正前的特征点的
        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        //从深度图像中获取这个特征点对应的深度点
        //NOTE 从这里看对深度图像进行去畸变处理是没有必要的,我们依旧可以直接通过未矫正的特征点的坐标来直接拿到深度数据
        const float d = imDepth.at<float>(v,u);

        //如果这个特征点合法（大于0），那么就更新对用的深度值和右图的x坐标
        //这里主要说一下右图x坐标的更新，是拿矫正之后的特征点的x坐标减去mbf/d（就是视差）
        //因为在双目视觉中，有一个公式叫做z=fb/d，其中z为深度值，d为视差，f为x的焦距，b为基线，看得出来视差和深度值成反比
        if(d>0)
        {
            mvDepth[i] = d;
            //注意这个地方不是加的，而是减的，是因为相机在向右移动，左图的像素在右图中的体现是偏左的
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

//将第i个特征点的像素坐标系转成世界坐标系
cv::Mat Frame::UnprojectStereo(const int &i)
{
    //获取第i个关键点的深度值z
    const float z = mvDepth[i];
    if(z>0)
    {
        //这里是像素坐标系坐标转成相机坐标系的坐标
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        //然后再根据旋转矩阵和平移向量，将当前相机坐标转换成世界坐标系
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
