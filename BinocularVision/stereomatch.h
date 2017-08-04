/********************************************************************
	创建 :	2013/6/30
	库名 :	BinocularVision
	作者 :	严庆安 yanqinganssg AT gmail/163 DOT com
	单位 :  武汉大学 
	使用 :	需要opencv库，并加入所依赖库路径和lib
	备注 :  具体使用参考BinocularTest2程序
*********************************************************************/


#pragma once

#include <vector>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"


class StereoMatch
{
public:
	StereoMatch();
	virtual ~StereoMatch();

	int init(int imgWidth, int imgHeight, const char *xmlFilePath);
	int bmMatch(cv::Mat &frameLeft, cv::Mat &frameRight, cv::Mat &disparity, cv::Mat &imageLeft, cv::Mat &imageRight);
	int getPointClouds(cv::Mat &disparity, cv::Mat &pointClouds);
	int getDisparityImage(cv::Mat &disparity, cv::Mat &disparityImage, bool isColor = false);
	void savePointClouds(cv::Mat &pointClouds, const char *filename);
    inline void setViewField(int viewWidth, int viewHeight, int viewDepth)
    {
        m_nViewWidth = viewWidth;
        m_nViewHeight = viewHeight;
        m_nViewDepth = viewDepth;
    }

	cv::StereoBM m_BM;     // 立体匹配 BM 方法
	double m_FL;    // 左摄像机校正后的焦距值

private:

	int loadCalibData(const char *xmlFilePath);

	bool m_Calib_Data_Loaded;    // 是否成功载入定标参数
    cv::Mat m_Calib_Mat_Q;    // Q 矩阵
	cv::Mat m_Calib_Mat_Remap_X_L;    // 左视图畸变校正像素坐标映射矩阵 X
	cv::Mat m_Calib_Mat_Remap_Y_L;    // 左视图畸变校正像素坐标映射矩阵 Y
	cv::Mat m_Calib_Mat_Remap_X_R;    // 右视图畸变校正像素坐标映射矩阵 X
	cv::Mat m_Calib_Mat_Remap_Y_R;    // 右视图畸变校正像素坐标映射矩阵 Y
	cv::Mat m_Calib_Mat_Mask_Roi;    // 左视图校正后的有效区域
	cv::Rect m_Calib_Roi_L;    // 左视图校正后的有效区域矩形
	cv::Rect m_Calib_Roi_R;    // 右视图校正后的有效区域矩形

	int m_frameWidth;    // 帧宽
	int m_frameHeight;    // 帧高
	int m_numberOfDisparies;    // 视差变化范围

    int m_nViewWidth;    // 视场宽度
    int m_nViewHeight;    // 视场高度
    int m_nViewDepth;    // 视场深度

};