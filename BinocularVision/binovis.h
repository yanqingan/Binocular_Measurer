/********************************************************************
	创建 :	2013/6/30
	库名 :	BinocularVision
	作者 :	严庆安 yanqinganssg AT gmail/163 DOT com
	单位 :  武汉大学 
	使用 :	需要opencv库，并加入所依赖库路径和lib
	备注 :  具体使用参考BinocularTest2程序
*********************************************************************/


#pragma once

#include "stereomatch.h"

// 默认分辨率大小，一般不要改
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

/*----------structures----------*/
namespace bv
{
struct Point3d
{
    double x;
    double y;
    double z;
    unsigned int bf;    // bf 是可信度参数，以找到视察所需最小窗口大小为准。取值为：1，3，5，7
};
}

/*----------class of BinocularVision----------*/
class BinocularVision
{
public:
	BinocularVision();
	~BinocularVision();

	// 下面为接口函数

/*----------------------------
* 功能 : 获得摄像头的分辨率大小
*----------------------------
* 函数 : BinocularVision::GetImageSize
* 访问 : public 
* 返回 : void
*
* 参数 : width	    [out]	用来存放图像的列数
* 参数 : height		[out]	用来存放图像的行数
*/
	inline void GetImageSize(int &width, int &height)
	{
		width = m_nImgWidth;
		height = m_nImgHeight;
	}

/*----------------------------
* 功能 : 设置摄像头的分辨率大小
*----------------------------
* 函数 : BinocularVision::SetImageSize
* 访问 : public 
* 返回 : void
*
* 参数 : width	    [in]	要修改成的图像列数
* 参数 : height		[in]	要修改成的图像行数
*/
	// 如需修改分辨率大小，请在Init函数之前调用
	inline void SetImageSize(int width, int height)
	{
		m_nImgWidth = width;
		m_nImgHeight = height;
	}

/*----------------------------
* 功能 : 开启摄像头，程序初始化
*----------------------------
* 函数 : BinocularVision::Init
* 访问 : public 
* 返回 : bool
*
* 参数 : lfCamID	    [in]	左摄像头ID
* 参数 : riCamID		[in]	右摄像头ID
*/
	// 如果后来的左图不在左边，那么交换之，或者有2个以上的摄像头时，自己按照要求来设置摄像头编号
	bool Init(int lfCamID = 0 /* 1 2 ... */, int riCamID = 1 /* 0 2 ... */); 

/*----------------------------
* 功能 : 加载标定数据
*----------------------------
* 函数 : BinocularVision::LoadCalibData
* 访问 : public 
* 返回 : bool
*
* 参数 : filename	    [in]	文件路径
*/
	// 读取MFC版本生成的xml标定文件，在循环外调用
	bool LoadCalibData(const char *filename); 

/*----------------------------
* 功能 : 获取左右摄像机图像帧
*----------------------------
* 函数 : BinocularVision::GetBothFrames
* 访问 : public 
* 返回 : bool
*
* 参数 : lfImage	    [out]	存放左摄像头图像帧
* 参数 : riImage		[out]	存放右摄像头图像帧
*/
	// 获取双目图像，如需获得视频显示，需要在while(1)循环中调用
	bool GetBothFrames(cv::Mat &lfImage, cv::Mat &riImage);   

/*----------------------------
* 功能 : 计算一个三维点坐标
*----------------------------
* 函数 : BinocularVision::GetPointCloud
* 访问 : public 
* 返回 : bool
*
* 参数 : lfImage	    [in]	左摄像头图像帧
* 参数 : riImage		[in]	右摄像头图像帧
* 参数 : pl	            [in]	要检测的图像像素点坐标
* 参数 : pt		        [out]	存放计算得到的三维点坐标
* 参数 : maxDisp	    [in]	需要调整的视差参数
*/
	// bf为7代表找邻域后仍没有视差，无法得到空间点坐标
	bool GetPointCloud(cv::Mat lfImage, cv::Mat riImage, cv::Point2i pl, bv::Point3d &pt, int maxDisp = 192);

/*----------------------------
* 功能 : 计算多个三维点坐标
*----------------------------
* 函数 : BinocularVision::GetPointCloud
* 访问 : public 
* 返回 : bool
*
* 参数 : lfImage	    [in]	左摄像头图像帧
* 参数 : riImage		[in]	右摄像头图像帧
* 参数 : pls	        [in]	要检测的图像像素点坐标（向量）
* 参数 : pts		    [out]	存放计算得到的三维点坐标（向量）
* 参数 : maxDisp	    [in]	需要调整的视差参数
*/
	bool GetPointCloud(cv::Mat lfImage, cv::Mat riImage, std::vector<cv::Point2i> pls, std::vector<bv::Point3d> &pts, int maxDisp = 192);

/*----------------------------
* 功能 : 显示结果
*----------------------------
* 函数 : BinocularVision::Show
* 访问 : public 
* 返回 : int
*
* 参数 : isShowFrames	    [in]	是否显示左右图像（立体校正后的）
* 参数 : isShowDisp		    [in]	是否显示视差图
* 参数 : delay	            [in]	显示下一帧之前所需的延迟时间（单位：毫秒）
*/
	// 0代表要按任意键才能显示下一帧（除了Esc，因为Esc是程序退出命令，需要在自己的程序里面加入判断），其它数字代表延迟delay毫秒显示下一帧
	int Show(bool isShowFrames = true, bool isShowDisp = true, int delay = 0);    

/*----------------------------
* 功能 : 释放程序动态生成的内存
*----------------------------
* 函数 : BinocularVision::Close
* 访问 : public 
* 返回 : void
* 
* 参数 : 无
*/
	void Close();

private:
	cv::VideoCapture m_lfCam;
	cv::VideoCapture m_riCam;
	cv::Mat m_lfImage;
	cv::Mat m_riImage;
	cv::Mat m_dispImage;
	int m_lfCamID;
	int m_riCamID;
	int m_nCamCount;
	int m_nImgWidth;
	int m_nImgHeight;
	int m_nImgChannel;
	bool m_bFlag;
	StereoMatch m_stereoMatcher;
};






