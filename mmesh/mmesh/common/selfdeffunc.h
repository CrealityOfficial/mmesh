#ifndef _NULLSPACE_SELFDEFFUNC_1591791524731_H
#define _NULLSPACE_SELFDEFFUNC_1591791524731_H
#include <functional>

typedef std::function<void(float)> selfProgressFunc;
typedef std::function<bool()> selfInterruptFunc;
typedef std::function<void(const char*)> selfFailedFunc;

namespace mmesh
{
	class StatusTracer
	{
	public:
		virtual ~StatusTracer() {}
		virtual void progress(float r) = 0;
		virtual bool interrupt() = 0;
		virtual void failed(const char* message) = 0;
		virtual void message(const char* format, ...) {};

		virtual void message(int msg, int ext1, int ext2, bool differentThread){}
	};
}

//消息码
#define CODE_PROGRESS           			2         //进度消息  (ext1 表示进度 0 - 100)
#define CODE_SAVE_MOON_STL  				1000      //月球灯模型保存完成 (ext1=1 成功，ext1=-1 失败)
#define CODE_SLICE_LOAD_FILE  				1001      //切片场景模型文件加载完成  (ext1=1 成功，ext1=-1 失败,  ext2 返回模型 id)
#define CODE_PHOTO_LOAD_FILE 				1002      //3D照片背景图加载完成  (ext1=1 成功，ext1=-1 失败)
#define CODE_PHOTO_SAVE_STL     			1003      //3D照片模型保存完成 (ext1=1 成功，ext1=-1 失败)
#define CODE_SNAPSHOT_RENDER_FINISH     	1004      //截图逻辑渲染操作完成 (ext1=1 成功，ext1=-1 失败)


//错误码
#define ERROR_FILE_OPEN         			10000     //打开文件错误

#define ERROR_LOAD_MODEL_FILE_OPEN          ERROR_FILE_OPEN    //加载模型打开文件失败，或 fd 转换失败
#define ERROR_LOAD_MODEL_UNKOWN_FORMAT      10050     //不支持的模型文件类型
#define ERROR_LOAD_MODEL_STL_TEXT           10051     //读取文本格式STL, 格式错误
#define ERROR_LOAD_MODEL_EMPTY_CONTENT      10052     //加载模型, 模型为空

#define ERROR_PHOTO3D_INVALID_IMAGE    		10100     //3D照片保存时,图像不合法
#define ERROR_PHOTO3D_ENCHASE_MATRIX    	10101     //3D照片保存时,算法生成灰度图失败
#define ERROR_PHOTO3D_ENCHASE		    	10103     //3D照片保存时,算法失败
#define ERROR_PHOTO3D_WRITE_EMPTY_NAME      10104     //3D照片保存时，名字为空
#define ERROR_PHOTO3D_WRITE_EMPTY_CONTENT   10105     //3D照片保存时，模型顶点数，面数为空
#define ERROR_PHOTO3D_WRITE_FILE_OPEN       ERROR_FILE_OPEN     //3D照片保存时，名字为空
#define ERROR_PHOTO3D_WRITE    				10106     //3D照片保存时,保存失败

#endif // _NULLSPACE_SELFDEFFUNC_1591791524731_H
