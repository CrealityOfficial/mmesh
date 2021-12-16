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

//��Ϣ��
#define CODE_PROGRESS           			2         //������Ϣ  (ext1 ��ʾ���� 0 - 100)
#define CODE_SAVE_MOON_STL  				1000      //�����ģ�ͱ������ (ext1=1 �ɹ���ext1=-1 ʧ��)
#define CODE_SLICE_LOAD_FILE  				1001      //��Ƭ����ģ���ļ��������  (ext1=1 �ɹ���ext1=-1 ʧ��,  ext2 ����ģ�� id)
#define CODE_PHOTO_LOAD_FILE 				1002      //3D��Ƭ����ͼ�������  (ext1=1 �ɹ���ext1=-1 ʧ��)
#define CODE_PHOTO_SAVE_STL     			1003      //3D��Ƭģ�ͱ������ (ext1=1 �ɹ���ext1=-1 ʧ��)
#define CODE_SNAPSHOT_RENDER_FINISH     	1004      //��ͼ�߼���Ⱦ������� (ext1=1 �ɹ���ext1=-1 ʧ��)


//������
#define ERROR_FILE_OPEN         			10000     //���ļ�����

#define ERROR_LOAD_MODEL_FILE_OPEN          ERROR_FILE_OPEN    //����ģ�ʹ��ļ�ʧ�ܣ��� fd ת��ʧ��
#define ERROR_LOAD_MODEL_UNKOWN_FORMAT      10050     //��֧�ֵ�ģ���ļ�����
#define ERROR_LOAD_MODEL_STL_TEXT           10051     //��ȡ�ı���ʽSTL, ��ʽ����
#define ERROR_LOAD_MODEL_EMPTY_CONTENT      10052     //����ģ��, ģ��Ϊ��

#define ERROR_PHOTO3D_INVALID_IMAGE    		10100     //3D��Ƭ����ʱ,ͼ�񲻺Ϸ�
#define ERROR_PHOTO3D_ENCHASE_MATRIX    	10101     //3D��Ƭ����ʱ,�㷨���ɻҶ�ͼʧ��
#define ERROR_PHOTO3D_ENCHASE		    	10103     //3D��Ƭ����ʱ,�㷨ʧ��
#define ERROR_PHOTO3D_WRITE_EMPTY_NAME      10104     //3D��Ƭ����ʱ������Ϊ��
#define ERROR_PHOTO3D_WRITE_EMPTY_CONTENT   10105     //3D��Ƭ����ʱ��ģ�Ͷ�����������Ϊ��
#define ERROR_PHOTO3D_WRITE_FILE_OPEN       ERROR_FILE_OPEN     //3D��Ƭ����ʱ������Ϊ��
#define ERROR_PHOTO3D_WRITE    				10106     //3D��Ƭ����ʱ,����ʧ��

#endif // _NULLSPACE_SELFDEFFUNC_1591791524731_H
