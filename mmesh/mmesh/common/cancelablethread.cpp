#include "cancelablethread.h"

namespace mmesh
{
	CancelableThread::CancelableThread()
		:m_interrupt(false)
	{

	}

	CancelableThread::~CancelableThread()
	{
		cancelThread();
	}

	void CancelableThread::startThread()
	{
		if (m_thread)
			cancelThread();

		m_interrupt = false;
		m_thread.reset(new std::thread(&CancelableThread::run, this));
	}

	void CancelableThread::cancelThread()
	{
		if (!isRunning())
			return;

		setInterrupt();
		m_thread->join();
		m_thread.reset();
	}

	void CancelableThread::setInterrupt()
	{
		m_interrupt = true;
	}

	bool CancelableThread::isRunning()
	{
		return m_thread.get();
	}

	bool CancelableThread::isInterrupt()
	{
		return m_interrupt;
	}
}