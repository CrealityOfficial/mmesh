#include "cancelablethread.h"

namespace mmesh
{
	CancelableThread::CancelableThread()
		:m_interrupt(false)
		, m_finished(false)
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

		m_finished = false;
		m_interrupt = false;
		m_thread.reset(new std::thread(&CancelableThread::run, this));
	}

	void CancelableThread::cancelThread()
	{
		if (!m_thread)
			return;

		setInterrupt();
		m_thread->join();
		m_thread.reset();
		m_finished = true;
	}

	void CancelableThread::setInterrupt()
	{
		m_interrupt = true;
	}

	bool CancelableThread::isRunning()
	{
		return m_thread.get() && !m_finished;
	}

	bool CancelableThread::isInterrupt()
	{
		return m_interrupt;
	}
}