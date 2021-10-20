#include "sequencejobmanager.h"

namespace mmesh
{
	SequenceJobManager::SequenceJobManager()
	{

	}

	SequenceJobManager::~SequenceJobManager()
	{

	}

	void SequenceJobManager::appendJob(CancelableJobPtr job)
	{
		if (!job || m_interrupt)
			return;

		m_lock.lock();
		m_jobs.push_back(job);
		m_lock.unlock();
	}

	void SequenceJobManager::run()
	{
		while (!m_interrupt)
		{
			if (m_executeJob && !m_executeJob->isRunning())
				m_executeJob.reset();
			
			if (!m_executeJob)
			{
				m_lock.lock();
				if (m_jobs.size() > 0)
				{
					m_executeJob = m_jobs.front();
					m_jobs.pop_front();
				}
				m_lock.unlock();

				if (m_executeJob)
					m_executeJob->startThread();
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (m_executeJob)
		{
			m_executeJob->cancelThread();
			m_executeJob.reset();
		}

		m_jobs.clear();
	}
}