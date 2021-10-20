#ifndef MMESH_SEQUENCEJOBMANAGER_1634722353871_H
#define MMESH_SEQUENCEJOBMANAGER_1634722353871_H
#include "mmesh/common/cancelablethread.h"
#include <list>

namespace mmesh
{
	class SequenceJobManager : public CancelableThread
	{
	public:
		SequenceJobManager();
		~SequenceJobManager();

		void appendJob(CancelableJobPtr job);

	protected:
		void run() override;
	protected:
		std::list<CancelableJobPtr> m_jobs;
		CancelableJobPtr m_executeJob;
	};
}

#endif // MMESH_SEQUENCEJOBMANAGER_1634722353871_H