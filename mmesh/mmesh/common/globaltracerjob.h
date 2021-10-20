#ifndef MMESH_GLOBALTRACERJOB_1634722353870_H
#define MMESH_GLOBALTRACERJOB_1634722353870_H
#include "mmesh/common/cancelablethread.h"
#include "ccglobal/tracer.h"

namespace mmesh
{
	class GlobalTracerJob : public mmesh::CancelableThread
		, public ccglobal::Tracer
	{
	public:
		GlobalTracerJob();
		virtual ~GlobalTracerJob();

		virtual void performRun(ccglobal::Tracer* tracer) {}
		void setTracer(ccglobal::Tracer* tracer);
	protected:
		void run() override;

		void progress(float r) override;
		bool interrupt() override;
		void failed(const char* message) override;
		void success() override;
		void message(const char* msg) override;
		void message(int msg, int ext1, int ext2, bool differentThread) override;
	protected:
		ccglobal::Tracer* m_tracer;
	};
}

#endif // MMESH_GLOBALTRACERJOB_1634722353870_H