#include "globaltracerjob.h"

namespace mmesh
{
	GlobalTracerJob::GlobalTracerJob()
		: m_tracer(nullptr)
	{

	}

	GlobalTracerJob::~GlobalTracerJob()
	{

	}

	void GlobalTracerJob::setTracer(ccglobal::Tracer* tracer)
	{
		m_tracer = tracer;
	}

	void GlobalTracerJob::run()
	{
		performRun(this);
		m_finished = true;
	}

	void GlobalTracerJob::progress(float r)
	{
		if (m_tracer)
			m_tracer->progress(r);
	}

	bool GlobalTracerJob::interrupt()
	{
		return m_interrupt;
	}

	void GlobalTracerJob::failed(const char* msg)
	{
		if (m_tracer)
			m_tracer->failed(msg);
	}

	void GlobalTracerJob::success()
	{
		if (m_tracer)
			m_tracer->success();
	}

	void GlobalTracerJob::message(const char* msg)
	{
		if (m_tracer)
			m_tracer->message(msg);
	}

	void GlobalTracerJob::message(int msg, int ext1, int ext2, bool differentThread)
	{
		if (m_tracer)
			m_tracer->message(msg, ext1, ext2, differentThread);
	}
}