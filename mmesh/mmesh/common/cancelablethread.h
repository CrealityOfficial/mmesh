#ifndef CRSA_CANCELABLETHREAD_1629092754823_H
#define CRSA_CANCELABLETHREAD_1629092754823_H
#include <thread>
#include <mutex>
#include <list>
#include <memory>

namespace mmesh
{
	class CancelableThread
	{
	public:
		CancelableThread();
		virtual ~CancelableThread();

		void startThread();
		void cancelThread();
		void waitThread();

		bool isRunning();
	protected:
		bool isInterrupt();
		void setInterrupt();

		virtual void run() = 0;
	protected:
		bool m_interrupt;
		std::unique_ptr<std::thread> m_thread;
		std::mutex m_lock;

		bool m_finished;
	};
}

typedef std::shared_ptr<mmesh::CancelableThread> CancelableJobPtr;

#endif // CRSA_CANCELABLETHREAD_1629092754823_H