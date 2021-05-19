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
	};
}

#endif // _NULLSPACE_SELFDEFFUNC_1591791524731_H
