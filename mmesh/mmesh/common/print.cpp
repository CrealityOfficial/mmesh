#include "print.h"
#include <iostream>
#include <cstdarg>

namespace mmesh
{
	void tracerFormartPrint(ccglobal::Tracer* tracer, const char* format, ...)
	{
		if (!tracer)
			return;

		char buf[1024] = { 0 };
		va_list args;
		va_start(args, format);
		vsprintf(buf, format, args);
		tracer->message(buf);
		va_end(args);
	}
}