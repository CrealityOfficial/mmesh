#ifndef MMESH_LOGCALLBACK_1614316131385_H
#define MMESH_LOGCALLBACK_1614316131385_H
#include <string>

class LogCallback
{
public:
	virtual ~LogCallback() {}

	virtual void log(const char* format, ...) = 0;
	virtual void logs(const std::string& logger) = 0;
};

#endif // MMESH_LOGCALLBACK_1614316131385_H