#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

#include "ErrorCodes.h"
#include <string>

std::string ReplaceString(std::string source, std::string find, std::string replacement);
void sleepms(int milliseconds);

class RoboteqDevice
{
private:
	int device_fd;
	int fd0;
	int handle;

protected:
	void InitPort();

	int Write(std::string str);
	int ReadAll(std::string &str);

	int IssueCommand(std::string commandType, std::string command, std::string args, int waitms, std::string &response, bool isplusminus = false);
	int IssueCommand(std::string commandType, std::string command, int waitms, std::string &response, bool isplusminus = false);

public:
	bool IsConnected();
	int Connect(std::string port);
	void Disconnect();

	int SetConfig(int configItem, int index, int value);
	int SetConfig(int configItem, int value);

	int SetCommand(int commandItem, int index, int value);
	int SetCommand(int commandItem, int value);
	int SetCommand(int commandItem);

	int GetConfig(int configItem, int index, int &result);
	int GetConfig(int configItem, int &result);

	int GetValue(int operatingItem, int index, int &result);
	int GetValue(int operatingItem, int &result);

	RoboteqDevice();
	~RoboteqDevice();
};

#endif
