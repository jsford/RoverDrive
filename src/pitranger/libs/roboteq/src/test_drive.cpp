#include <iostream>
#include <stdio.h>
#include <string.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;
using namespace roboteq;


int MotorControllerSample(string port) 
{
	cout << endl << "Motor Controller Sample:" << endl;
	cout << "------------------------" << endl;
	string response = "";
	RoboteqDevice device;
	int status = device.Connect(port);
    int result = 0;

    // Check Connection Success
	if (status != RQ_SUCCESS)
	{
		cout << "Error connecting to device: " << status << "." << endl;
		return 1;
	}
	sleepms(10);

    // Set Operating Mode to Closed-Loop Speed
    cout << device.SetConfig(_MMOD, 1, 1) << endl;
    sleepms(10);

    // Set Encoder Mode to Feedback
    cout << device.SetConfig(_EMOD, 1, 2) << endl;
    sleepms(10);

    // Set Encoder Pulses Per Revolution
    cout << device.SetConfig(_EPPR, 1, 28) << endl;
    sleepms(10);

    // Set Max. Motor RPM
    cout << device.SetConfig(_MXRPM, 1, 5640) << endl;
    sleepms(10);

    // Set Motor Accel. in 0.1 RPM/sec.
    cout << device.SetConfig(_MAC, 1, 56400) << endl;
    sleepms(10);

    // Set Motor Decel. in 0.1 RPM/sec.
    cout << device.SetConfig(_MDEC, 1, 56400) << endl;
    sleepms(10);

    // Set KP to 1.1
    cout << device.SetConfig(_KP, 1, 11) << endl;
    sleepms(10);

    // Set KI to 1.0
    cout << device.SetConfig(_KI, 1, 10) << endl;
    sleepms(10);

    // Set KD to 0.0
    cout << device.SetConfig(_KD, 1, 0) << endl;
    sleepms(10);

    // Disable WatchDog
    cout << device.SetConfig(_RWD, 0) << endl;
    sleepms(10);


    while(1) {
        // Go Max Forward for 5 seconds
        cout << device.SetCommand(_MOTCMD, 1, 1000) << endl;
        sleepms(5000);
        // Go Max Reverse for 5 seconds
        cout << device.SetCommand(_MOTCMD, 1, -1000) << endl;
        sleepms(5000);
    }
    // Stop!
    cout << device.SetCommand(_MOTCMD, 1, 0) << endl;

	device.Disconnect();
	return 0;
}

int main(int argc, char *argv[])
{
	string port = "/dev/roboteq1";
	return MotorControllerSample(port);
}
