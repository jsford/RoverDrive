#include <iostream>
#include <stdio.h>
#include <string.h>
#include <cmath>

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

    // Disable WatchDog
    cout << device.SetConfig(_RWD, 0) << endl;

    // Disable Loop Error Detection
    cout << device.SetConfig(_CLERD, 1, 0) << endl;

    // Configure Motor 1
    {
        // Set Operating Mode to Closed-Loop Position Tracking
        cout << device.SetConfig(_MMOD, 1, 4) << endl;

        // Set Position Mode Velocity (0 RPM)
        cout << device.SetConfig(_MVEL, 1, 1000) << endl;

        // Set Turns Min to Max (0.01 RPM)
        cout << device.SetConfig(_MXTRN, 1, 202500) << endl;

        // Set KP (0.1 ul)
        cout << device.SetConfig(_KP, 1, 160) << endl;

        // Set KI (0.1 ul)
        cout << device.SetConfig(_KI, 1, 0) << endl;

        // Set KD (0.1 ul)
        cout << device.SetConfig(_KD, 1, 0) << endl;
    }

    // Configure AIN4 as Motor 1 Feedback
    {
        // Set AIN4 Use to Feedback for Motor 1
        cout << device.SetConfig(_AINA, 4, 18) << endl;
        // Set AIN4 Conversion Type to Absolute
        cout << device.SetConfig(_AMOD, 4, 1) << endl;
        // Set AIN4 Input Min (mV)
        cout << device.SetConfig(_AMIN, 4, 1100) << endl;
        // Set AIN4 Input Center (mV)
        cout << device.SetConfig(_ACTR, 4, 2380) << endl;
        // Set AIN4 Input Max (mV)
        cout << device.SetConfig(_AMAX, 4, 3665) << endl;
    }

    // Configure DIN5 and DIN6 as Limit Switches
    {
        // Set Active Level High
        cout << device.SetConfig(_DINL, 5, 0) << endl;
        cout << device.SetConfig(_DINL, 6, 0) << endl;
        // Set DIN5 Action to Forward Limit Switch
        cout << device.SetConfig(_DINA, 5, 4+16) << endl;
        // Set DIN6 Action to Reverse Limit Switch
        cout << device.SetConfig(_DINA, 6, 5+16) << endl;
    }

    double tilt_deg = -45;

    while(1) {
        cout << "GO! " << device.SetCommand(_MOTCMD, tilt_deg*1000.0/90.0) << endl;
        while(1) {
            int pos;
            device.GetValue(_F, 1, pos);
            double d = pos * 90.0/1000.0;
            cout << d << endl;
            if(abs(d-tilt_deg) <= 2.0) { break; }
            sleepms(100);
        }
        sleepms(1000);
        tilt_deg *= -1;
    }

	device.Disconnect();
	return 0;
}

int main(int argc, char *argv[])
{
	string port = "/dev/roboteq2";
	return MotorControllerSample(port);
}
