/*
FILENAME... iMCDriver.cpp
USAGE...    Motor driver support for the Isel Step-Controller iMC-XX.

Michael Sintschuk
April 25, 2023

*/


#include <stdio.h>
#include <string.h>
#include <sstream>
#include <stdlib.h>
#include <math.h>

#include <iostream>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "iMCDriver.h"
#include <epicsExport.h>

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new iMCController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] iMCPortName       The name of the drvAsynSerialPort that was created previously to connect to the iMC controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
iMCController::iMCController(const char *portName, const char *iMCPortName, int numAxes, double movingPollPeriod,
                             double idlePollPeriod)
    :  asynMotorController(portName, numAxes, NUM_iMC_PARAMS,
                           0, // No additional interfaces beyond those in base class
                           0, // No additional callback interfaces beyond those in base class
                           ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                           1, // autoconnect
                           0, 0)  // Default priority and stack size
{
    int axis;
    asynStatus status;
    iMCAxis *pAxis;
    static const char *functionName = "iMCController::iMCController";

    /* Connect to iMC controller */
    status = pasynOctetSyncIO->connect(iMCPortName, 0, &pasynUserController_, NULL);
    pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r", 2);
    if (status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to iMC controller\n",
                  functionName);
    }

    // Flush I/O in case there is lingering garbage
    writeReadController();

    // initialize the controller with the desired numAxes
    if (numAxes == 1) {
        // only x axis
        sprintf(outString_, "@01");
    } else if (numAxes == 2) {
        // x and y axis
        sprintf(outString_, "@03");
    } else if (numAxes == 3) {
        // x, y and z axis
        sprintf(outString_, "@07");
    } else if (numAxes == 4) {
        // x, y and z axis
        sprintf(outString_, "@07");
        status = writeReadController();
        sprintf(outString_, "@08");
    }

    status = writeReadController();

    /* endschalter low-aktiv + aktivieren */
    sprintf(outString_, "@0IE57343");
    status = writeReadController();

    /* X-Achsrichtung invertieren */
    sprintf(outString_, "@0ID13");
    status = writeReadController();

    /* bei X-Achse Endschalter vertauschen */
    sprintf(outString_, "@0Ie9");
    status = writeReadController();

    for (axis=0; axis<numAxes; axis++)
    {
        pAxis = new iMCAxis(this, axis);
    }
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new iMCController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] iMCPortName       The name of the drvSerialPort that was created previously to connect to the iMC controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int iMCCreateController(const char *portName, const char *iMCPortName, int numAxes, int movingPollPeriod,
                                   int idlePollPeriod)
{
    iMCController *piMCController
        = new iMCController(portName, iMCPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
    piMCController = NULL;
    return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void iMCController::report(FILE *fp, int level)
{
    fprintf(fp, "iMC motor driver\n");
    fprintf(fp, "  port name=%s\n", this->portName);
    fprintf(fp, "  moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "  idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Writes a string to the controller and reads the response.
  * Calls writeReadController() with default locations of the input and output strings
  * and default timeout. */
asynStatus iMCController::writeReadController()
{
    size_t nread;
    return writeReadController(outString_, inString_, sizeof(inString_), &nread, DEFAULT_CONTROLLER_TIMEOUT);
}

/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus iMCController::writeReadController(const char *output, char *input,
        size_t maxChars, size_t *nread, double timeout)
{
    size_t nwrite;
    asynStatus status;
    int eomReason;
    // const char *functionName="writeReadController";

    status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
                                         strlen(output), input, maxChars, timeout,
                                         &nwrite, nread, &eomReason);
    // the first character is the handshake character; if it is zero, don't send it as answer
    if (input[0] == '0') {
        if (27 < strlen(input)) {
            memmove(input, input, 2);
            std::cout << input << " input, not working to cut, to do ...\n";
        } else if (24 < strlen(input) && strlen(input) < 27) {
            memmove(input, input+(strlen(input) - 24), strlen(input));
        } else {
            memmove(input, input+1, strlen(input));
        }
    }

    return status;
}

/** Polls the asynMotorController (not a specific asynMotorAxis).
  * The base class asynMotorPoller thread calls this method once just before it calls asynMotorAxis::poll
  * for each axis.
  * This base class implementation does nothing.  Derived classes can implement this method if there
  * are controller-wide parameters that need to be polled.  It can also be used for efficiency in some
  * cases. For example some controllers can return the status or positions for all axes in a single
  * command.  In that case asynMotorController::poll() could read that information, and then
  * asynMotorAxis::poll() might just extract the axis-specific information from the result.
  */
asynStatus iMCController::poll()
{
    char posResponse[30];
    std::string str, x, y, z, a;
    std::stringstream ss_x, ss_y, ss_z, ss_a;

    asynStatus comStatus = asynSuccess;

    // Read the current motor position
    sprintf(outString_, "@0P");
    comStatus = writeReadController();

    //std::cout << inString_ << " inString_\n";

    if (strlen(inString_) == 24) {

        axisXdone_ = 1;
        axisYdone_ = 1;
        axisZdone_ = 1;
        axisAdone_ = 1;

        strcpy(posResponse, inString_);

        // the response-string contains positions of all axes
        // each axis has a length of 6
        str = posResponse;

        x = str.substr(0, 6);
        y = str.substr(6, 6);
        z = str.substr(12, 6);
        a = str.substr(18, 6);
        ss_x << std::hex << x;
        ss_y << std::hex << y;
        ss_z << std::hex << z;
        ss_a << std::hex << a;
        ss_x >> axisXpos_;
        ss_y >> axisYpos_;
        ss_z >> axisZpos_;
        ss_a >> axisApos_;

        /*
        // Read the limit states; this takes to long to read ...
        // don't ask for now
        sprintf(outString_, "@0DRp");
        comStatus = writeReadController();
        std::cout << inString_ << " inString_ pos limit\n";

        sprintf(outString_, "@0DRn");
        comStatus = writeReadController();
        std::cout << inString_ << " inString_ neg limit\n";
        */
    }

    return comStatus ? asynError : asynSuccess;
}


// These are the iMCAxis methods

/** Creates a new iMCAxis object.
  * \param[in] pC Pointer to the iMCController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * Initializes register numbers, etc.
  */
iMCAxis::iMCAxis(iMCController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo),
      pC_(pC)
{
    int errorFlag = 0;
    //asynStatus status;
    static const char *functionName = "iMCAxis::iMCAxis";

    // controller axes are numbered from 1
    axisIndex_ = axisNo + 1;

    callParamCallbacks();
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void iMCAxis::report(FILE *fp, int level)
{
    if (level > 0)
    {
        fprintf(fp, "  axis %d\n", axisNo_);
    }

    // Call the base class method
    asynMotorAxis::report(fp, level);
}

asynStatus iMCAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    asynStatus status;
    double relPosition;
    // static const char *functionName = "iMCAxis::move";

    // we drive only relative because of zero relative means the axis will not be touched
    if (axisIndex_ == 1) {
        relPosition = position - pC_->axisXpos_;
        sprintf(pC_->outString_, "@0A %d,%d,0,500,0,500,0,500", NINT(relPosition), NINT(maxVelocity));
        pC_->axisXdone_ = 0;
    } else if (axisIndex_ == 2) {
        relPosition = position - pC_->axisYpos_;
        sprintf(pC_->outString_, "@0A 0,500,%d,%d,0,500,0,500", NINT(relPosition), NINT(maxVelocity));
        pC_->axisYdone_ = 0;
    } else if (axisIndex_ == 3) {
        relPosition = position - pC_->axisZpos_;
        sprintf(pC_->outString_, "@0A 0,500,0,500,%d,%d,0,500", NINT(relPosition), NINT(maxVelocity));
        pC_->axisZdone_ = 0;
    } else if (axisIndex_ == 4) {
        relPosition = position - pC_->axisApos_;
        sprintf(pC_->outString_, "@0A 0,500,0,500,0,500,%d,%d", NINT(relPosition), NINT(maxVelocity));
        pC_->axisAdone_ = 0;
    }

    status = pC_->writeController();
    // stop the polling for the driving time because the controller won't answer anyway
    epicsThreadSleep(abs(NINT(relPosition)/NINT(maxVelocity)));

    return status;
}

asynStatus iMCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
    asynStatus status;

    // static const char *functionName = "iMCAxis::home";
    if (axisIndex_ < 3) {
        sprintf(pC_->outString_, "@0R%i", axisIndex_);
    } else if (axisIndex_ == 3) {
        sprintf(pC_->outString_, "@0R4");
    } else if (axisIndex_ == 4) {
        sprintf(pC_->outString_, "@0R8");
    }

    status = pC_->writeController();

    return status;
}

/** Polls the axis.
  * This function reads the motor position and the moving status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus iMCAxis::poll(bool *moving)
{
    asynStatus comStatus = asynSuccess;

    if (axisIndex_ == 1) {
        setDoubleParam(pC_->motorPosition_, pC_->axisXpos_);
        *moving = pC_->axisXdone_ ? false:true;
        setIntegerParam(pC_->motorStatusDone_, pC_->axisXdone_);
    } else if (axisIndex_ == 2) {
        setDoubleParam(pC_->motorPosition_, pC_->axisYpos_);
        *moving = pC_->axisYdone_ ? false:true;
        setIntegerParam(pC_->motorStatusDone_, pC_->axisYdone_);
    } else if (axisIndex_ == 3) {
        setDoubleParam(pC_->motorPosition_, pC_->axisZpos_);
        *moving = pC_->axisZdone_ ? false:true;
        setIntegerParam(pC_->motorStatusDone_, pC_->axisZdone_);
    } else if (axisIndex_ == 4) {
        setDoubleParam(pC_->motorPosition_, pC_->axisApos_);
        *moving = pC_->axisAdone_ ? false:true;
        setIntegerParam(pC_->motorStatusDone_, pC_->axisAdone_);
    }

    // Read the limit status
    /*sprintf(pC_->outString_, "%02d?ESTAT1", slaveID_);
    comStatus = pC_->writeReadController();
    if (comStatus) goto skip;
    // The response string is of the form "10101"
    sscanf(pC_->inString_, "%d", &limit);
    if (limit == 8)
    {
        setIntegerParam(pC_->motorStatusHighLimit_, 1);
    }
    if (limit == 1)
    {
        setIntegerParam(pC_->motorStatusLowLimit_, 1);
    }
    if (limit == 0)
    {
        setIntegerParam(pC_->motorStatusHighLimit_, 0);
        setIntegerParam(pC_->motorStatusLowLimit_, 0);
    }*/

    // for now we can't see if there is a controller problem
    //setIntegerParam(pC_->motorStatusProblem_, 0);
    callParamCallbacks();
    return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg iMCCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg iMCCreateControllerArg1 = {"Controller port name", iocshArgString};
static const iocshArg iMCCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg iMCCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg iMCCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const iMCCreateControllerArgs[] = {&iMCCreateControllerArg0,
                                                            &iMCCreateControllerArg1,
                                                            &iMCCreateControllerArg2,
                                                            &iMCCreateControllerArg3,
                                                            &iMCCreateControllerArg4,
                                                           };
static const iocshFuncDef iMCCreateControllerDef = {"iMCCreateController", 5, iMCCreateControllerArgs};
static void iMCCreateContollerCallFunc(const iocshArgBuf *args)
{
    iMCCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void iMCRegister(void)
{
    iocshRegister(&iMCCreateControllerDef, iMCCreateContollerCallFunc);
}

extern "C" {
    epicsExportRegistrar(iMCRegister);
}
