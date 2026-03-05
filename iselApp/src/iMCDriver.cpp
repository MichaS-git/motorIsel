/*
FILENAME... iMCDriver.cpp
USAGE...    Motor driver support for the Isel Step-Controller iMC-XX.

Michael Sintschuk
April 25, 2023

*/

#include <string.h>
#include <sstream>
#include <cmath>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsTime.h>

#include "iMCDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

/** Creates a new iMCController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] iMCPortName       The name of the drvAsynSerialPort that was created previously to connect to the iMC controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  * \param[in] initCmds          Possible List with initialization commands for the controller
  */
iMCController::iMCController(const char *portName, const char *iMCPortName, int numAxes, double movingPollPeriod,
                             double idlePollPeriod, const char *initCmds)
    :  asynMotorController(portName, numAxes,
                           0, // No controller parameters
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
    pasynOctetSyncIO->setOutputEos(pasynUserController_, "\r", 1);
    if (status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to iMC controller\n",
                  functionName);
    }

    // there is no end-of-line-termination
    // only one byte comes back, except when asking position
    expectedBytes_ = 1;

    totalAxesNum_ = numAxes;

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

    writeReadController();

    // go through the init commands and send them to controller
    if (initCmds) {
        std::string str = initCmds;
        std::stringstream text_stream(initCmds);
        std::string item;

        while (std::getline(text_stream, item, ',')) {
            sprintf(outString_, item.c_str());
            writeReadController();
        }
    }

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
  * \param[in] initCmds          Possible List with initialization commands for the controller
  */
extern "C" int iMCCreateController(const char *portName, const char *iMCPortName, int numAxes, int movingPollPeriod,
                                   int idlePollPeriod, const char *initCmds)
{
    iMCController *piMCController
        = new iMCController(portName, iMCPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000., initCmds);
    piMCController = NULL;
    return(asynSuccess);
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

    // after a movement we expect 1 byte, the handshake of the movement
    // just get it
    if (afterMovement_) {
        timeout = movingTime_;
        pasynOctetSyncIO->read(pasynUserController_, input, 1, timeout, nread, &eomReason);
        afterMovement_ = false;
    }

    maxChars = expectedBytes_;
    status = pasynOctetSyncIO->writeRead(pasynUserController_, output,
                                         strlen(output), input, maxChars, timeout,
                                         &nwrite, nread, &eomReason);

    // expect at least 1 byte as answer (handshake)
    if (*nread < 1) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "iMC: empty response... wait while moving!\n");
        return asynError;
    }

    if (!status) {
        // the first character is the handshake character; if it is zero, everything is fine
        if (input[0] == '0') {
            // omit the handshake character
            memmove(input, input+1, strlen(input));
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "ERROR, handshake character returned: %c\n", input[0]);
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
    char posResponse[32];
    std::stringstream ss_x, ss_y, ss_z, ss_a;

    asynStatus comStatus = asynSuccess;

    // when one of the axis is moving, calculate the current position
    if (moveActive_) {

        epicsTimeStamp now;
        epicsTimeGetCurrent(&now);
        double elapsed = epicsTimeDiffInSeconds(&now, &moveStartTime_);
        int delta = static_cast<int>(direction_ * moveVelocity_ * elapsed);

        if (elapsed >= movingTime_) {
            moveActive_ = false;
            return asynSuccess;
        }

        if (!axisXdone_) {
            axisXpos_ = moveStartPos_ + delta;
        } else if (!axisYdone_) {
            axisYpos_ = moveStartPos_ + delta;
        } else if (!axisZdone_) {
            axisZpos_ = moveStartPos_ + delta;
        } else if (!axisAdone_) {
            axisApos_ = moveStartPos_ + delta;
        }

    // when no axis is moving, ask the controller about the position
    } else {

        // if the controller has 4 axes, it returns all 4 positions: 24 bytes + 1 byte handshake
        // else, it returns always 3 positions 18 bytes + 1 byte handshake
        if (totalAxesNum_ < 4) expectedBytes_ = 19;
        else expectedBytes_ = 25;

        // Read the current motor position
        sprintf(outString_, "@0P");
        comStatus = writeReadController();

        if (comStatus) return comStatus;

        strcpy(posResponse, inString_);

        std::string str = posResponse;

        auto parseAxis = [](const std::string &hexStr) -> int32_t {
            int32_t value = 0;
            std::stringstream ss;
            ss << std::hex << hexStr;
            ss >> value;

            // 24-bit Vorzeichenbit erkennen und auf 32 Bit erweitern
            if (value & 0x800000)
                value |= 0xFF000000;

            return value;
        };

        if (totalAxesNum_ == 4) {
            axisXdone_ = axisYdone_ = axisZdone_ = axisAdone_ = 1;
            axisXpos_ = parseAxis(str.substr(0,6));
            axisYpos_ = parseAxis(str.substr(6,6));
            axisZpos_ = parseAxis(str.substr(12,6));
            axisApos_ = parseAxis(str.substr(18,6));
        } else {  // 1 or 2 or 3 axis
            axisXdone_ = axisYdone_ = axisZdone_ = 1;
            axisXpos_ = parseAxis(str.substr(0,6));
            axisYpos_ = parseAxis(str.substr(6,6));
            axisZpos_ = parseAxis(str.substr(12,6));
        }
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
    // controller axes are numbered from 1
    axisIndex_ = axisNo + 1;

    callParamCallbacks();
}

asynStatus iMCAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    asynStatus comStatus = asynSuccess;
    double relPosition=0;

    if (pC_->moveActive_) return comStatus;

    // we drive only relative because of zero relative means the axis will not be touched
	if (pC_->totalAxesNum_ == 4 || pC_->totalAxesNum_ == 3) {
		if (axisIndex_ == 1) {
			relPosition = position - pC_->axisXpos_;
			sprintf(pC_->outString_, "@0A %d,%d,0,500,0,500,0,500", NINT(relPosition), NINT(maxVelocity));
			pC_->axisXdone_ = 0;
            pC_->moveStartPos_ = static_cast<int>(pC_->axisXpos_);
		} else if (axisIndex_ == 2) {
			relPosition = position - pC_->axisYpos_;
			sprintf(pC_->outString_, "@0A 0,500,%d,%d,0,500,0,500", NINT(relPosition), NINT(maxVelocity));
			pC_->axisYdone_ = 0;
            pC_->moveStartPos_ = static_cast<int>(pC_->axisYpos_);
		} else if (axisIndex_ == 3) {
			relPosition = position - pC_->axisZpos_;
			sprintf(pC_->outString_, "@0A 0,500,0,500,%d,%d,0,500", NINT(relPosition), NINT(maxVelocity));
			pC_->axisZdone_ = 0;
            pC_->moveStartPos_ = static_cast<int>(pC_->axisZpos_);
		} else if (axisIndex_ == 4) {
			relPosition = position - pC_->axisApos_;
			sprintf(pC_->outString_, "@0A 0,500,0,500,0,500,%d,%d", NINT(relPosition), NINT(maxVelocity));
			pC_->axisAdone_ = 0;
            pC_->moveStartPos_ = static_cast<int>(pC_->axisApos_);
		}
	} else if (pC_->totalAxesNum_ == 2) {
		if (axisIndex_ == 1) {
			relPosition = position - pC_->axisXpos_;
			sprintf(pC_->outString_, "@0A %d,%d,0,500", NINT(relPosition), NINT(maxVelocity));
			pC_->axisXdone_ = 0;
            pC_->moveStartPos_ = static_cast<int>(pC_->axisXpos_);
		} else if (axisIndex_ == 2) {
			relPosition = position - pC_->axisYpos_;
			sprintf(pC_->outString_, "@0A 0,500,%d,%d", NINT(relPosition), NINT(maxVelocity));
			pC_->axisYdone_ = 0;
            pC_->moveStartPos_ = static_cast<int>(pC_->axisYpos_);
		}
    } else if (pC_->totalAxesNum_ == 1) {
        relPosition = position - pC_->axisXpos_;
        sprintf(pC_->outString_, "@0A %d,%d", NINT(relPosition), NINT(maxVelocity));
        pC_->axisXdone_ = 0;
        pC_->moveStartPos_ = static_cast<int>(pC_->axisXpos_);
        }

    pC_->expectedBytes_ = 1;
    pC_->movingTime_ = abs(relPosition/maxVelocity);
    pC_->direction_ = (NINT(relPosition) > 0) - (NINT(relPosition) < 0);
    pC_->moveVelocity_ = NINT(maxVelocity);
    pC_->moveActive_ = true;
    comStatus = pC_->writeController();
    epicsTimeGetCurrent(&pC_->moveStartTime_);
    pC_->afterMovement_ = true;

    return comStatus ? asynError : asynSuccess;
}

asynStatus iMCAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
    asynStatus comStatus = asynSuccess;

    if (axisIndex_ < 3) {
        sprintf(pC_->outString_, "@0R%i", axisIndex_);
        if (axisIndex_ == 1) {
            pC_->axisXdone_ = 0;
        } else if (axisIndex_ == 2) {
            pC_->axisYdone_ = 0;
        }
    } else if (axisIndex_ == 3) {
        sprintf(pC_->outString_, "@0R4");
        pC_->axisZdone_ = 0;
    } else if (axisIndex_ == 4) {
        sprintf(pC_->outString_, "@0R8");
        pC_->axisAdone_ = 0;
    }

    pC_->expectedBytes_ = 1;
    // homing could take a while, set a long timeout
    pC_->movingTime_ = 999;
    comStatus = pC_->writeController();
    pC_->afterMovement_ = true;

    return comStatus ? asynError : asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position and the moving status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus iMCAxis::poll(bool *moving)
{
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

    callParamCallbacks();
    return asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg iMCCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg iMCCreateControllerArg1 = {"Controller port name", iocshArgString};
static const iocshArg iMCCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg iMCCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg iMCCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg iMCCreateControllerArg5 = {"Controller Init Commands", iocshArgString};
static const iocshArg * const iMCCreateControllerArgs[] = {&iMCCreateControllerArg0,
                                                            &iMCCreateControllerArg1,
                                                            &iMCCreateControllerArg2,
                                                            &iMCCreateControllerArg3,
                                                            &iMCCreateControllerArg4,
                                                            &iMCCreateControllerArg5,
                                                           };
static const iocshFuncDef iMCCreateControllerDef = {"iMCCreateController", 6, iMCCreateControllerArgs};
static void iMCCreateContollerCallFunc(const iocshArgBuf *args)
{
    iMCCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval);
}

static void iMCRegister(void)
{
    iocshRegister(&iMCCreateControllerDef, iMCCreateContollerCallFunc);
}

extern "C" {
    epicsExportRegistrar(iMCRegister);
}
