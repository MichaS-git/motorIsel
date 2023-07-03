/*
FILENAME...   iMCDriver.h
USAGE...      Motor driver support for the Isel Step-Controller iMC-XX.

Michael Sintschuk
April 25, 2023

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

// No controller-specific parameters yet
#define NUM_iMC_PARAMS 0

class iMCAxis : public asynMotorAxis
{
public:
    /* These are the methods we override from the base class */
    iMCAxis(class iMCController *pC, int axis);
    void report(FILE *fp, int level);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus poll(bool *moving);

private:
    iMCController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
    int axisIndex_;    /* Numbered from 1 */

    friend class iMCController;
};

class iMCController : public asynMotorController
{
public:
    iMCController(const char *portName, const char *iMCPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

    /* These are the methods that we override from asynMotorDriver */
    void report(FILE *fp, int level);
    asynStatus writeReadController();
    asynStatus writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);
    asynStatus poll();

private:
    int done_;
    int axisXpos_;
    int axisYpos_;
    int axisZpos_;
    int axisApos_;
    int axisXdone_;
    int axisYdone_;
    int axisZdone_;
    int axisAdone_;

    friend class iMCAxis;
};
