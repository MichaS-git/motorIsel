# motorIsel
EPICS motor drivers for the following [Isel](https://www.isel.com/en/multiple-axis-controller-imc-s8.html) controller: Step-Controller iMC-XX

motorOwis is a submodule of [motor](https://github.com/epics-modules/motor).  When motorOwis is built in the ``motor/modules`` directory, no manual configuration is needed.

motorOwis can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorOwis contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
