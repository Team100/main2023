# Control Lib

This is an implementation of full-state feedback control, with three notable features

* Nonlinear sytems: no matrices here. :-)
* Intrinsic geometry: geometry is part of type, rather than something you have to remember to correct for, e.g. in the case of rotation.
* Bitemporality: asynchronous, late-arriving measurements are handled correctly.

There's a utility class for calculating LQR gains if you want to use them, but the gains are up to you.