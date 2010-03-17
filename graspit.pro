TEMPLATE	= app
LANGUAGE	= C++

#-------------------------options--------------------------

#comment out this line for compiling in Release mode
#usually, compiling in Release mode delivers a significant gain in performance
#in MS Visual Studio *also* set the project mode to Release
CONFIG += graspitdbg

#select collision detection engine
COLLISION = graspit_collision
CONFIG += $$COLLISION

#link against the Hardware library (included with this distribution) which 
#provides access to various hardware. Hardware library must be compiled
#separately and is Windows-only.
#CONFIG += hardwarelib

#link against Mosek QP solver (must be installed separately)
#CONFIG += mosek

#enable the CGAL QP solver (must be installed separately)
#also needs boost
#CONFIG += cgal_qp

#enable linking against boost (must be installed separately)
#CONFIG += boost

#------------------system-specific libraries---------------

graspitdbg {
    CONFIG += debug
} else {
    CONFIG += release
	DEFINES += GRASPIT_RELEASE
}

win32 {
	include(graspit-lib-WINDOWS.pro)
} else {
	include(graspit-lib-LINUX.pro)
}

#------------------GraspIt! core files---------------------

include(graspit-core.pro)
