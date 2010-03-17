# Windows-specific libraries for GraspIt!. Included from graspit.pro - not for standalone use.

# ---------------------- General libraries and utilities ----------------------------------

LIBS	+= "$(MKLDIR)/ia32/lib/mkl_c_dll.lib"

graspitdbg {
	LIBS += qhull/windows/Debug/qhull.lib $(COINDIR)/lib/Coin2d.lib $(COINDIR)/lib/SoQt1d.lib
} else {
	LIBS += qhull/windows/Release/qhull.lib $(COINDIR)/lib/Coin2.lib $(COINDIR)/lib/SoQt1.lib
}

DEFINES	+= COIN_DLL SOQT_DLL MKL WIN32

INCLUDEPATH += "$(MKLDIR)/include"

#------------------------------------ add-ons --------------------------------------------

mosek {
	!exists($(MOSEK5_0_INSTALLDIR)) {
		error("Mosek not installed or MOSEK5_0_INSTALLDIR environment variable not set")
	}
	INCLUDEPATH += $(MOSEK5_0_INSTALLDIR)/tools/platform/win/h
	#no separate debug or release versions of the lib
	LIBS += $(MOSEK5_0_INSTALLDIR)/tools/platform/win/dll/mosek5_0.lib
}

cgal_qp {
	!exists($(CGAL_DIR)) {
		error("CGAL not installed or CGAL environment variable not set")
	}
	INCLUDEPATH += $(CGAL_DIR)/include
	graspitdbg {
		LIBS += $(CGAL_DIR)/lib/CGAL-vc71-mt-gd.lib
	} else {
		LIBS += $(CGAL_DIR)/lib/CGAL-vc71-mt.lib
	}
}

boost {
	!exists($(BOOST_ROOT)) {
		error("Boost not installed or BOOST_ROOT environment variable not set")
	}
	INCLUDEPATH += $(BOOST_ROOT)
	graspitdbg {
		LIBS += $(BOOST_ROOT)/lib/libboost_thread-vc71-mt-gd-1_37.lib
	} else {
		LIBS += $(BOOST_ROOT)/lib/libboost_thread-vc71-mt-1_37.lib
	}
}

hardwarelib {
	DEFINES += HARDWARE_LIB
	graspitdbg {
		LIBS += hardware/Debug/HardwareLib.lib
	} else {
		LIBS += hardware/Release/HardwareLib.lib
	}
	INCLUDEPATH += hardware
}
