# Linux-specific libraries for GraspIt!. Included from graspit.pro - not for standalone use.

# ---------------------- General libraries and utilities ----------------------------------

LIBS	+= -lblas -llapack -Lqhull -lqhull -L$(COINDIR)/lib -lSoQt -lCoin -lGL -lpthread

MOC_DIR = .moc
OBJECTS_DIR = .obj

#------------------------------------ add-ons --------------------------------------------

mosek {
	error("Mosek linking only tested under Windows")
}

cgal_qp {
	error("CGAL linking only tested under Windows")
}

boost {
	error("Boost linking only tested under Windows")
}

hardwarelib {
	error("Hardware library only available under Windows")
}
