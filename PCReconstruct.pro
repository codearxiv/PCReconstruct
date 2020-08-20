#-------------------------------------------------
#
# Project created by QtCreator 2019-09-28T10:50:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PCReconstruct
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

PRECOMPILED_HEADER  = stable.h

SOURCES += \
	BoundBox.cpp \
	Cloud.cpp \
	CloudWorker.cpp \
	GLWidget.cpp \
	MainWindow.cpp \
	MessageLogger.cpp \
	Window.cpp \
	dialogs/DecimateDialog.cpp \
	dialogs/ReconstructDialog.cpp \
	dialogs/SetRandomDialog.cpp \
	dialogs/SparsifyDialog.cpp \
	dictionarylearning/MatchingPursuit.cpp \
	dictionarylearning/OrthogonalPursuit.cpp \
	dictionarylearning/cosine_transform.cpp \
	dictionarylearning/ksvd.cpp \
	dictionarylearning/ksvd_dct2D.cpp \
	main.cpp \
	utils/Plane.cpp \
	utils/cloud_normal.cpp \
	utils/pt_to_pt_distsq.cpp \
	utils/rotations.cpp

HEADERS += \
	BoundBox.h \
	Cloud.h \
	CloudWorker.h \
	Cover-Tree/CoverTreePoint.h \
	Cover-Tree/Cover_Tree.h \
	GLWidget.h \
	MainWindow.h \
	MessageLogger.h \
	Window.h \
	alignment.h \
	constants.h \
	dialogs/DecimateDialog.h \
	dialogs/ReconstructDialog.h \
	dialogs/SetRandomDialog.h \
	dialogs/SparsifyDialog.h \
	dictionarylearning/MatchingPursuit.h \
	dictionarylearning/OrthogonalPursuit.h \
	dictionarylearning/cosine_transform.h \
	dictionarylearning/ksvd.h \
	dictionarylearning/ksvd_dct2D.h \
	stable.h \
	\         \
	utils/Plane.h \
	utils/cloud_normal.h \
	utils/ensure_buffer_size.h \
	utils/pt_to_pt_distsq.h \
	utils/rotations.h

FORMS +=

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += $$PWD/'../../lib/PCL 1.9.1/include/pcl-1.9' \
		   $$PWD/'../../lib/eigen3' \
		   $$PWD/'../../lib/Boost/include/boost-1_68' \
		   $$PWD/'../../lib/OpenNI2/Include' \
		   $$PWD/'../../lib/FLANN/include' \
		   $$PWD/'Cover-Tree' \
		   $$PWD/'dialogs' \
		   $$PWD/'dictionarylearning' \
		   $$PWD/'utils'

LIBS += -L$$PWD/'../../lib/PCL 1.9.1/lib/'
CONFIG( debug, debug|release ) {
## debug
LIBS += \
	-lpcl_common_debug \
	-lpcl_filters_debug \
	-lpcl_kdtree_debug \
	-lpcl_search_debug \
	-lpcl_io_debug \
	-lpcl_io_ply_debug
}
else {
# release
LIBS += \
	-lpcl_common_release \
	-lpcl_filters_release \
	-lpcl_kdtree_release \
	-lpcl_search_release \
	-lpcl_io_release \
	-lpcl_io_ply_release
}

LIBS += -L$$PWD/'../../lib/Boost/lib/'
LIBS += -L$$PWD/'../../lib/OpenNI2/Lib/' -lOpenNI2

RESOURCES += \
	PCReconstruct.qrc

msvc{
    QMAKE_CXXFLAGS += -openmp
}

gcc{
    QMAKE_CXXFLAGS += -fopenmp -Wno-attributes -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-ignored-attributes
    QMAKE_LFLAGS += -fopenmp
    LIBS += -fopenmp
}
