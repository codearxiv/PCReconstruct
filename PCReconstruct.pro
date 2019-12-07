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
        Cover-Tree/CoverTreePoint.cpp \
        cloud.cpp \
        dictionarylearning/MatchingPursuit.cpp \
        dictionarylearning/OrthogonalPursuit.cpp \
        dictionarylearning/cosine_transform.cpp \
        dictionarylearning/ksvd.cpp \
        dictionarylearning/ksvd_dct2D.cpp \
        glwidget.cpp \
        main.cpp \
        mainwindow.cpp \
        release/moc_glwidget.cpp \
        release/moc_mainwindow.cpp \
        release/moc_window.cpp \
        utils/pt_to_pt_distsq.cpp \
        window.cpp

HEADERS += \
        Cover-Tree/CoverTreePoint.h \
        Cover-Tree/Cover_Tree.h \
        cloud.h \
        constants.h \
        dictionarylearning/MatchingPursuit.h \
        dictionarylearning/OrthogonalPursuit.h \
        dictionarylearning/cosine_transform.h \
        dictionarylearning/ksvd.h \
        dictionarylearning/ksvd_dct2D.h \
        glwidget.h \
        mainwindow.h \
        release/moc_predefs.h \
        stable.h \
        ui_mainwindow.h \         \
        utils/ensure_buffer_size.h \
        utils/pt_to_pt_distsq.h
        window.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
#qnx: target.path = /tmp/$${TARGET}/bin
#else: unix:!android: target.path = /opt/$${TARGET}/bin
#!isEmpty(target.path): INSTALLS += target
# Default rules for deployment.
target.path = ..\workspace4\PCReconstruct
INSTALLS += target

INCLUDEPATH += $$PWD/'../lib/PCL 1.9.1/include/pcl-1.9' \
	       $$PWD/'../lib/eigen3' \
	       $$PWD/'../lib/Boost/include/boost-1_68' \
	       $$PWD/'../lib/OpenNI2/Include' \
		   $$PWD/'../lib/FLANN/include' \
		   $$PWD/'Cover-Tree' \
		   $$PWD/'dictionarylearning' \
		   $$PWD/'utils'

#DEPENDPATH += $$PWD/'../lib/PCL 1.9.1/include/pcl-1.9'
#DEPENDPATH += $$PWD/'../lib/eigen3'


LIBS += -L$$PWD/'../lib/PCL 1.9.1/lib/'
LIBS += \
	-lpcl_common_release \
	-lpcl_filters_release \
	-lpcl_kdtree_release \
	-lpcl_search_release \
	-lpcl_io_release \
	-lpcl_io_ply_release
LIBS += -L$$PWD/'../lib/Boost/lib/'
LIBS += -L$$PWD/'../lib/OpenNI2/Lib/' -lOpenNI2


#PRE_TARGETDEPS += $$PWD/'../lib/PCL 1.9.1/lib/pcl_io_release.lib'
#PRE_TARGETDEPS += $$PWD/'../lib/PCL 1.9.1/lib/pcl_io_ply_release.lib'
#PRE_TARGETDEPS += $$PWD/'../lib/PCL 1.9.1/lib/pcl_common_release.lib'
#PRE_TARGETDEPS += $$PWD/'../lib/OpenNI2/Lib/OpenNI2.lib'



