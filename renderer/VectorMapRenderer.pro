#-------------------------------------------------
#
# Project created by QtCreator 2015-07-06T11:28:41
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = VectorMapRenderer
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    ../libvectormap/src/vector_map.cpp

HEADERS  += mainwindow.h \
    ../libvectormap/include/Math.h \
    ../libvectormap/include/vector_map.h

FORMS    += mainwindow.ui
