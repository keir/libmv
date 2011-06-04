// Copyright 2011 libmv authors
#ifndef SRC_UI_TRACKER_TRACKER_H_
#define SRC_UI_TRACKER_TRACKER_H_
#include <QApplication>
#include <QMainWindow>
#include <QToolBar>
#include <QSpinBox>
#include <QAction>
#include <QSlider>
#include <QLabel>
#include <QTimer>
namespace libmv {
class KLT;
class ImagePyramid;
}

class Tracker : public QMainWindow {
    Q_OBJECT
public:
    Tracker();
public slots:
    void open();
    void seek(int);
    void first();
    void previous();
    void next();
    void last();
    void togglePlay(bool);
    void start();
    void stop();
private:
    void open(QStringList);

    QSpinBox frameNumber;
    QAction* playAction;
    QSlider slider;
    QLabel view;
    QTimer playTimer;
    QList<QString> images;
    int current;

    libmv::KLT* klt;
    QVector<libmv::ImagePyramid*> pyramids;
};
#endif
