#pragma once
#include <QApplication>
#include <QMainWindow>
#include <QToolBar>
#include <QSpinBox>
#include <QAction>
#include <QSlider>
#include <QLabel>
#include <QTimer>
class KLT;

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
    KLT* klt;
};
