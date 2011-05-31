// Copyright (c) 2011 libmv authors.
// Initial revision by Matthias Fauconneau.
#include "tracker.h"
#include "klt.h"
#include <QFileDialog>
#include <QDesktopWidget>
#include <QDebug>
int main(int argc, char *argv[]) {
    QApplication app(argc, argv); Tracker window; window.show(); return app.exec();
}
Tracker::Tracker() : klt(0), current(-1) {
    setWindowTitle("Qt Tracker");
    QToolBar* toolBar = addToolBar("Main Toolbar"); toolBar->setObjectName("mainToolbar");
    toolBar->addAction(QIcon::fromTheme("document-open"),"Open...",this,SLOT(open()));
    toolBar->addAction(QIcon::fromTheme("media-skip-backward"),"First Frame",this,SLOT(first()));
    toolBar->addAction(QIcon::fromTheme("media-seek-backward"),"Previous Frame",this,SLOT(previous()))->setShortcut(QKeySequence("Left"));
    toolBar->addWidget( &frameNumber );
    connect(&frameNumber,SIGNAL(valueChanged(int)),SLOT(seek(int)));
    playAction = toolBar->addAction(QIcon::fromTheme("media-playback-start"),QKeySequence("Play"));
    playAction->setCheckable(true); playAction->setShortcut(QKeySequence("Space"));
    connect(playAction,SIGNAL(triggered(bool)),SLOT(togglePlay(bool)));
    toolBar->addWidget( &slider ); slider.setOrientation(Qt::Horizontal);
    connect(&slider,SIGNAL(sliderMoved(int)),SLOT(seek(int)));
    toolBar->addAction(QIcon::fromTheme("media-seek-forward"),"Next Frame",this,SLOT(next()))->setShortcut(QKeySequence("Right"));
    toolBar->addAction(QIcon::fromTheme("media-skip-forward"),"Last Frame",this,SLOT(last()));
    view.setScaledContents(true); view.setMaximumSize(qApp->desktop()->availableGeometry().size());
    setCentralWidget(&view);
    connect(&playTimer,SIGNAL(timeout()),SLOT(next()));
    QStringList args = qApp->arguments(); args.removeFirst();
    if(args.isEmpty()) view.setText("No image sequence selected"); else open(args);
}

void Tracker::open() { open(QFileDialog::getOpenFileNames(this, tr("Select Images"),"","Images (*.png *.jpg);;All Files (*.*)")); }
void Tracker::open(QStringList paths) {
    if(paths.isEmpty()) return;
    images.clear();
    foreach(QString path,paths) {
        if(QFileInfo(path).isFile()) images << path;
        else foreach(QString file,QDir(path).entryList(QStringList("*.jpg")<<"*.png",QDir::Files,QDir::Name)) images << QDir(path).filePath(file);
    }
    frameNumber.setMaximum(images.count()-1);
    slider.setMaximum(images.count()-1);
    first();
    std::vector<std::string> files; foreach(QString image,images) files.push_back(image.toStdString());
    //klt=new KLT(files);
}

void Tracker::seek(int frame) {
    if(frame<0 || frame==current || frame>=images.count()) { qDebug()<<frame<<current; stop(); return; }
    view.setPixmap(QPixmap(images[current=frame]));
    slider.setValue(frame); frameNumber.setValue(frame);
}
void Tracker::first() { seek(0); }
void Tracker::previous() { seek(current-1); }
void Tracker::next() { seek(current+1); }
void Tracker::last() { seek(images.count()-1); }

void Tracker::start() { playAction->setChecked(true); playTimer.start(100); }
void Tracker::stop() { playAction->setChecked(false); playTimer.stop(); }
void Tracker::togglePlay(bool play) { if(play) start(); else stop(); }
