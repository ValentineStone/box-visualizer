#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "utils.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void mouseReleaseEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);

private:
    Ui::MainWindow *ui;


    rs2::config rs_config;
    rs2::pipeline rs_pipeline;
    rs2::pipeline_profile rs_profile;
    WorkerThread *workerThread;

};
#endif // MAINWINDOW_H
