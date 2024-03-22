#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "utils.h"

void uint16_t_cleanupFunction(void *info) {
    uint16_t* buff = (uint16_t*) info;
    delete[] buff;
}

void uint8_t_cleanupFunction(void *info) {
    uint8_t* buff = (uint8_t*) info;
    delete[] buff;
}

void rgb_pixel_t_cleanupFunction(void *info) {
    rgb_pixel_t* buff = (rgb_pixel_t*) info;
    delete[] buff;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setMouseTracking(true);

    workerThread = new WorkerThread(this);


    workerThread->rotateA = ui->sliderA->value() * PI / 180;
    workerThread->rotateB = ui->sliderB->value() * PI / 180;
    workerThread->rotateC = ui->sliderC->value() * PI / 180;
    workerThread->zoomScale = ui->sliderD->value() / 100.;
    workerThread->sliceLevel = ui->sliderE->value() / 1000.;

    connect(workerThread, &WorkerThread::resultReady1, this, [this](uint16_t* buff, size_t w, size_t h, bool cleanup){
        uint16_t* copy = new uint16_t[w*h];
        memcpy(copy, buff, w*h*2);
        if (cleanup) delete[] buff;
        auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_Grayscale16, uint16_t_cleanupFunction, copy));
        ui->image1Label->setPixmap(pixmap);
    });
    connect(workerThread, &WorkerThread::resultReady2, this, [this](uint16_t* buff, size_t w, size_t h, bool cleanup){
        uint16_t* copy = new uint16_t[w*h];
        memcpy(copy, buff, w*h*2);
        if (cleanup) delete[] buff;
        auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_Grayscale16, uint16_t_cleanupFunction, copy));
        ui->image2Label->setPixmap(pixmap);
    });
    connect(workerThread, &WorkerThread::resultReady3, this, [this](uint8_t* buff, size_t w, size_t h, bool cleanup){
        uint8_t* copy = new uint8_t[w*h*3];
        memcpy(copy, buff, w*h*3);
        if (cleanup) delete[] buff;
        auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_RGB888, uint8_t_cleanupFunction, copy));
        ui->image3Label->setPixmap(pixmap);
    });
    connect(workerThread, &WorkerThread::resultReady, this, [this](QString label, void* buff, size_t w, size_t h, bool cleanup){
        if (label == "label1") {
            uint16_t* copy = new uint16_t[w*h];
            memcpy(copy, buff, w*h*2);
            if (cleanup) delete[] (uint16_t*)buff;
            auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_Grayscale16, uint16_t_cleanupFunction, copy));
            ui->image1Label->setPixmap(pixmap);
        } else if (label == "label2") {
            uint16_t* copy = new uint16_t[w*h];
            memcpy(copy, buff, w*h*2);
            if (cleanup) delete[] (uint16_t*)buff;
            auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_Grayscale16, uint16_t_cleanupFunction, copy));
            ui->image2Label->setPixmap(pixmap);
        } else if (label == "label3") {
            uint8_t* copy = new uint8_t[w*h*3];
            memcpy(copy, buff, w*h*3);
            if (cleanup) delete[] (uint8_t*)buff;
            auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_RGB888, uint8_t_cleanupFunction, copy));
            ui->image3Label->setPixmap(pixmap);
        } else if (label == "rgb") {
            rgb_pixel_t* copy = new rgb_pixel_t[w*h];
            memcpy(copy, buff, w*h*sizeof(rgb_pixel_t));
            if (cleanup) delete[] (rgb_pixel_t*)buff;
            auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_RGB888, rgb_pixel_t_cleanupFunction, copy));
            ui->image2Label->setPixmap(pixmap);
        } else if (label == "rgb2") {
            rgb_pixel_t* copy = new rgb_pixel_t[w*h];
            memcpy(copy, buff, w*h*sizeof(rgb_pixel_t));
            if (cleanup) delete[] (rgb_pixel_t*)buff;
            auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_RGB888, rgb_pixel_t_cleanupFunction, copy));
            ui->image4Label->setPixmap(pixmap);
        } else if (label == "rgb3") {
            rgb_pixel_t* copy = new rgb_pixel_t[w*h];
            memcpy(copy, buff, w*h*sizeof(rgb_pixel_t));
            if (cleanup) delete[] (rgb_pixel_t*)buff;
            auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_RGB888, rgb_pixel_t_cleanupFunction, copy));
            ui->image5Label->setPixmap(pixmap);
        } else if (label == "rgb4") {
            rgb_pixel_t* copy = new rgb_pixel_t[w*h];
            memcpy(copy, buff, w*h*sizeof(rgb_pixel_t));
            if (cleanup) delete[] (rgb_pixel_t*)buff;
            auto pixmap = QPixmap::fromImage(QImage((uchar*)copy, w, h, QImage::Format_RGB888, rgb_pixel_t_cleanupFunction, copy));
            ui->image6Label->setPixmap(pixmap);
        }
    });
    connect(workerThread, &WorkerThread::statusBarResult, this, [this](QString text){
        ui->statusbar->showMessage(text);
    });
    connect(workerThread, &WorkerThread::debugText, this, [this](QString text){
        ui->debugText->clear();
        QTextDocument* doc = ui->debugText->document();
        QTextCursor cursor(doc);
        cursor.movePosition(QTextCursor::Start);
        cursor.beginEditBlock();
        cursor.insertText(text);
        cursor.endEditBlock();
    });
    connect(workerThread, &WorkerThread::finished, workerThread, &QObject::deleteLater);
    workerThread->start();

    connect(ui->pauseBtn, &QPushButton::clicked, this, [this](){
        workerThread->paused = !workerThread->paused;
    });

    connect(ui->sliderA, &QSlider::valueChanged, this, [this](int position){
        workerThread->rotateA = position * PI / 180;
    });

    connect(ui->sliderB, &QSlider::valueChanged, this, [this](int position){
        workerThread->rotateB = position * PI / 180;
    });

    connect(ui->sliderC, &QSlider::valueChanged, this, [this](int position){
        workerThread->rotateC = position * PI / 180;
    });

    connect(ui->sliderD, &QSlider::valueChanged, this, [this](int position){
        workerThread->zoomScale = position / 100.;
    });

    connect(ui->sliderE, &QSlider::valueChanged, this, [this](int position){
        workerThread->sliceLevel = ui->sliderE->value() / 1000.;
        ui->textLabel1->setText(QString::number(workerThread->sliceLevel));
    });

    connect(ui->buttonA, &QPushButton::clicked, this, [this](){
        workerThread->rotateA = 0;
        ui->sliderA->setValue(0);
    });

    connect(ui->buttonB, &QPushButton::clicked, this, [this](){
        workerThread->rotateB = 0;
        ui->sliderB->setValue(0);
    });

    connect(ui->buttonC, &QPushButton::clicked, this, [this](){
        workerThread->rotateC = 0;
        ui->sliderC->setValue(0);
    });


    auto updateBoxHeight = [this](const QString& text){
        bool ok = false;
        auto boxHeight = ui->boxHeightEdit->text().toFloat(&ok);
        if (ok) {
            auto palletHeight = ui->palletHeightEdit->text().toFloat(&ok);
            if (ok) {
                workerThread->boxHeight = boxHeight + palletHeight;
            }
        }
    };


    connect(ui->boxHeightEdit, &QLineEdit::textChanged, this, updateBoxHeight);
    connect(ui->palletHeightEdit, &QLineEdit::textChanged, this, updateBoxHeight);
    updateBoxHeight("");


}

MainWindow::~MainWindow()
{
    delete ui;
}


bool dragging = false;
int dragPrevX = 0;
int dragPrevY = 0;

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    dragging = false;
    event->accept();
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    QPoint pos = QCursor::pos();
    dragPrevX = pos.x();
    dragPrevY = pos.y();
    dragging = true;
    event->accept();
}

void MainWindow::mouseMoveEvent(QMouseEvent *event){
    if (dragging) {
        QPoint pos = QCursor::pos();
        int dx = dragPrevX - pos.x();
        int dy = dragPrevY - pos.y();
        dragPrevX = pos.x();
        dragPrevY = pos.y();

        ui->sliderA->setValue(ui->sliderA->value() + dy / 2);
        ui->sliderB->setValue(ui->sliderB->value() - dx / 2);
    }
    event->accept();
}

void MainWindow::wheelEvent(QWheelEvent *event)
{
    if (dragging) {
        int dz = event->pixelDelta().y() / 12;
        ui->sliderC->setValue(ui->sliderC->value() + dz);
    } else {
        int dz = event->pixelDelta().y() / 12;
        ui->sliderD->setValue(ui->sliderD->value() + dz);
    }
    event->accept();
}
