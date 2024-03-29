#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "utils.h"
#include "flowlayout.h"

template<typename T>
void cleanupFunction(void *info) {
    T* buff = (T*) info;
    delete[] buff;
}

QLabel* firstLabel;

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

    connect(workerThread, &WorkerThread::drawDepth, this, [this](uint16_t* buff, size_t w, size_t h, int rotate){
        auto pixmap = QPixmap::fromImage(QImage((uchar*)buff, w, h, QImage::Format_Grayscale16, cleanupFunction<uint16_t>, buff));
        QTransform tr;
        tr.rotate(rotate);
        float scale = w > h ? 320. / w : 320. / h;
        tr.scale(scale, scale);
        pixmap = pixmap.transformed(tr);
        ui->depthLabel->setPixmap(pixmap);
    });

    connect(workerThread, &WorkerThread::drawColor, this, [this](uint8_t* buff, size_t w, size_t h, int rotate){
        auto pixmap = QPixmap::fromImage(QImage((uchar*)buff, w, h, QImage::Format_RGB888, cleanupFunction<uint8_t>, buff));
        QTransform tr;
        tr.rotate(rotate);
        float scale = w > h ? 320. / w : 320. / h;
        tr.scale(scale, scale);
        pixmap = pixmap.transformed(tr);
        ui->colorLabel->setPixmap(pixmap);
    });

    connect(workerThread, &WorkerThread::drawRgb, this, [this](QString label, uint8_t* buff, size_t w, size_t h, int rotate){
        static FlowLayout *flowLayout = new FlowLayout;
        static QMap<QString, QLabel*> labelsMap;
        static bool first = true;
        if (!labelsMap.contains(label)) {
            labelsMap[label] = new QLabel();
            flowLayout->addWidget(labelsMap[label]);
        }
        if (first) {
            first = false;
            ui->flowWidget->setLayout(flowLayout);
            firstLabel = labelsMap[label];
        }
        auto pixmap = QPixmap::fromImage(QImage((uchar*)buff, w, h, QImage::Format_RGB888, cleanupFunction<uint8_t>, buff));
        QTransform tr;
        tr.rotate(rotate);
        float scale = w > h ? 320. / w : 320. / h;
        tr.scale(scale, scale);
        pixmap = pixmap.transformed(tr);
        labelsMap[label]->setPixmap(pixmap);
    });

    connect(workerThread, &WorkerThread::statusMessage, this, [this](QString text){
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


    auto lookatGeometry = firstLabel->geometry();
    auto lookatTopLeft = firstLabel->mapToGlobal(lookatGeometry.topLeft());

    if (pos.x() >= lookatTopLeft.x() && pos.x() < lookatTopLeft.x() + lookatGeometry.width() && pos.y() >= lookatTopLeft.y() && pos.y() < lookatTopLeft.y() + lookatGeometry.height()) {
        workerThread->lookat_x = (pos.x() - lookatTopLeft.x()) / (float)lookatGeometry.width();
        workerThread->lookat_y = (pos.y() - lookatTopLeft.y()) / (float)lookatGeometry.height();
    } else {
        dragPrevX = pos.x();
        dragPrevY = pos.y();
        dragging = true;
    }

    event->accept();
}

void MainWindow::mouseMoveEvent(QMouseEvent *event){
    QPoint pos = QCursor::pos();
    int dx = dragPrevX - pos.x();
    int dy = dragPrevY - pos.y();
    dragPrevX = pos.x();
    dragPrevY = pos.y();

    auto lookatGeometry = firstLabel->geometry();
    auto lookatTopLeft = firstLabel->mapToGlobal(lookatGeometry.topLeft());

    if (pos.x() >= lookatTopLeft.x() && pos.x() < lookatTopLeft.x() + lookatGeometry.width() && pos.y() >= lookatTopLeft.y() && pos.y() < lookatTopLeft.y() + lookatGeometry.height()) {
        workerThread->lookat_x = (pos.x() - lookatTopLeft.x()) / (float)lookatGeometry.width();
        workerThread->lookat_y = (pos.y() - lookatTopLeft.y()) / (float)lookatGeometry.height();
    } else if (dragging) {
        ui->sliderA->setValue(ui->sliderA->value() + dx / 2);
        ui->sliderB->setValue(ui->sliderB->value() + dy / 2);
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
