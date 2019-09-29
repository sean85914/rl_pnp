#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "src/ik_solver.hpp"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    x_(0.0f), y_(0.0f), z_(0.0f),
    qx_(0.0f), qy_(0.0f), qz_(0.0f), qw_(1.0f),
    tool_length_(0.0f)
{
    ui->setupUi(this);
    // Set value
    connect(ui->input_x, SIGNAL(valueChanged(double)), SLOT(setX(double)));
    connect(ui->input_y, SIGNAL(valueChanged(double)), SLOT(setY(double)));
    connect(ui->input_z, SIGNAL(valueChanged(double)), SLOT(setZ(double)));
    connect(ui->input_qx, SIGNAL(valueChanged(double)), SLOT(setQX(double)));
    connect(ui->input_qy, SIGNAL(valueChanged(double)), SLOT(setQY(double)));
    connect(ui->input_qz, SIGNAL(valueChanged(double)), SLOT(setQZ(double)));
    connect(ui->input_qw, SIGNAL(valueChanged(double)), SLOT(setQW(double)));
    connect(ui->input_tool_length, SIGNAL(valueChanged(double)), SLOT(setToolLength(double)));
    // Reset widgets
    connect(ui->button_clear, SIGNAL(clicked()), SLOT(clear_widgets(void)));
    connect(ui->button_compute, SIGNAL(clicked()), SLOT(compute(void)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::clear_widgets(void){
    ui->input_x->setValue(0.0f);
    ui->input_y->setValue(0.0f);
    ui->input_z->setValue(0.0f);
    ui->input_qx->setValue(0.0f);
    ui->input_qy->setValue(0.0f);
    ui->input_qz->setValue(0.0f);
    ui->input_qw->setValue(0.0f);
    ui->input_tool_length->setValue(0.0f);
}

void MainWindow::compute(void){
    result_string = UIBridge(x_, y_, z_, qx_, qy_, qz_, qw_, tool_length_);
    ui->text_placeholder->setText(QString::fromUtf8(result_string.c_str()));
}
