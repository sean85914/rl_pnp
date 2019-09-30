#include <tf/transform_broadcaster.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"

bool check_valid_norm(double x, double y, double z){
    double norm = sqrt(x*x+y*y+z*z);
    double err = fabs(norm-1.0f);
    if(err>epsilon) return false; // Invalid
    else return true; // Valid
}

bool check_valid_norm(double x, double y, double z, double w){
    double norm = sqrt(x*x+y*y+z*z+w*w);
    double err = fabs(norm-1.0f);
    if(err>epsilon) return false; // Invalid
    else return true; // Valid
}

bool check_valid_rh(tf::Vector3 v_x, tf::Vector3 v_y, tf::Vector3 v_z){
  tf::Vector3 new_z = v_x.cross(v_y),
              diff = new_z;
  diff -= v_z;
  if(diff.length()>epsilon) return false; // Invalid
  else return true; // Valid
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    mode(0),
    xx(1.0), xy(0.0), xz(0.0),
    yx(0.0), yy(1.0), yz(0.0),
    zx(0.0), zy(0.0), zz(1.0),
    qx(0.0), qy(0.0), qz(0.0), qw(1.0)
{
    ui->setupUi(this);
    ui->status->setText("wait for input...");
    ui->qx_output->setValue(qx);
    ui->qy_output->setValue(qy);
    ui->qz_output->setValue(qz);
    ui->qw_output->setValue(qw);
    connect(ui->xx_input, SIGNAL(valueChanged(double)), SLOT(setXX(double)));
    connect(ui->xy_input, SIGNAL(valueChanged(double)), SLOT(setXY(double)));
    connect(ui->xz_input, SIGNAL(valueChanged(double)), SLOT(setXZ(double)));
    connect(ui->yx_input, SIGNAL(valueChanged(double)), SLOT(setYX(double)));
    connect(ui->yy_input, SIGNAL(valueChanged(double)), SLOT(setYY(double)));
    connect(ui->yz_input, SIGNAL(valueChanged(double)), SLOT(setYZ(double)));
    connect(ui->zx_input, SIGNAL(valueChanged(double)), SLOT(setZX(double)));
    connect(ui->zy_input, SIGNAL(valueChanged(double)), SLOT(setZY(double)));
    connect(ui->zz_input, SIGNAL(valueChanged(double)), SLOT(setZZ(double)));
    connect(ui->qx_output, SIGNAL(valueChanged(double)), SLOT(setQX(double)));
    connect(ui->qy_output, SIGNAL(valueChanged(double)), SLOT(setQY(double)));
    connect(ui->qz_output, SIGNAL(valueChanged(double)), SLOT(setQZ(double)));
    connect(ui->qw_output, SIGNAL(valueChanged(double)), SLOT(setQW(double)));
    connect(ui->clear_button, SIGNAL(clicked()), SLOT(clear()));
    connect(ui->compute_button, SIGNAL(clicked()), SLOT(compute()));
    connect(ui->mode_box, SIGNAL(clicked()), SLOT(changeMode()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setXX(double value) {xx = value;}
void MainWindow::setXY(double value) {xy = value;}
void MainWindow::setXZ(double value) {xz = value;}
void MainWindow::setYX(double value) {yx = value;}
void MainWindow::setYY(double value) {yy = value;}
void MainWindow::setYZ(double value) {yz = value;}
void MainWindow::setZX(double value) {zx = value;}
void MainWindow::setZY(double value) {zy = value;}
void MainWindow::setZZ(double value) {zz = value;}
void MainWindow::setQX(double value) {qx = value;}
void MainWindow::setQY(double value) {qy = value;}
void MainWindow::setQZ(double value) {qz = value;}
void MainWindow::setQW(double value) {qw = value;}
void MainWindow::clear(void) {
  // Reset input
    ui->xx_input->setValue(1.0);
    ui->xy_input->setValue(0.0);
    ui->xz_input->setValue(0.0);
    ui->yx_input->setValue(0.0);
    ui->yy_input->setValue(1.0);
    ui->yz_input->setValue(0.0);
    ui->zx_input->setValue(0.0);
    ui->zy_input->setValue(0.0);
    ui->zz_input->setValue(1.0);
    ui->qx_output->setValue(0.0);
    ui->qy_output->setValue(0.0);
    ui->qz_output->setValue(0.0);
    ui->qw_output->setValue(1.0);
    ui->status->setText("wait for input...");
}
void MainWindow::compute(void) {
    if(!mode){ // mat2quat
        if(!check_valid_norm(xx, xy, xz) or \
       	   !check_valid_norm(yx, yy, yz) or \
           !check_valid_norm(zx, zy, zz)){
        	ui->status->setText("Invalid input: \nnon-unit vector");
        }else if(!check_valid_rh(tf::Vector3(xx, xy, xz), 
                                 tf::Vector3(yx, yy, yz),
                                 tf::Vector3(zx, zy, zz))){
            ui->status->setText("Invalid input: \nnon right-hand coord.");
    	}else{
            tf::Matrix3x3 rot_mat(xx, yx, zx,
                                  xy, yy, zy,
                                  xz, yz, zz);
        	tf::Quaternion quat; rot_mat.getRotation(quat);
            ui->qx_output->setValue(quat.getX()); qx = quat.getX();
            ui->qy_output->setValue(quat.getY()); qy = quat.getY();
            ui->qz_output->setValue(quat.getZ()); qz = quat.getZ();
            ui->qw_output->setValue(quat.getW()); qw = quat.getW();
            ui->status->setText("Completed!");
        }
    }else{ // quat2mat
        if(!check_valid_norm(qx, qy, qz, qw))
            ui->status->setText("Invalid input: \nnon-unit quat");
        else{
            tf::Quaternion quat(qx, qy, qz, qw);
            tf::Matrix3x3 rot_mat(quat);
            ui->xx_input->setValue(rot_mat[0][0]); xx = rot_mat[0][0];
            ui->xy_input->setValue(rot_mat[0][1]); xy = rot_mat[0][1];
            ui->xz_input->setValue(rot_mat[0][2]); xz = rot_mat[0][2];
            ui->yx_input->setValue(rot_mat[1][0]); yx = rot_mat[1][0];
            ui->yy_input->setValue(rot_mat[1][1]); yy = rot_mat[1][1];
            ui->yz_input->setValue(rot_mat[1][2]); yz = rot_mat[1][2];
            ui->zx_input->setValue(rot_mat[2][0]); zx = rot_mat[2][0];
            ui->zy_input->setValue(rot_mat[2][1]); zy = rot_mat[2][1];
            ui->zz_input->setValue(rot_mat[2][2]); zz = rot_mat[2][2];
            ui->status->setText("Completed!");
        }
    }
}

void MainWindow::changeMode(void){
    mode = !mode;
    if(!mode){ // mat2quat
        ui->xx_input->setReadOnly(false);
        ui->xy_input->setReadOnly(false);
        ui->xz_input->setReadOnly(false);
        ui->yx_input->setReadOnly(false);
        ui->yy_input->setReadOnly(false);
        ui->yz_input->setReadOnly(false);
        ui->zx_input->setReadOnly(false);
        ui->zy_input->setReadOnly(false);
        ui->zz_input->setReadOnly(false);
        ui->qx_output->setReadOnly(true);
        ui->qy_output->setReadOnly(true);
        ui->qz_output->setReadOnly(true);
        ui->qw_output->setReadOnly(true);
        ui->status->setText("Mode changed: \nmat2quat");
    }else{ // quat2mat
        ui->xx_input->setReadOnly(true);
        ui->xy_input->setReadOnly(true);
        ui->xz_input->setReadOnly(true);
        ui->yx_input->setReadOnly(true);
        ui->yy_input->setReadOnly(true);
        ui->yz_input->setReadOnly(true);
        ui->zx_input->setReadOnly(true);
        ui->zy_input->setReadOnly(true);
        ui->zz_input->setReadOnly(true);
        ui->qx_output->setReadOnly(false);
        ui->qy_output->setReadOnly(false);
        ui->qz_output->setReadOnly(false);
        ui->qw_output->setReadOnly(false);
        ui->status->setText("Mode changed: \nquat2mat");
    }
}
