#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <cmath>
#include <tf/transform_broadcaster.h>
#include <QMainWindow>

const double epsilon = 1e-3;

bool check_valid_norm(double, double, double);
bool check_valid_norm(double, double, double, double);
bool check_valid_rh(tf::Vector3, tf::Vector3, tf::Vector3);

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    bool mode;
    double xx, xy, xz;
    double yx, yy, yz;
    double zx, zy, zz;
    double qx, qy, qz, qw;
public slots:
    void setXX(double);
    void setXY(double);
    void setXZ(double);
    void setYX(double);
    void setYY(double);
    void setYZ(double);
    void setZX(double);
    void setZY(double);
    void setZZ(double);
    void setQX(double);
    void setQY(double);
    void setQZ(double);
    void setQW(double);
    void clear(void);
    void changeMode(void);
    void compute(void);
};

#endif // MAINWINDOW_H
