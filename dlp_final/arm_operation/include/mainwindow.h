#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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
    double x_, y_, z_;
    double qx_, qy_, qz_, qw_;
    double tool_length_;
    std::string result_string;

public slots:
    void setX(double x) {x_ = x;}
    void setY(double y) {y_ = y;}
    void setZ(double z) {z_ = z;}
    void setQX(double qx) {qx_ = qx;}
    void setQY(double qy) {qy_ = qy;}
    void setQZ(double qz) {qz_ = qz;}
    void setQW(double qw) {qw_ = qw;}
    void setToolLength(double tool_length) {tool_length_ = tool_length;}
    void clear_widgets(void);
    void compute(void);
};

#endif // MAINWINDOW_H
