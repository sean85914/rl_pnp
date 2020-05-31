/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *label_x;
    QLabel *label_z;
    QLabel *label_y;
    QLabel *label_qx;
    QLabel *label_qy;
    QLabel *label_qz;
    QLabel *label_qw;
    QLabel *label_tool_length;
    QDoubleSpinBox *input_x;
    QDoubleSpinBox *input_z;
    QDoubleSpinBox *input_qx;
    QDoubleSpinBox *input_qy;
    QDoubleSpinBox *input_qz;
    QDoubleSpinBox *input_tool_length;
    QDoubleSpinBox *input_qw;
    QDoubleSpinBox *input_y;
    QPushButton *button_compute;
    QPushButton *button_clear;
    QTextBrowser *text_placeholder;
    QLabel *label_result;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(498, 515);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label_x = new QLabel(centralWidget);
        label_x->setObjectName(QString::fromUtf8("label_x"));
        label_x->setGeometry(QRect(20, 20, 20, 20));
        label_z = new QLabel(centralWidget);
        label_z->setObjectName(QString::fromUtf8("label_z"));
        label_z->setGeometry(QRect(220, 20, 20, 20));
        label_y = new QLabel(centralWidget);
        label_y->setObjectName(QString::fromUtf8("label_y"));
        label_y->setGeometry(QRect(120, 20, 20, 20));
        label_qx = new QLabel(centralWidget);
        label_qx->setObjectName(QString::fromUtf8("label_qx"));
        label_qx->setGeometry(QRect(20, 50, 20, 20));
        label_qy = new QLabel(centralWidget);
        label_qy->setObjectName(QString::fromUtf8("label_qy"));
        label_qy->setGeometry(QRect(130, 50, 20, 20));
        label_qz = new QLabel(centralWidget);
        label_qz->setObjectName(QString::fromUtf8("label_qz"));
        label_qz->setGeometry(QRect(240, 50, 20, 20));
        label_qw = new QLabel(centralWidget);
        label_qw->setObjectName(QString::fromUtf8("label_qw"));
        label_qw->setGeometry(QRect(350, 50, 25, 20));
        label_tool_length = new QLabel(centralWidget);
        label_tool_length->setObjectName(QString::fromUtf8("label_tool_length"));
        label_tool_length->setGeometry(QRect(20, 90, 80, 20));
        input_x = new QDoubleSpinBox(centralWidget);
        input_x->setObjectName(QString::fromUtf8("input_x"));
        input_x->setGeometry(QRect(40, 20, 70, 20));
        input_x->setDecimals(3);
        input_x->setSingleStep(0.01);
        input_z = new QDoubleSpinBox(centralWidget);
        input_z->setObjectName(QString::fromUtf8("input_z"));
        input_z->setGeometry(QRect(240, 20, 70, 20));
        input_z->setDecimals(3);
        input_z->setSingleStep(0.01);
        input_qx = new QDoubleSpinBox(centralWidget);
        input_qx->setObjectName(QString::fromUtf8("input_qx"));
        input_qx->setGeometry(QRect(50, 50, 70, 20));
        input_qx->setDecimals(3);
        input_qx->setMaximum(1);
        input_qx->setSingleStep(0.01);
        input_qy = new QDoubleSpinBox(centralWidget);
        input_qy->setObjectName(QString::fromUtf8("input_qy"));
        input_qy->setGeometry(QRect(160, 50, 70, 20));
        input_qy->setDecimals(3);
        input_qy->setMaximum(1);
        input_qy->setSingleStep(0.01);
        input_qz = new QDoubleSpinBox(centralWidget);
        input_qz->setObjectName(QString::fromUtf8("input_qz"));
        input_qz->setGeometry(QRect(270, 50, 70, 20));
        input_qz->setDecimals(3);
        input_qz->setMaximum(1);
        input_qz->setSingleStep(0.01);
        input_tool_length = new QDoubleSpinBox(centralWidget);
        input_tool_length->setObjectName(QString::fromUtf8("input_tool_length"));
        input_tool_length->setGeometry(QRect(110, 90, 70, 20));
        input_tool_length->setDecimals(3);
        input_tool_length->setSingleStep(0.01);
        input_tool_length->setValue(0);
        input_qw = new QDoubleSpinBox(centralWidget);
        input_qw->setObjectName(QString::fromUtf8("input_qw"));
        input_qw->setGeometry(QRect(380, 50, 70, 20));
        input_qw->setDecimals(3);
        input_qw->setMaximum(1);
        input_qw->setSingleStep(0.01);
        input_y = new QDoubleSpinBox(centralWidget);
        input_y->setObjectName(QString::fromUtf8("input_y"));
        input_y->setGeometry(QRect(140, 20, 70, 20));
        input_y->setDecimals(3);
        input_y->setSingleStep(0.01);
        button_compute = new QPushButton(centralWidget);
        button_compute->setObjectName(QString::fromUtf8("button_compute"));
        button_compute->setGeometry(QRect(210, 80, 91, 41));
        button_clear = new QPushButton(centralWidget);
        button_clear->setObjectName(QString::fromUtf8("button_clear"));
        button_clear->setGeometry(QRect(320, 80, 91, 41));
        text_placeholder = new QTextBrowser(centralWidget);
        text_placeholder->setObjectName(QString::fromUtf8("text_placeholder"));
        text_placeholder->setGeometry(QRect(20, 200, 441, 241));
        text_placeholder->setReadOnly(true);
        label_result = new QLabel(centralWidget);
        label_result->setObjectName(QString::fromUtf8("label_result"));
        label_result->setGeometry(QRect(20, 150, 81, 31));
        QFont font;
        font.setPointSize(18);
        label_result->setFont(font);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 498, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        label_x->setText(QApplication::translate("MainWindow", "X", 0));
        label_z->setText(QApplication::translate("MainWindow", "Z", 0));
        label_y->setText(QApplication::translate("MainWindow", "Y", 0));
        label_qx->setText(QApplication::translate("MainWindow", "QX", 0));
        label_qy->setText(QApplication::translate("MainWindow", "QY", 0));
        label_qz->setText(QApplication::translate("MainWindow", "QZ", 0));
        label_qw->setText(QApplication::translate("MainWindow", "QW", 0));
        label_tool_length->setText(QApplication::translate("MainWindow", "Tool Length", 0));
        button_compute->setText(QApplication::translate("MainWindow", "Compute", 0));
        button_clear->setText(QApplication::translate("MainWindow", "Clear", 0));
        label_result->setText(QApplication::translate("MainWindow", "Result", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
