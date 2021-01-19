#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QProgressBar>
#include "form.h"
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
public slots:
    void slot_btn_click(bool);
    void slot_btn2_click(bool);
    void slot_checkbox_state_changed(int);
    void slot_sliderbar_value_changed(int);
    void slot_close_and_open();
private slots:
    void on_open_new_btn_clicked();

private:
    Ui::MainWindow *ui;
    QPushButton* btn2;
};
#endif // MAINWINDOW_H
