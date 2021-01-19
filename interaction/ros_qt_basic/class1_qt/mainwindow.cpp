#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->pushButton->setText("hello word!");
    connect(ui->pushButton,SIGNAL(clicked(bool)),this,SLOT(slot_btn_click(bool)));
    btn2=new QPushButton;
    btn2->setText("btn2");
    ui->verticalLayout_2->addWidget(btn2);
    connect(btn2,SIGNAL(clicked(bool)),this,SLOT(slot_btn2_click(bool)));
    //checkbox signal
    connect(ui->checkBox,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_state_changed(int)));
    ui->horizontalSlider->setMaximum(20);
    ui->horizontalSlider->setMinimum(0);

    //progressbar
    connect(ui->horizontalSlider,SIGNAL(valueChanged(int)),this,SLOT(slot_sliderbar_value_changed(int)));
    ui->progressBar->setRange(0,100);
    ui->progressBar->setValue(0);

    ui->treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
    QTreeWidgetItem* item1=new QTreeWidgetItem(QStringList()<<"Global Options");
    ui->treeWidget->addTopLevelItem(item1);
    item1->setIcon(0,QIcon("://images/options.png"));

    QTreeWidgetItem* item1_child1=new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    item1->addChild(item1_child1);

    QComboBox* box1=new QComboBox;
    box1->setMaximumWidth(160);
    box1->addItem("map");
    box1->setEditable(true);
    ui->treeWidget->setItemWidget(item1_child1,1,box1);

}
void MainWindow::slot_sliderbar_value_changed(int value)
{
    qDebug()<<"sliderbar value:"<<value;
    ui->label_slider_value->setText(QString::number(value));
    ui->progressBar->setValue(value);

}
void MainWindow::slot_checkbox_state_changed(int state)
{
    qDebug()<<"checkbox state:"<<state;
}
void MainWindow::slot_btn_click(bool)
{
    ui->pushButton->setText("clicked");
     ui->checkBox->setChecked(true);
    bool is_check= ui->checkBox->isChecked();
    qDebug()<<is_check;
}
void MainWindow::slot_btn2_click(bool)
{
    btn2->setText("clicked");
}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::slot_close_and_open()
{
    this->show();
}

void MainWindow::on_open_new_btn_clicked()
{
    Form* f=new Form;
    f->show();
    connect(f,SIGNAL(close_and_open()),this,SLOT(slot_close_and_open()));
    this->hide();
}
