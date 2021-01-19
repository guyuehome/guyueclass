#ifndef FORM_H
#define FORM_H

#include <QWidget>

namespace Ui {
class Form;
}

class Form : public QWidget
{
    Q_OBJECT

public:
    explicit Form(QWidget *parent = nullptr);
    ~Form();
private slots:
    void on_pushButton_clicked();
signals:
    void close_and_open();

private:
    Ui::Form *ui;
};

#endif // FORM_H
