#ifndef SLIDERSGROUP_H
#define SLIDERSGROUP_H

#include <QGroupBox>
#include <QSlider>
#include <qwt/qwt_slider.h>

QT_BEGIN_NAMESPACE
class QLabel;
class QLineEdit;
QT_END_NAMESPACE

class SlidersGroup : public QGroupBox
{
    Q_OBJECT

public:
    SlidersGroup(const QString &title, const double& min, const double& max,
                 QWidget *parent = 0);

signals:
    void valueChanged(int value);

public slots:
    void setValue(QString value);
    void setValue(double value);

private:
    QwtSlider*      slider_;
    QLineEdit*      current_;
    double          value_;
};

#endif
