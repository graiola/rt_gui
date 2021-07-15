#ifndef QT_UTILS_WINDOW_H
#define QT_UTILS_WINDOW_H

#include <QWidget>
#include <QtWidgets>
#include <QMap>

QT_BEGIN_NAMESPACE
class QGroupBox;
QT_END_NAMESPACE
class Slider;
class SlidersGroup;

class Window : public QWidget
{
    Q_OBJECT

public:
    Window(const QString& title);

public slots:
    void addSlider(const QString& group_name, const QString& data_name, const double& min, const double& max, const double& init);

    void valueChanged(double value);

signals:
    void updateServer(QString group_name, QString data_name, double value);

private:
    void createTabs();

    QMap<QString,SlidersGroup* > sliders_;
    QTabWidget* tabs_;
    QVBoxLayout* main_layout_;
    QGroupBox* sliders_layout_;

};

#endif
