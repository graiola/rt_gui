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
    Window();

    double* addSlider(const QString& group_name, const QString& data_name, const double& min, const double& max);

    void createTabs();

private:
    QMap<QString,SlidersGroup* > sliders_;
    QTabWidget* tabs_;
    QVBoxLayout* main_layout_;
    QGroupBox* sliders_layout_;

};

#endif
