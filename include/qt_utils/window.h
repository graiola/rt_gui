#ifndef QT_UTILS_WINDOW_H
#define QT_UTILS_WINDOW_H

#include <QWidget>
#include <QtWidgets>
#include <QMap>

QT_BEGIN_NAMESPACE
class QGroupBox;
QT_END_NAMESPACE
class Slider;
class RadioButton;

class WidgetsGroup : public QGroupBox
{
  Q_OBJECT

public:
  WidgetsGroup(const QString &title,
               QWidget *parent = 0);

  void add(QWidget* widget);

private:
  QBoxLayout* layout_;
};

class Window : public QWidget
{
    Q_OBJECT

public:

    typedef QMap<QString,WidgetsGroup* > widgets_group_map_t;

    Window(const QString& title);

public slots:
    void addSlider(const QString& group_name, const QString& data_name, const double& min, const double& max, const double& init);
    void addRadioButton(const QString& group_name, const QString& data_name, const bool& init);
    void addComboBox(const QString& group_name, const QString& data_name, const QStringList& list, const QString& init);

    void sliderChanged(double value);
    void radioButtonChanged(bool value);
    void comboBoxChanged(QString value);

signals:
    void updateSlider(QString group_name, QString data_name, double value);
    void updateRadioButton(QString group_name, QString data_name, bool value);
    void updateComboBox(QString group_name, QString data_name, QString value);

private:
    void createTabs();

    bool checkIfDuplicated(const widgets_group_map_t& map, const QString& group_name, const QString& data_name);

    widgets_group_map_t widgets_group_;
    QTabWidget* tabs_;
    QVBoxLayout* main_layout_;
    QGroupBox* sliders_layout_;

};

#endif
