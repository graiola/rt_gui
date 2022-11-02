#ifndef RT_GUI_QT_UTILS_WINDOW_H
#define RT_GUI_QT_UTILS_WINDOW_H

#include <QWidget>
#include <QtWidgets>
#include <QMap>

QT_BEGIN_NAMESPACE
class QGroupBox;
QT_END_NAMESPACE
class DoubleSlider;
class RadioButton;

class WidgetsGroup : public QGroupBox
{
  Q_OBJECT

public:
  WidgetsGroup(const QString &title,
               QWidget *parent = nullptr);

  void add(QWidget* widget);

  void remove(QWidget* widget);

private:
  QBoxLayout* layout_;
};

class Window : public QWidget
{
    Q_OBJECT

public:

    typedef QMap<QString,WidgetsGroup* > widgets_group_map_t;

    Window(const QString& title, QWidget* parent = nullptr);

public slots:
    void addText(const QString& client_name, const QString& group_name, const QString& data_name, const QString& placeholder);
    void addLabel(const QString& client_name, const QString& group_name, const QString& data_name, const QString& placeholder);
    void addButton(const QString& client_name, const QString& group_name, const QString& data_name);
    void addDoubleSlider(const QString& client_name, const QString& group_name, const QString& data_name, const double& min, const double& max, const double& init);
    void addIntSlider(const QString& client_name, const QString& group_name, const QString& data_name, const int& min, const int& max, const int& init);
    void addRadioButton(const QString& client_name, const QString& group_name, const QString& data_name, const bool& init);
    void addComboBox(const QString& client_name, const QString& group_name, const QString& data_name, const QStringList& list, const QString& init);

    void textChanged(QString value);
    void labelChanged(QString value);
    void buttonChanged();
    void intSliderChanged(int value);
    void doubleSliderChanged(double value);
    void radioButtonChanged(bool value);
    void comboBoxChanged(QString value);

    void removeWidget(const QString& client_name, const QString& group_name, const QString& data_name);

signals:
    void updateText(QString client_name, QString group_name, QString data_name, QString value);
    void updateLabel(QString client_name, QString group_name, QString data_name, QString value);
    void updateButton(QString client_name, QString group_name, QString data_name);
    void updateDoubleSlider(QString client_name, QString group_name, QString data_name, double value);
    void updateIntSlider(QString client_name, QString group_name, QString data_name, int value);
    void updateRadioButton(QString client_name, QString group_name, QString data_name, bool value);
    void updateComboBox(QString client_name, QString group_name, QString data_name, QString value);

private:
    void createTabs();

    bool checkIfDuplicated(const widgets_group_map_t& map, const QString& group_name, const QString& data_name);

    widgets_group_map_t widgets_group_;
    QTabWidget* tabs_;
    QVBoxLayout* main_layout_;
    QGroupBox* sliders_layout_;


};

#endif
