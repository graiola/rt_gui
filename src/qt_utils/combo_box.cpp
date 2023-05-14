#include <rt_gui/qt_utils/combo_box.h>

ComboBox::ComboBox(const QString &client_name, const QString &group_name, const QString &data_name, const QStringList& list, const QString& init, QWidget *parent)
  : QGroupBox(data_name, parent)
{
  group_name_ = group_name;
  data_name_  = data_name;
  client_name_= client_name;

  setObjectName(data_name);

  combo_ = new QComboBox();
  combo_->addItems(list);

  int index = combo_->findText(init);
  if (index != -1)// -1 for not found
     combo_->setCurrentIndex(index);
  else
    qDebug() << init <<  " is not in the list!";

  value_ = init;

  connect(combo_,  SIGNAL(currentTextChanged(const QString&)), this, SLOT(setValue(const QString&)));

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::LeftToRight);
  widgets_layout->addWidget(combo_);
  setLayout(widgets_layout);
}

QString ComboBox::getValue()
{
  return value_;
}

const QString &ComboBox::getDataName() const
{
  return data_name_;
}

const QString &ComboBox::getGroupName() const
{
  return group_name_;
}

const QString &ComboBox::getClientName() const
{
  return client_name_;
}

void ComboBox::setValue(const QString& value)
{
  value_ = value;
  emit valueChanged(value_);
}

QDataStream &operator<<(QDataStream &out, ComboBox* widget)
{
  //out << typeid(*widget).name() << " ";
  //out << widget->group_name_ << " ";
  //out << widget->data_name_ << " ";
  //out << widget->client_name_ << " ";
  //out << "/n";
  return out;
}
