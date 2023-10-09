#include <rt_gui/qt_utils/check_box.h>

CheckBox::CheckBox(const QString &client_name, const QString &group_name, const QString &data_name, const QStringList& list, const QVector<bool>& init, QWidget *parent)
  : QGroupBox(data_name, parent)
{
  group_name_ = group_name;
  data_name_  = data_name;
  client_name_= client_name;

  setObjectName(data_name);

  // Check
  if(list.count() != init.size())
  {
    qDebug() << "Wrong size in CheckBox!";
    return;
  }

  QStringListIterator it(list);

  widget_list_ = new QListWidget();

  int idx = 0;
  while (it.hasNext())
  {
      QListWidgetItem* list_item = new QListWidgetItem(it.next(),widget_list_);
      if(init[idx])
        list_item->setCheckState(Qt::Checked);
      else
        list_item->setCheckState(Qt::Unchecked);

      widget_list_->addItem(list_item);

      idx ++ ;
  }

  value_ = init;

  connect(widget_list_,  SIGNAL(itemChanged(QListWidgetItem*)), this, SLOT(setValue(QListWidgetItem*)));

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::LeftToRight);
  widgets_layout->addWidget(widget_list_);
  setLayout(widgets_layout);
}

QVector<bool> CheckBox::getValue()
{
  return value_;
}

const QString &CheckBox::getDataName() const
{
  return data_name_;
}

const QString &CheckBox::getGroupName() const
{
  return group_name_;
}

const QString &CheckBox::getClientName() const
{
  return client_name_;
}

void CheckBox::setValue(QListWidgetItem* /*item*/)
{
  for(int i = 0; i < widget_list_->count(); ++i)
  {
      QListWidgetItem* item = widget_list_->item(i);
      if(Qt::Checked == item->checkState())
        value_[i] = true;
      else
        value_[i] = false;
  }
  emit valueChanged(value_);
}
