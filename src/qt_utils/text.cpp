#include <rt_gui/qt_utils/text.h>
#include <QStyle>

Text::Text(const QString &client_name, const QString &group_name, const QString &data_name, const QString &placeholder, QWidget *parent)
  : QGroupBox()
{
  group_name_ = group_name;
  data_name_  = data_name;
  client_name_= client_name;

  setObjectName(data_name);

  text_ = new QLineEdit();
  text_->setPlaceholderText(placeholder);
  text_->setFocus();

  title_ = new QLabel(data_name);

  value_ = placeholder;

  QIcon icon;
  QSize sz(16, 16);
  icon.addPixmap(style()->standardIcon(QStyle::SP_ArrowRight).pixmap(sz),
                  QIcon::Normal, QIcon::On);
  QAction *pushButton = text_->addAction(icon, QLineEdit::TrailingPosition);
  connect(pushButton, &QAction::triggered, this, &Text::setValue);

  QBoxLayout *widgets_layout = new QBoxLayout(QBoxLayout::TopToBottom);
  widgets_layout->addWidget(title_);
  widgets_layout->addWidget(text_);
  setLayout(widgets_layout);
}

void Text::setValue()
{
  if(text_->text().isEmpty())
    value_ = text_->placeholderText();
  else
    value_ = text_->text();
  emit valueChanged(value_);
}

QString Text::getValue()
{
  return value_;
}

const QString &Text::getDataName() const
{
  return data_name_;
}

const QString &Text::getGroupName() const
{
  return group_name_;
}

const QString &Text::getClientName() const
{
  return client_name_;
}
