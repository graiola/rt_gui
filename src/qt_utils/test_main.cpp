#include <QApplication>

#include "window.h"
#include "rt_gui/rt_gui.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Window window;
    window.show();
    return app.exec();
}
