#include <QtGui/QApplication>
#include "Quad.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    Quad foo;
    foo.show();
    return app.exec();
}
