#include <QApplication>

#include "qt_example.h"

int
main (int argc, char** argv)
{
    QApplication a (argc, argv);
    QtExample w;
    w.show ();
    return a.exec ();
}

