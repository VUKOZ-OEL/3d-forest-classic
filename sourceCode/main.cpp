//  3DFOREST - tool for processing lidar data from forest environment>
//    Copyright (C) <2015>
//
//    Jan Trochta
//    Martin Krucek
//    Kamil Kral
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <QtWidgets/QApplication>
//#include <omp.h>
 #include "mainwindow.h"

 int main(int argc, char *argv[])
{
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
  QApplication app(argc, argv);
  //Q_INIT_RESOURCE(3dforest);
  app.setOrganizationName("VUKOZ v.v.i.");
  app.setApplicationName("3D Forest - Forest lidar data processing tool");
  app.setWindowIcon(QIcon(":/images/icon.ico"));
 
  MainWindow mainWin;
  mainWin.show();
  return app.exec();
 }
