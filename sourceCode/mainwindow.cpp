//    This file is part of 3DFOREST  www.3dforest.eu
//
//    3DFOREST is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    3DFOREST is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with 3DFOREST.  If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////
#include "mainwindow.h"
#include "geomcalculations.h"
#include <random>


#include <liblas/liblas.hpp>
#include <pcl/kdtree/kdtree_flann.h>
//include BASE
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/math/special_functions/round.hpp>
#include <iostream>
#include <string>
#include <random>
#include <pcl/common/common_headers.h>
//include INPUT OUTPUT
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//include VISUALIZATION
#include <pcl/visualization/area_picking_event.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>

//include FILTERS
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>


//#include <vtkWin32OpenGLRenderWindow.h>
#include <vtkTIFFWriter.h>
#include <vtkRenderWindow.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <vtkSmartPointer.h>
#include <vtkLODActor.h>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkWindowToImageFilter.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkRenderer.h>
#include <vtkPNGWriter.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkInteractorStyle.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>


#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QColorDialog>


struct double3in
{
  double x,y,z;
  float in;
};
BOOST_FUSION_ADAPT_STRUCT(double3in, (double, x)(double, y)(double, z)(float, in))
struct double3
{
  double x,y,z;
};
BOOST_FUSION_ADAPT_STRUCT(double3, (double, x)(double, y)(double, z))

////MainWindow
 MainWindow::MainWindow()
{
  //Q_INIT_RESOURCE(3dforest);
    setWindowTitle ( tr("3D Forest - Forest Lidar Data Processing Tool") );

//QVTKwidget - visualizer
    qvtkwidget = new QVTKOpenGLNativeWidget(this);

    auto renderer = vtkSmartPointer<vtkRenderer >::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow >::New();
    auto style = vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle>::New();
    
    renderWindow->AddRenderer(renderer);
    
    m_vis = new pcl::visualization::PCLVisualizer(renderer, renderWindow, "data viewer", false);
    m_vis->setShowFPS(false);
    qvtkwidget->setRenderWindow(m_vis->getRenderWindow());
    m_vis->setupInteractor(qvtkwidget->interactor()->GetInteractorStyle()->GetInteractor(), qvtkwidget->renderWindow());
    coordianteAxes();
    setCentralWidget(qvtkwidget);
    qvtkwidget->show();
    qvtkwidget->update();
    

// Tree widget
  treeWidget = new MyTree;
  QDockWidget *dockWidget = new QDockWidget(tr("Pointcloud Layers"), this);
  dockWidget->setAllowedAreas(Qt::LeftDockWidgetArea);
  dockWidget->setMaximumWidth(500);
  dockWidget->setWidget(treeWidget);
  addDockWidget(Qt::LeftDockWidgetArea,dockWidget);
//Attribute table dock
  m_attributeTable = new QTableView;
  DockTableWidget = new QDockWidget(tr("Attribute table"), this);
  DockTableWidget->setWidget(m_attributeTable);
  addDockWidget(Qt::BottomDockWidgetArea, DockTableWidget);
  DockTableWidget->hide();
  //interaction table
  m_intersectionTable = new QTableView;
  DockITableWidget = new QDockWidget(tr("Intersection table"), this);
  DockITableWidget->setWidget(m_intersectionTable);
  addDockWidget(Qt::BottomDockWidgetArea, DockITableWidget);
  DockITableWidget->hide();
    //Feature table
    m_featureTable = new QTableView;
    DockFTableWidget = new QDockWidget(tr("Feature table"), this);
    DockFTableWidget->setWidget(m_featureTable);
    addDockWidget(Qt::BottomDockWidgetArea, DockFTableWidget);
    DockFTableWidget->hide();


  resize(QSize(1024, 800));
  move(QPoint(50, 50));

  createActions();
  createMenus();
  createToolbars();

  m_cloud1 = new Cloud();
  m_cloud = new Cloud();
  Proj = new Project();

  statusBar()->showMessage(tr("3D Forest Ready"));
  point_ev = m_vis->registerPointPickingCallback (&MainWindow::pointEvent, *this );

  m_editCloud = false;
}

MainWindow::~MainWindow()
{
  delete m_cloud1;
  delete m_cloud;
  delete Proj;
  delete treeWidget;
  delete qvtkwidget;
}
//PROJECT methods
void MainWindow::newProject()
{
  ProjImport * wiz = new ProjImport(this);
  wiz->setStartId(1);
  if(wiz->exec() == 1)
  {
    QString filename = QString("%1%2%3%2%4.3df").arg(wiz->field("projectPath").toString()).arg(QDir::separator ()).arg(wiz->field("projectName").toString()).arg(wiz->field("projectName").toString());
    closeProject();
    openProject(filename);
  }
}
void MainWindow::openProject()
{
//SET 3DF FILE
  QString fileName = QFileDialog::getOpenFileName(this,tr("Open Project File"),"",tr("files (*.3df)"));
  if (fileName.isEmpty())
    return;
  closeProject();
  openProject(fileName);



}
void MainWindow::openProject(QString path)
{
  QFile file (path);
  file.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&file);

  QFileInfo info (file);

  bool first_line = true;
  while(!in.atEnd())
  {
    QString lines = in.readLine();

    QStringList coords = lines.split(" ");
//READ FIRST LINE

    if(first_line == true) //pridat porovneni ulozene a nove cesty!
    {
      if(coords.size() == 5)
      {
        Proj = new Project(coords.at(1).toDouble(), coords.at(2).toDouble(), coords.at(3).toDouble(), coords.at(0).toUtf8().constData());
        Proj->set_path(coords.at(4).toUtf8().constData());
      }
      else
      {
        Proj = new Project(coords.at(1).toDouble(), coords.at(2).toDouble(), coords.at(0).toUtf8().constData());
        Proj->set_path(coords.at(3).toUtf8().constData());
        Proj->set_zTransform(0);
      }
      if(info.absoluteDir() != Proj->get_Path())
      {
        QMessageBox::StandardButton reply;
         reply = QMessageBox::question(0,tr("ddd"),("The actual path differs from the path saved in the project."
                                                        "Do you want to use the actual path and look up the project files in the directory?"),QMessageBox::Yes|QMessageBox::No);
        if(reply == QMessageBox::Yes)
        {
          Proj->set_path(info.absoluteDir().absolutePath());
        }
      }
      first_line = false;
      QString a = QString("3DForest - %1 ").arg(coords.at(0).toUtf8().constData());
      setWindowTitle(a);
    }
//READ REST OF FILE
    else
    {
      if(coords.size() >=2)
      {
        QFileInfo fileInfo (coords.at(1));
        QString name= Proj->get_Path();
        name.append(QDir::separator ());
        name.append( fileInfo.fileName());
        QFile f (name);

        if(!f.exists())
        {
            QString a =QString("The selected file \n -- %1 --  does not exist.\n Please check the path to the file in project file or try to import whole project.").arg(name);
            QMessageBox::StandardButton reply;
            reply = QMessageBox::warning(this,("Warning"),a,QMessageBox::Ok|QMessageBox::Abort);
            if(reply == QMessageBox::Abort)
              continue;
        }
        if(coords.size() >2)
        {
            QColor col = QColor(coords.at(2).toInt(),coords.at(3).toInt(),coords.at(4).toInt());
            openCloudFile(name,coords.at(0),col,false);
        }
        else if(coords.size() ==2)
        {
            openCloudFile(name,coords.at(0),false);
        }
        else
          continue;
      }
    }
  }
  file.close();
  m_vis->resetCamera();
  showAttributTableT->setEnabled(true);
}
void MainWindow::closeProject()
{
  //clean project
  Proj->cleanAll();
  //clean treewidget
  treeWidget->cleanAll();
  //clean qvtwidget
  m_vis->removeAllPointClouds();
  m_vis->removeAllShapes();
// set connection of treebar to default.
  positionT->setIcon(QPixmap(":/images/treeBar/position.png"));
  positionT->setEnabled(false);
  dbhthT->setIcon(QPixmap(":/images/treeBar/dbhHT.png"));
  dbhthT->setEnabled(false);
  dbhlsrT->setIcon(QPixmap(":/images/treeBar/dbhLSR.png"));
  dbhlsrT->setEnabled(false);
  heightT->setIcon(QPixmap(":/images/treeBar/height.png"));
  heightT->setEnabled(false);
  lengthT->setIcon(QPixmap(":/images/treeBar/length.png"));
  lengthT->setEnabled(false);
  stemCurveT->setIcon(QPixmap(":/images/treeBar/stemCurve.png"));
  stemCurveT->setEnabled(false);
  convexT->setIcon(QPixmap(":/images/treeBar/convex.png"));
  convexT->setEnabled(false);
  concaveT->setIcon(QPixmap(":/images/treeBar/concave.png"));
  concaveT->setEnabled(false);
  //set tree methods false
  dbhEditAct->setEnabled(false);
  dbhHTAct->setEnabled(false);
  dbhLSRAct->setEnabled(false);
  heightAct->setEnabled(false);
  lengAct->setEnabled(false);
  treeReconstructionAct->setEnabled(false);
  convexAct->setEnabled(false);
  concaveAct->setEnabled(false);
  stemCurvatureAct->setEnabled(false);
  tAAct->setEnabled(false);
  exportCONVEXAct->setEnabled(false);
  exportCONCAVEAct->setEnabled(false);
  exportStemCurvAct->setEnabled(false);
  showAttributTableT->setEnabled(false);

  crownDisplayHideT->setEnabled(false);
  crownHeightsDisplyHideT->setEnabled(false);
  crownPositionDisplayHideT->setEnabled(false);
  setVolumeByVoxAct->setEnabled(false);
  exportAttributesAct->setEnabled(false);
  convexHull3DAct->setEnabled(false);
  setCrownSectionsAct->setEnabled(false);
  crownExternalPtsT->setEnabled(false);
  setCrownAutomaticAct->setEnabled(false);
  setCrownManualAct->setEnabled(false);
  intersectionAct->setEnabled(false);
  crownIntersectionsT->setEnabled(false);
  crownIntersectionTableT->setEnabled(false);
  crownSurfaceBy3DHullT->setEnabled(false);
  crownSurfaceBySectionsT->setEnabled(false);

  setWindowTitle ( tr("3D Forest - Forest Lidar Data Processing Tool") );
  qvtkwidget->update();
}
void MainWindow::importProject()
{
  ProjImport * wiz = new ProjImport(this);
  wiz->setStartId(3);
  if(wiz->exec() == 1)
  {
    QString filename = QString("%1%2%3%2%3.3df").arg(wiz->field("newprojectPath").toString()).arg(QDir::separator ()).arg(wiz->field("newprojectname").toString());
    closeProject();
    openProject(filename);
  }
}
void MainWindow::createAttTable()
{
  int rows = Proj->get_sizeTreeCV();
  m_attributeTable->setModel(getModel());
  for(int i = 0; i < rows; i++)
    m_attributeTable->setRowHeight(i,20);

  for(int i=1;i<6;i++)
    m_attributeTable->setColumnWidth(i,70);

  for(int i=6;i<21;i++)
    m_attributeTable->setColumnWidth(i,100);

  for(int i=21;i<36;i++)
    m_attributeTable->setColumnWidth(i,70);

  m_attributeTable->setColumnWidth(9,100);

  QString label = QString("%1 - Attribute table").arg(Proj->get_ProjName());
  m_attributeTable->setWindowTitle(label);
}

QStandardItemModel* MainWindow::getModel()
{
  QStringList headers;
  headers <<"Name"<<"Points"<<"DBH HT\n[cm]"<<"DBH LSR\n[cm]"<<"DBH QSM\n[cm]"<<"Height\n[m]"<<"Lenght\n[m]"<<"Convex planar\n projection area\n[m^2]"<<"Concave planar\n projection area\n[m^2]"<<"volume QSM\n[cm]" <<" Volume \nsortiment 1\n (m^3)" <<"Lenght\n sortiment 1\n (m)" <<  "Volume\n sortiment 2\n (m^3)" <<"Lenght\n sortiment 2\n (m)" << "Volume\n sortiment 3\n (m^3)"<<"Lenght\n sortiment 3\n (m)" <<"Volume\n sortiment 4\n (m^3)" <<"Lenght\n sortiment 4\n (m)"<<"Volume\n sortiment 5\n (m^3)"<<"Lenght\n sortiment 5\n (m)" <<"Volume\n sortiment 6\n (m^3)"<<"Lenght\n sortiment 6\n (m)" ;
  headers <<"Crown\npoints"<<"Crown bottom\n height\n[m]"<<"Crown\nheight\n[m]"<<"Crown total\n height\n[m]"<<"Crown\n lenght\n[m]"<<"Crown\n width\n[m]"<<"Position\n distance\n[m]"<<"Position\n azimuth\n[deg]";
  headers <<"Volume by\n voxels\n[m^3]"<<"Volume by\n sections\n[m^3]"<<"Surface\n[m^2]"<<"Section\nheight\n[cm]"<<"Threshold\ndistance\n[cm]"<<"3DConvex\nsurface\n[m^2]"<<"3DConvex\nvolume\n[m^3]";

    int rows = Proj->get_sizeTreeCV();
    QStandardItemModel *model = new QStandardItemModel(rows,headers.size(),this);
  model->setHorizontalHeaderLabels(headers);

  for(int row = 0; row < rows; row++)
  {
            //TREE ATTRIBUTES
      //names
    QModelIndex index= model->index(row,0,QModelIndex());
    model->setData(index,Proj->get_TreeCloud(row).get_name());

      //points
    index= model->index(row,1,QModelIndex());
    model->setData(index,Proj->get_TreeCloud(row).get_Cloud()->width);

      // DBH HT
    index= model->index(row,2,QModelIndex());
    if(Proj->get_TreeCloud(row).get_dbhHT().r*2>0){
      model->setData(index,Proj->get_TreeCloud(row).get_dbhHT().r*2);}

      //DBH LSR
    index= model->index(row,3,QModelIndex());
    if(Proj->get_TreeCloud(row).get_dbhLSR().r>0){
      model->setData(index,Proj->get_TreeCloud(row).get_dbhLSR().r*2);}
      //DBH QSM
      index= model->index(row,4,QModelIndex());
      if(Proj->get_TreeCloud(row).getDBH_QSM().r>0){
          model->setData(index,Proj->get_TreeCloud(row).getDBH_QSM().r*2);}

      // height
    index= model->index(row,5,QModelIndex());
    if(Proj->get_TreeCloud(row).get_height()>0){
      model->setData(index,Proj->get_TreeCloud(row).get_height());}
//lenght
    index= model->index(row,6,QModelIndex());
    model->setData(index,Proj->get_TreeCloud(row).get_length());
// convex area
    index= model->index(row,7,QModelIndex());
    if(Proj->get_TreeCloud(row).getConvexAreaToInfoLine()>0){
    model->setData(index,Proj->get_TreeCloud(row).getConvexAreaToInfoLine());}
//concave area
    index= model->index(row,8,QModelIndex());
    if(Proj->get_TreeCloud(row).getConcaveAreaToInfoLine()>0){
    model->setData(index,Proj->get_TreeCloud(row).getConcaveAreaToInfoLine());}

//volume qsm
      index= model->index(row,9,QModelIndex());
      if(Proj->get_TreeCloud(row).getVolume() > -1)
          model->setData(index,Proj->get_TreeCloud(row).getVolume());

      //volume sortiment 1
      index= model->index(row,10,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentVolume(1) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentVolume(1));
      //lenght sortiment 1
      index= model->index(row,11,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentLenght(1) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentLenght(1));
      //volume sortiment 2
      index= model->index(row,12,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentVolume(2) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentVolume(2));
      //lenght sortiment 2
      index= model->index(row,13,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentLenght(2) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentLenght(2));
      //volume sortiment 3
      index= model->index(row,14,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentVolume(3) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentVolume(3));
      //lenght sortiment 3
      index= model->index(row,15,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentLenght(3) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentLenght(3));
      //volume sortiment 4
      index= model->index(row,16,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentVolume(4) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentVolume(4));
      //lenght sortiment 4
      index= model->index(row,17,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentLenght(4) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentLenght(4));
      //volume sortiment 5
      index= model->index(row,18,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentVolume(5) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentVolume(5));
      //lenght sortiment 5
      index= model->index(row,19,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentLenght(5) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentLenght(5));
      //volume sortiment 6
      index= model->index(row,20,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentVolume(6) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentVolume(6));
      //lenght sortiment 6
      index= model->index(row,21,QModelIndex());
      if(Proj->get_TreeCloud(row).getSortimentLenght(6) > -1)
          model->setData(index,Proj->get_TreeCloud(row).getSortimentLenght(6));

            //CROWN ATTRIBUTES
    if(Proj->get_TreeCloud(row).isCrownExist() == true)
    {
        //crown poitns size
      index= model->index(row,22,QModelIndex());
        int pointsize =Proj->get_TreeCloud(row).get_TreeCrown().get_Cloud()->points.size();
      model->setData(index, pointsize);
//crown bottom heigt
                index= model->index(row,23,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getCrownBottomHeight());
//crown height
                index= model->index(row,24,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getCrownHeight());
//crown total height
                index= model->index(row,25,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getCrownTotalHeight());

                index= model->index(row,26,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getCrownLenghtXY());

                index= model->index(row,27,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getCrownWidthXY());

                index= model->index(row,28,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getPosDist());

                index= model->index(row,29,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getAzimuth());

                index= model->index(row,30,QModelIndex());
                if(Proj->get_TreeCloud(row).get_TreeCrown().getVolumeVoxels()>0){
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getVolumeVoxels());}

                index= model->index(row,31,QModelIndex());
                if(Proj->get_TreeCloud(row).get_TreeCrown().isSectionsPolyhedronExist()==true){
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getVolumeSections());
                 }

                index= model->index(row,32,QModelIndex());
                if(Proj->get_TreeCloud(row).get_TreeCrown().isSectionsPolyhedronExist()==true){
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getPolyhedronFromSections().getSurfaceArea());
                }

                index= model->index(row,33,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getSectionHeight());

                index= model->index(row,34,QModelIndex());
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().getThresholdDistanceForsectionsHull());

                index= model->index(row,35,QModelIndex());
                if(Proj->get_TreeCloud(row).get_TreeCrown().isConvexhull3DExist()==true){
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().get3DConvexhull().getSurfaceArea());
                }

                index= model->index(row,36,QModelIndex());
                if(Proj->get_TreeCloud(row).get_TreeCrown().isConvexhull3DExist()==true){
                model->setData(index,Proj->get_TreeCloud(row).get_TreeCrown().get3DConvexhull().getVolume());
      }
    }
  }
  return model;
}
void MainWindow::showAttributeTable()
{
  createAttTable();
  if(DockTableWidget->isHidden())
    DockTableWidget->show();

  if(visAtt == false)
  {
    visAtt=true;
    m_attributeTable->show();
  }
  else
  {
    DockTableWidget->hide();
    m_attributeTable->hide();
    visAtt=false;
  }
}
void MainWindow::refreshAttTable()
{
  m_attributeTable->setModel(getModel());
}
void MainWindow::createFeatureTable()
{
    std::cout<<"createFeatureTable()\n";
    int rows = Proj->getFeatureSize();
    m_featureTable->setModel(getFeatureModel());
    for(int i = 0; i < rows; i++)
      m_featureTable->setRowHeight(i,20);

    for(int i=1;i<6;i++)
     m_featureTable->setColumnWidth(i,70);


    QString label = QString("%1 - Feature table").arg(Proj->get_ProjName());
    m_featureTable->setWindowTitle(label);
    
}
void MainWindow::showFeatureTable()
{
  createFeatureTable();
  if(DockFTableWidget->isHidden())
    DockFTableWidget->show();

  if(visFT == false)
  {
    visFT=true;
    m_featureTable->show();
  }
  else
  {
    DockFTableWidget->hide();
    m_featureTable->hide();
    visFT=false;
  }
}
QStandardItemModel*  MainWindow::getFeatureModel(){
    //std::cout<<"getFeatureModel()\n";
    QStringList headers;
    headers <<"Name"<<"Points"<<"X PCA axis lenght"<<"Y PCA axis lenght"<<"Convex Area"<<"Concave Area"<<"dalsi" ;
      //std::cout<<"header\n";
    int rows = Proj->getFeatureSize();
    //std::cout<<"pocet zaznamu: " << rows << "\n";
    QStandardItemModel *model = new QStandardItemModel(rows,headers.size(),this);
    model->setHorizontalHeaderLabels(headers);

    for(int row = 0; row < rows; row++)
    {
        //Feature ATTRIBUTES
        //names
        QModelIndex index= model->index(row,0,QModelIndex());
        model->setData(index,Proj->getFeature(row).getPointCloud()->get_name());

          //points
        index= model->index(row,1,QModelIndex());
        model->setData(index,Proj->getFeature(row).getPointNumber());

          // X pca
        index= model->index(row,2,QModelIndex());
        if(Proj->getFeature(row).getXlenght()>0){
            model->setData(index,Proj->getFeature(row).getXlenght());}

          //Y pca
        index= model->index(row,3,QModelIndex());
        if(Proj->getFeature(row).getYlenght()>0){
            model->setData(index,Proj->getFeature(row).getYlenght());}
          //Convex Area
        index= model->index(row,4,QModelIndex());
        if(Proj->getFeature(row).getConvexArea() >0){
            model->setData(index,Proj->getFeature(row).getConvexArea());}
          // Concave Area
        index= model->index(row,5,QModelIndex());
        if(Proj->getFeature(row).getConcaveArea()>0){
          model->setData(index,Proj->getFeature(row).getConcaveArea());}
        // centroid
      }
    return model;
}
void MainWindow::refreshFeatureTable(){
    m_featureTable->setModel(getFeatureModel());
}
//IMPORT methods
void MainWindow::importTXT(QString file, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
    boost::iostreams::mapped_file mmap(
        file.toLatin1().constData(),
        boost::iostreams::mapped_file::readonly);

    BOOST_AUTO (f, mmap.const_data());
    BOOST_AUTO (l, f + mmap.size());

    float pointsNum = 0;
    while (f && f!=l)
        if ((f = static_cast<const char*>(memchr(f, '\n', l-f))))
            pointsNum++, f++;

    BOOST_AUTO (ff, mmap.const_data());
    BOOST_AUTO (ll, ff + mmap.size());

    std::vector<double> data;
    bool ret = boost::spirit::qi::phrase_parse(ff, ll, (*boost::spirit::qi::double_   % boost::spirit::qi::space >>  *boost::spirit::qi::double_   % boost::spirit::qi::eol)  , boost::spirit::qi::blank, data);
    std::cout<<"veliksot data: " << data.size()<< "\n";


    if (data.size() == 0)
    {
        importTXTnonStandard(file,output);
    }
    else if(ret == true)
    {
        double r= data.size()/pointsNum;
        int fields=6;
//        if(r > 2.5 && r < 3.5)
//            fields =3;
//        else if(r > 3.5 && r < 5)
//            fields =4;
//        else
//            fields =-1;

        if( fields < 3)
        {
            QMessageBox::warning(this, tr("Error"),tr("Cannot parse the imported file. Import failed."));
            return;
        }
        for(int q=0; q < (data.size()-fields); q+=fields)
        {
        //std::cout<<"data: x " << data.at(q) << " y " << data.at(q+1) <<" z " << data.at(q+2) <<"\n";
            pcl::PointXYZI data_t;
            data_t.x = data.at(q) + Proj->get_Xtransform();
            data_t.y = data.at(q+1) + Proj->get_Ytransform();
            data_t.z = data.at(q+2) + Proj->get_Ztransform();
            data_t.intensity=data.at(q+3);
//            if(fields == 4)
//                data_t.intensity=data.at(q+3);
//            else
//                data_t.intensity=0;
            output->points.push_back(data_t);
        }
        output->width = output->points.size();
        output->is_dense=true;
        output->height=1;
        return;
    }
    else {
        QMessageBox::warning(this, tr("Error"),tr("Cannot parse the imported file. Import failed."));
        return;
    }
}
void MainWindow::importTXTnonStandard(QString file, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
    QFile fileName (file);
    fileName.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&fileName);
    QString line = in.readLine();

    QStringList coordsSpace = line.split(" ");
    QStringList coordsTab = line.split("\t");
    QStringList coordsSemicolon = line.split(";");

    QStringList firstLine;
    QString split;

    if(coordsSpace.size() > coordsTab.size() && coordsSpace.size() > coordsSemicolon.size())
    {
        split = " ";
        firstLine = coordsSpace;

    }
    else if(coordsTab.size() > coordsSpace.size() && coordsTab.size() > coordsSemicolon.size())
    {
        split = "\t";
        firstLine = coordsTab;
    }
    else if(coordsSemicolon.size() > coordsSpace.size() && coordsSemicolon.size() > coordsTab.size())
    {
        split = ";";
        firstLine = coordsSemicolon;
    }
    else{
        QMessageBox::warning(this, tr("Error"),tr("Cannot parse the imported file. Import failed."));
        return;
    }

    bool isNum = false;
    bool isIntensity = false;

    firstLine.at(0).toDouble(&isNum);

    if(isNum == true && firstLine.size()>4)
    {
        pcl::PointXYZI data;
        data.x = firstLine.at(0).toFloat() + Proj->get_Xtransform();
        data.y = firstLine.at(1).toFloat() + Proj->get_Ytransform();
        data.z = firstLine.at(2).toFloat() + Proj->get_Ztransform();
        data.intensity=firstLine.at(3).toFloat();
        output->points.push_back(data);
        isIntensity = true;
    }
    else if(isNum == true && firstLine.size()==4)
    {
        pcl::PointXYZI data;
        data.x = firstLine.at(0).toFloat() + Proj->get_Xtransform();
        data.y = firstLine.at(1).toFloat() + Proj->get_Ytransform();
        data.z = firstLine.at(2).toFloat() + Proj->get_Ztransform();
        data.intensity=0.0f;
        output->points.push_back(data);
    }
    if(isIntensity == true)
    {
        while(!in.atEnd())
        {
            QString line = in.readLine();
            QStringList coords = line.split(split);
            pcl::PointXYZI data;
            data.x = coords.at(0).toFloat() + Proj->get_Xtransform();
            data.y = coords.at(1).toFloat() + Proj->get_Ytransform();
            data.z = coords.at(2).toFloat() + Proj->get_Ztransform();
            data.intensity=coords.at(3).toFloat();
            output->points.push_back(data);
        }
    }
    else{
        while(!in.atEnd())
        {
            QString line = in.readLine();
            QStringList coords = line.split(split);
            pcl::PointXYZI data;
            data.x = coords.at(0).toFloat() + Proj->get_Xtransform();
            data.y = coords.at(1).toFloat() + Proj->get_Ytransform();
            data.z = coords.at(2).toFloat() + Proj->get_Ztransform();
            data.intensity=0.0f;
            output->points.push_back(data);
        }
    }
    output->width = output->points.size();
    output->is_dense=true;
    output->height=1;
    fileName.close();
    return;
}
void MainWindow::importPTS(QString file, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
  QFile fileName (file);
  fileName.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileName);
  bool first_line = true;
  int cloud_number = 1;
  while(!in.atEnd())
  {
    QString line = in.readLine();
    QStringList coords = line.split(" ");

    if(coords.size() == 1) // header
    {
      if (first_line != true )
      {
        output->width = output->points.size();
        output->is_dense=true;
        output->height=1;
          //cloud name
        QString a = QString ("xxx_%1").arg(cloud_number);
        // save cloud
        Proj->save_newCloud( "cloud", a, output);
      //empty cloud
        output->points.clear();
        cloud_number++;
      }
      first_line = false;

    }
    else //points
    {
      pcl::PointXYZI data;
      data.x = coords.at(0).toDouble() + Proj->get_Xtransform();
      data.y = coords.at(1).toDouble() + Proj->get_Ytransform();
      data.z = coords.at(2).toDouble() + Proj->get_Ztransform();
      data.intensity=coords.at(3).toFloat();
      output->points.push_back(data);
    }
  }
  fileName.close();
  output->width = output->points.size();
  output->is_dense=true;
  output->height=1;
}
void MainWindow::importPTX(QString file, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
  QFile fileName (file);
  fileName.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileName);
  int i = 0;
  while(!in.atEnd())
  {
    if (i < 11)
    {
      QString hlavicka = in.readLine();
    }
    else
    {
      QString line = in.readLine();
      QStringList coords = line.split(" ");
      pcl::PointXYZI data;
      double x,y;
      float z,i;
      x = coords.at(0).toDouble();
      y = coords.at(1).toDouble();
      z = coords.at(2).toFloat();
      i = coords.at(3).toFloat();
      if (x != 0 && y != 0 && z != 0 && i != 0.5)
      {
        data.x = x+Proj->get_Xtransform();
        data.y = y+Proj->get_Ytransform();
        data.z = z+ Proj->get_Ztransform();
        data.intensity=i;
        output->points.push_back(data);
      }
    }
    i++;
  }
  fileName.close();
  in.~QTextStream();
  output->width = output->points.size ();
  output->is_dense=true;
  output->height=1;
}
void MainWindow::importLAS(QString file, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
  qWarning()<<"importing las file";
  std::ifstream ifs;
  ifs.open(file.toUtf8().constData(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);
  //qWarning()<<"reader";
//read data into cloud
  while (reader.ReadNextPoint())
  {
    //qWarning()<<"reading points";
    liblas::Point const& p = reader.GetPoint();
    pcl::PointXYZI data;
    data.x = p.GetX()+Proj->get_Xtransform();
    data.y = p.GetY()+Proj->get_Ytransform();
    data.z = p.GetZ()+Proj->get_Ztransform();
    data.intensity = p.GetIntensity();
    output->points.push_back(data);
  }
  //qWarning()<<"saving";
  output->width = output->points.size ();
  output->is_dense=true;
  output->height=1;
  qWarning()<<"importing las file finished";
}
void MainWindow::importPCD(QString file, pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(file.toUtf8().constData(),*cloud_tmp);

//  QMessageBox::StandardButton reply;
//  reply =QMessageBox::question(0,("Transform cloud?"), "Do you want to apply transform matrix on selected file?",QMessageBox::Yes|QMessageBox::No );
//  if(reply ==QMessageBox::Yes )
//  {
//    for(int i = 0; i <cloud_tmp->points.size();i++)
//    {
//      pcl::PointXYZI oldp, newp;
//      oldp = cloud_tmp->points.at(i);
//      newp.x = oldp.x + Proj->get_Xtransform();
//      newp.y = oldp.y + Proj->get_Ytransform();
//      newp.z = oldp.z + Proj->get_Ztransform();
//      newp.intensity = oldp.intensity;
//      output->points.push_back(newp);
//    }
//  }
//  else{*output = *cloud_tmp;}
  *output = *cloud_tmp;
  output->width = output->points.size();
  output->is_dense=true;
  output->height=1;
  cloud_tmp.reset();
}

void MainWindow::importBaseCloud()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select File"),Proj->get_Path(),
                                                 tr("All Supported Files (*.pcd *.txt *.xyz *.las  *.pts *.ptx);;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No File Selected");
    return;
  }
//qWarning()<<"vybrano";
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);
     // qWarning()<<fileName;
    QStringList typ = fileName.split(".");

    if(typ.at(typ.size()-1) == "pcd")
       importPCD(fileName, cloud);
    if(typ.at(typ.size()-1) == "txt")
      importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "xyz")
      importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "las")
      importLAS(fileName, cloud);
    if(typ.at(typ.size()-1) == "pts")
       importPTS(fileName, cloud);
    if(typ.at(typ.size()-1) == "ptx")
      importPTX(fileName, cloud);


    boost::filesystem::path p (fileName.toLatin1().constData());
    std::string name = p.filename().stem().string();

   //if file exist in project folder, ask for another name
    QString name_t = QString::fromStdString(name);
    QString newFile = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name_t);
     // qWarning()<<newFile;
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name.at(0));
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = QString::fromStdString(name);
    }

    Proj->save_newCloud("cloud",newName,cloud);
    QString newFilepath = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(newName);
    openCloudFile(newFilepath,"cloud");
    cloud.reset();
  }
}
void MainWindow::importTerrainFile()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select File"),Proj->get_Path(),
                                                 tr("All Supported Files (*.pcd *.txt *.xyz *.las *.pts *.ptx);;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No File Selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);

    QStringList typ = fileName.split(".");


    if(typ.at(typ.size()-1) == "pcd")
      importPCD(fileName, cloud);
    if(typ.at(typ.size()-1) == "txt")
      importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "xyz")
      importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "las")
      importLAS(fileName, cloud);
    if(typ.at(typ.size()-1) == "pts")
      importPTS(fileName, cloud);
    if(typ.at(typ.size()-1) == "ptx")
      importPTX(fileName, cloud);

    cloud->width = cloud->points.size();
    cloud->is_dense=true;
    cloud->height=1;

    boost::filesystem::path p (fileName.toLatin1().constData());
    std::string name = p.filename().stem().string();
   //if file exist in project folder, ask for another name
    QString name_t = QString::fromStdString(name);
    QString newFile = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name_t);
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name.at(0));
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = QString::fromStdString(name);
    }

    Proj->save_newCloud("teren",newName,cloud );

    QString newFilepath = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(newName);
    openCloudFile(newFilepath,"teren");
    cloud.reset();
  }
}
void MainWindow::importVegeCloud()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select File"),Proj->get_Path(),
                                                 tr("All Supported Files (*.pcd *.txt *.xyz *.las *.pts *.ptx);;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No File Selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);

    QStringList typ = fileName.split(".");

    if(typ.at(typ.size()-1) == "pcd")
       importPCD(fileName, cloud);
    if(typ.at(typ.size()-1) == "txt")
      importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "xyz")
       importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "las")
      importLAS(fileName, cloud);
    if(typ.at(typ.size()-1) == "pts")
       importPTS(fileName, cloud);
    if(typ.at(typ.size()-1) == "ptx")
      importPTX(fileName, cloud);

    boost::filesystem::path p (fileName.toLatin1().constData());
    std::string name = p.filename().stem().string();
   //if file exist in project folder, ask for another name
    QString name_t = QString::fromStdString(name);
    QString newFile = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name_t);
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name.at(0));
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = QString::fromStdString(name);
    }
    Proj->save_newCloud("vege",newName,cloud);
    QString newFilepath = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(newName);
    openCloudFile(newFilepath,"vege");
  }
}
void MainWindow::importTreeCloud()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select File"),Proj->get_Path(),
                                                 tr("All Supported Files (*.pcd *.txt *.xyz *.las *.pts *.ptx);;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No File Selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);

    QStringList typ = fileName.split(".");

    if(typ.at(typ.size()-1) == "pcd")
       importPCD(fileName, cloud);
    if(typ.at(typ.size()-1) == "txt")
      importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "xyz")
       importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "las")
      importLAS(fileName, cloud);
    if(typ.at(typ.size()-1) == "pts")
       importPTS(fileName, cloud);
    if(typ.at(typ.size()-1) == "ptx")
      importPTX(fileName, cloud);



    boost::filesystem::path p (fileName.toLatin1().constData());
    std::string name = p.filename().stem().string();
   //if file exist in project folder, ask for another name
    QString name_t = QString::fromStdString(name);
    QString newFile = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name_t);
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name_t);
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = QString::fromStdString(name);
    }
    Proj->save_newCloud("strom",newName,cloud);
    QString newFilepath = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(newName);
    openCloudFile(newFilepath, "strom");
  }
}
void MainWindow::importOstCloud()
{
  QString selectedFilter;
  QStringList ls = QFileDialog::getOpenFileNames(this,tr("Select File"),Proj->get_Path(),
                                                 tr("All Supported Files (*.pcd *.txt *.xyz *.las *.pts *.ptx);;PCD files (*.pcd);;TXT files (*.txt);;LAS files (*.las);;PTS files (*.pts);;PTX files (*.ptx)" ),
                                                 &selectedFilter);
  if (ls.isEmpty())
  {
    QMessageBox::warning(this,"ERROR","No File Selected");
    return;
  }
  for(int i=0;i<ls.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    QString fileName = ls.at(i);

    QStringList typ = fileName.split(".");

    if(typ.at(typ.size()-1) == "pcd")
       importPCD(fileName, cloud);
    if(typ.at(typ.size()-1) == "txt")
      importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "xyz")
       importTXT(fileName, cloud);
    if(typ.at(typ.size()-1) == "las")
      importLAS(fileName, cloud);
    if(typ.at(typ.size()-1) == "pts")
       importPTS(fileName, cloud);
    if(typ.at(typ.size()-1) == "ptx")
      importPTX(fileName, cloud);

    boost::filesystem::path p (fileName.toLatin1().constData());
    std::string name = p.filename().stem().string();
   //if file exist in project folder, ask for another name
    QString name_t = QString::fromStdString(name);
    QString newFile = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name_t);
    QString newName;
    if(QFile(newFile).exists())
    {
      QString a = QString ("In project folder already exist file with the same name - %1.pcd. Please enter new name for imported file.").arg(name.at(0));
      QMessageBox::information(0,("File exist"), a);
      newName = name_Exists();
    }
    else
    {
      newName = QString::fromStdString(name);
    }
    Proj->save_newCloud("ost",newName,cloud);
    QString newFilepath = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(newName);
    openCloudFile(newFilepath, "ost");
  }
}
//OPEN PROTECTED methods
void MainWindow::openCloudFile(QString file, QString type,bool visible )
{
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    auto r= dis(gen);
    auto g= dis(gen);
    auto b= dis(gen);

  QColor col = QColor(r, g, b);
  openCloudFile(file, type, col, visible);

}
void MainWindow::openCloudFile(QString file, QString type, QColor col,bool visible )
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (file.toUtf8().constData(), *cloud);

  QStringList coords = file.split(QDir::separator ());
  Cloud *c =new Cloud(cloud,coords.back(),col);

  if(type == "teren")
    Proj->set_TerrainCloud(*c);
  if(type == "cloud")
    Proj->set_baseCloud(*c);
  if(type == "vege")
    Proj->set_VegeCloud(*c);
  if(type == "strom")
    Proj->set_Tree(*c);
  if(type == "ost")
    Proj->set_OstCloud(*c);
    if(type == "feature")
    Proj->setFeature(*c);

//if(visible == true)
 // dispCloud(*c);
  addTreeItem(c->get_name(), visible);
  m_vis->resetCamera();
  delete c;
  cloud.reset();
}

//EXPORT method
void MainWindow::exportCloud()
{
  QStringList types;
  types << ".txt" << ".ply" << ".pts"<<".wkt" ;

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud Export");
  in->set_path(Proj->get_Path());
  in->set_description("\tExports selected clouds into a text file.");
  in->set_inputList("Input Tree Cloud:",get_allNames());
  //in->set_inputCloud1("Input cloud:",names);
  in->set_outputDir("Path to the destination folder:","c:\\exported_clouds");
  // chtelo by to nastavit separator
  //chce to rozdÄlit typ souboru

  in->set_outputType("Type of the output file:", types);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Accepted && in->get_inputList().size() > 0 && !in->get_outputDir().isEmpty())
  {
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    for(int i = 0; i < selected.size() ; i++ )
    {
      QString fileName = in->get_outputDir();
      fileName.append(QDir::separator ());
      QString baseName = selected.at(i);
      //QString baseName2 = ;
      fileName.append(baseName.remove(".pcd"));
      if(in->get_outputType()== ".txt")
        fileName.append((".txt"));
      if(in->get_outputType()== ".pts")
      fileName.append((".pts"));
      if(in->get_outputType()== ".ply")
        fileName.append((".ply"));
        if(in->get_outputType()== ".wkt")
        fileName.append((".wkt"));
      QFile file(fileName);

      if(file.exists())
      {
        // zadat nove jmeno
        QString a =QString("File -- %1 -- exist. Skipping export.").arg(fileName);
        QMessageBox::warning(0,("ERROR"),a);
      }
      else
      {
        if(in->get_outputType()== ".txt")
        {
          file.open(QIODevice::WriteOnly | QIODevice::Text);
          QTextStream out(&file);
          out.setRealNumberNotation(QTextStream::FixedNotation);
          out.setRealNumberPrecision(3);
          for(pcl::PointCloud<pcl::PointXYZI>::iterator it = Proj->get_Cloud(selected.at(i)).get_Cloud()->begin(); it != Proj->get_Cloud(selected.at(i)).get_Cloud()->end(); it++)
          {
            double x = it->x - Proj->get_Xtransform();
            double y = it->y - Proj->get_Ytransform();
            double z = it->z - Proj->get_Ztransform();
            out << x << " " << y << " " << z << " " << it->intensity<<"\n";
          }
          file.close();
        }
        if(in->get_outputType()== ".pts")
        {
          file.open(QIODevice::WriteOnly | QIODevice::Text);
          QTextStream out(&file);
          out.setRealNumberNotation(QTextStream::FixedNotation);
          out.setRealNumberPrecision(3);

          int pocetbodu = Proj->get_Cloud(selected.at(i)).get_Cloud()->width;
          out << pocetbodu << "\n";

          for(pcl::PointCloud<pcl::PointXYZI>::iterator it = Proj->get_Cloud(selected.at(i)).get_Cloud()->begin(); it != Proj->get_Cloud(selected.at(i)).get_Cloud()->end(); it++)
          {
            double x = it->x - Proj->get_Xtransform();
            double y = it->y - Proj->get_Ytransform();
            double z = it->z - Proj->get_Ztransform();
            int r = Proj->get_Cloud(selected.at(i)).get_color().red();
            int g = Proj->get_Cloud(selected.at(i)).get_color().green();
            int b = Proj->get_Cloud(selected.at(i)).get_color().blue();
            out << x << " " << y << " " << z << " " << it->intensity<<" " << r << " " << g<<" " << b<<"\n";
          }
          file.close();

        }
        if(in->get_outputType()== ".ply")
        {
          pcl::io::savePLYFileASCII(fileName.toUtf8().constData(),*Proj->get_Cloud(selected.at(i)).get_Cloud());
        }
          if(in->get_outputType()== ".wkt")
          {
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out.setRealNumberNotation(QTextStream::FixedNotation);
            out.setRealNumberPrecision(3);
              out<<"MULTIPOINT(";
              
            for(pcl::PointCloud<pcl::PointXYZI>::iterator it = Proj->get_Cloud(selected.at(i)).get_Cloud()->begin(); it != Proj->get_Cloud(selected.at(i)).get_Cloud()->end(); it++)
            {
              double x = it->x - Proj->get_Xtransform();
              double y = it->y - Proj->get_Ytransform();
              double z = it->z - Proj->get_Ztransform();
              out <<"("<<  x << " " << y << " " << z << ")";
                if(it != Proj->get_Cloud(selected.at(i)).get_Cloud()->end()-1)
                    out<<",";
            }
              out<<")";
            file.close();
          }
      // save file

      }
      showPBarValue(v+= percent);
    }
    removePbar();
  }
  delete in;
}

//EXIT method
void MainWindow::closeEvent(QCloseEvent *event)
{
  //Proj->cleanAll();
  event->accept();
}
//TEREN methods
void MainWindow::voxelgrid()
{
  QStringList names;
  names << get_baseNames();
  names << get_terrainNames();
  names << get_ostNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Terrain by Voxels");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for automatic terrain segmentation. "
                      "It is based on voxelization that converts the input cloud into voxels of a defined resolution. "
                      "Centroids of the lowest voxels are selected and saved as a terrain cloud, and the rest is saved as a vegetation.\n"
                      "\tThis method is well suited for terrain extraction since the result is a regular grid of points. In combination "
                      "with the octree method, it can produce precise and accurate terrain representation usable in GIS or other analysis. "
                      "However, voxelization is not appropriate for further vegetation analysis due to considerable reduction of point-cloud density.");
  in->set_inputCloud1("Input Cloud:",names);
  in->set_outputCloud1("Output Terrain Cloud:","voxel-terrain");
  in->set_outputCloud2("Output Non-ground Cloud:","voxel-vegetation");
  in->set_inputInt("Resolution in [cm]:","20");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    createPBar();
    float res = in->get_intValue()/100.0;

    VoxelTerrain *vc = new VoxelTerrain();
    vc->setBaseCloud(Proj->get_Cloud(in->get_inputCloud1()));
    vc->setResolution(res);
    vc->setVegetationName(in->get_outputCloud2());
    vc->setTerrainName(in->get_outputCloud1());

    m_thread = new QThread();

    connect(m_thread, SIGNAL(started()), vc, SLOT(execute()));
    connect(vc, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
    connect(vc, SIGNAL(sendingVegetation(Cloud *)), this, SLOT(saveVegetation(Cloud *)));
    connect(vc, SIGNAL(sendingTerrain( Cloud *)), this, SLOT(saveTerrain( Cloud *)));
    connect(this, SIGNAL(savedTerrain()), vc, SLOT(hotovo()));

    connect(vc, SIGNAL(finished()),  this, SLOT(removePbar()));
    connect(vc, SIGNAL(finished()),  m_thread, SLOT(quit()));
    connect(vc, SIGNAL(finished()), vc, SLOT(deleteLater()));
    connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));

    m_thread->start();
    vc->moveToThread(m_thread);
  }
  delete in;
}
void MainWindow::octreeSlot()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Terrain by Octree");
  in->set_path(Proj->get_Path());
  //  qWarning<<Proj->get_Path();
  in->set_description("\tThis tool serves for automatic terrain segmentation. "
                      "The octree method is based on a recursive dividing base cloud into a smaller subset of 8 cubes until the size of the cube resolves. "
                      "Cubes containing points and having the lowest z-value are considered to be the ground cubes. "
                      "The terrain is then defined by the points in these ground cubes.");
  in->set_inputCloud1("Input Cloud:",get_baseNames());
  in->set_outputCloud1("Output Terrain Cloud:","Terrain_octree");
  in->set_outputCloud2("Output Non-ground Cloud:","Vegetation_octree");
  in->set_inputInt("Resolution in [cm]:","10");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
      std::cout<<"octreeSlot()\n";
    createPBar();
    float res = in->get_intValue()/100.0;
    OctreeTerrain *oc = new OctreeTerrain();
    oc->setBaseCloud(Proj->get_Cloud(in->get_inputCloud1()));
    oc->setResolution(res);
    oc->setVegetationName(in->get_outputCloud2());
    oc->setTerrainName(in->get_outputCloud1());

    m_thread = new QThread();

    connect(m_thread, SIGNAL(started()), oc, SLOT(execute()));
    connect(oc, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
    connect(oc, SIGNAL(sendingVegetation(Cloud *)), this, SLOT(saveVegetation(Cloud *)));
    connect(oc, SIGNAL(sendingTerrain( Cloud *)), this, SLOT(saveTerrain( Cloud *)));
    connect(this, SIGNAL(savedTerrain()), oc, SLOT(hotovo()));

    connect(oc, SIGNAL(finished()),  this, SLOT(removePbar()));


    connect(oc, SIGNAL(finished()),  m_thread, SLOT(quit()));
    connect(oc, SIGNAL(finished()), oc, SLOT(deleteLater()));
    connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));

    m_thread->start();
    oc->moveToThread(m_thread);
  }
delete in;
}
void MainWindow::manualAdjust()
{
  InputDialog *in = new InputDialog(this);
  in->set_path(Proj->get_Path());
  in->set_title("Manual Adjustment of Terrain Cloud");
  in->set_description("\tThis tool serves for examination and manual editing of terrain point clouds."
                      " Noise points that do not represent the ground surface can be removed. \n"
                      "\tFor editing please press the \'x\' key and draw a selection box by dragging a mouse."
                      " Selected points will be deleted from the cloud representing the terrain and saved separately.");
  in->set_inputCloud1("Input Terrain Cloud:",get_terrainNames());
  in->set_outputCloud1("Output Cloud of Deleted Points:","terrain-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    //zkopirovat do m_cloud
    m_cloud->get_Cloud()->clear();
    QString name = QString("%1.edit").arg(in->get_inputCloud1());
    m_cloud->set_name(name);
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_outputCloud1());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //spustit editacni listu
    addToolBarBreak ();
    editBar = new QToolBar(tr("Edit Bar"),this);
    editBar->setIconSize(QSize(24,24));
    this->addToolBar(editBar);
    stopE = editBar->addAction(QPixmap(":/images/editBar/stopEdit.png"),"Stop EDIT");
    connect(stopE,SIGNAL(triggered()),this,SLOT(manualAdjustStop()));
    undoAct = editBar->addAction(QPixmap(":/images/editBar/undo.png"),"undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    displEC = editBar->addAction(QPixmap(":/images/editBar/displayEditCloud.png"),"Display/hide editing cloud");
    connect(displEC,SIGNAL(triggered()),this,SLOT(displayHideEditCloud()) );
    QAction *exitEC = editBar->addAction(QPixmap(":/images/editBar/exitEdit.png"),"Stop edit without saving anything");
    connect(exitEC,SIGNAL(triggered()),this,SLOT(manualSelectExit()) );

    sliceEC = editBar->addAction(QPixmap(":/images/editBar/slice.png"),"start/stop slice");
    connect(sliceEC,SIGNAL(triggered()),this,SLOT(slice()) );

    nextSliceEC = editBar->addAction(QPixmap(":/images/editBar/next.png"),"next slice");
    nextSliceEC->setEnabled(false);
    connect(nextSliceEC,SIGNAL(triggered()),this,SLOT(nextSlice()) );

    prevSliceEC = editBar->addAction(QPixmap(":/images/editBar/prev.png"),"previous slice");
    prevSliceEC->setEnabled(false);
    connect(prevSliceEC,SIGNAL(triggered()),this,SLOT(prevSlice()) );

    undopoint.clear();
    m_vis->removeAllPointClouds();
    treeWidget->allItemOFF();
    m_editCloud = false;
    displayHideEditCloud();

    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
delete in;
}
void MainWindow::manualAdjustStop()
{
  //save m_cloud1 into ost cloud
  saveCloud(m_cloud1,"ost");
//save m_cloud into old file
  QStringList name = m_cloud->get_name().split(".edit");

  QString path_out = QString ("%1%2%3").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name.at(0));
  pcl::io::savePCDFileBinaryCompressed(path_out.toUtf8().constData(), *m_cloud->get_Cloud());

  Proj->get_Cloud(name.at(0)).set_Cloud(m_cloud->get_Cloud());
  treeWidget->itemON(name.at(0));
  m_vis->removeAllPointClouds();
  dispCloud(name.at(0));
  //opencloud
  //openCloudFile(path_out);
  delete editBar;
  m_cloud->get_Cloud()->clear();
  m_cloud1->get_Cloud()->clear();
  undopoint.clear();
  area.disconnect();
}
void MainWindow::IDWslot()
{
   //inputDialog
  InputDialog *in = new InputDialog(this);
  in->set_title(tr("Inverse Distance Weighting (IDW) Algorithm for Terrain Interpolation"));
  in->set_path(Proj->get_Path());
  in->set_description(tr("IDW interpolates a new surface within a given resolution from the terrain cloud input. "
                         "It serves for filling areas with missing data in the original terrain cloud. "
                         "User specifies the resolution of regular grid in [cm] and the number of surrounding points "
                         "of the original cloud used for the interpolation. The output is a new terrain cloud that consists of the interpolated points."));
  in->set_inputCloud1(tr("Input Terrain Cloud:"), get_terrainNames());
  in->set_outputCloud1(tr("Output Cloud of Ground:"),"idw-");
  in->set_inputInt(tr("Output Resolution in [cm]:"),"20");
  in->set_inputInt2(tr("Number of Surrounding Points:"),"10");

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
  { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    createPBar();
    float res = in->get_intValue()/100.0;


    IDW *idw = new IDW();
    idw->setBaseCloud(Proj->get_Cloud(in->get_inputCloud1()));
    idw->setOutputName(in->get_outputCloud1());
    idw->setResolution(res);
    idw->setPointNumber(in->get_intValue2());

    m_thread = new QThread();

    connect(m_thread, SIGNAL(started()), idw, SLOT(execute()));
    connect(idw, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
    connect(idw, SIGNAL(sendingoutput(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
    connect(this, SIGNAL(savedTerrain()), idw, SLOT(hotovo()));

    connect(idw, SIGNAL(finished()), this , SLOT(removePbar()));
    connect(idw, SIGNAL(finished()),  m_thread, SLOT(quit()));
    connect(idw, SIGNAL(finished()), idw, SLOT(deleteLater()));
    connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));

    m_thread->start();
    idw->moveToThread(m_thread);

  }
  delete in;
}
void MainWindow::statisticalOutlierRemovalTerrain()
{
  InputDialog *in = new InputDialog(this);
  in->set_path(Proj->get_Path());
  in->set_title("Statistical Outlier Removal");
  in->set_description("\t This tool removes outliers using mean k-nearest neighbor distance. All points with a mean distance greater than average are removed from the point cloud.");
  in->set_inputCloud1("Input Terrain Cloud:",get_terrainNames());
  in->set_outputCloud1("Output Terrain Cloud:","terrain-rest-sor");
  in->set_inputInt("Number of Neighbors","3");
  //in->set_inputInt2("Mean Distance Multiplier","0");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    createPBar();
//    float res = in->get_intValue()/100.0;

    StatOutlierRemoval *sor = new StatOutlierRemoval();
    sor->setBaseCloud(Proj->get_Cloud(in->get_inputCloud1()));
    sor->setOutputName(in->get_outputCloud1());
   // sor->setMeanDistance(in->get_intValue2());
    sor->setNeighborhood(in->get_intValue());

    m_thread = new QThread();

    connect(m_thread, SIGNAL(started()), sor, SLOT(execute()));
    connect(sor, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
    connect(sor, SIGNAL(sendingoutput(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
    connect(this, SIGNAL(savedTerrain()), sor, SLOT(hotovo()));

    connect(sor, SIGNAL(finished()), this , SLOT(removePbar()));
    connect(sor, SIGNAL(finished()), m_thread, SLOT(quit()));
    connect(sor, SIGNAL(finished()), sor, SLOT(deleteLater()));
    connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));

    m_thread->start();
    sor->moveToThread(m_thread);
  }
}
void MainWindow::radiusOutlierRemovalTerrain()
{
    InputDialog *in = new InputDialog(this);
  in->set_path(Proj->get_Path());
  in->set_title("Radius Outlier Removal");
  in->set_description("\tThis tool removes all points with fewer neighbors inside the given radius.");
  in->set_inputCloud1("Input Terrain Cloud:",get_terrainNames());
  in->set_outputCloud1("Output Terrain Cloud:","terrain-filterROR");
  in->set_inputInt("Radius [cm]","25");
  in->set_inputInt2("Minimum Neighbors in Radius","100");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }
    float radius = in->get_intValue();
    radius /=100;
  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    createPBar();
    float res = in->get_intValue()/100.0;


    RadiusOutlierRemoval *ror = new RadiusOutlierRemoval();
    ror->setBaseCloud(Proj->get_Cloud(in->get_inputCloud1()));
    ror->setOutputName(in->get_outputCloud1());
    ror->setRadius(res);
    ror->setNeighborhood(in->get_intValue2());

    m_thread = new QThread();

    connect(m_thread, SIGNAL(started()), ror, SLOT(execute()));
    connect(ror, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
    connect(ror, SIGNAL(sendingoutput(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
    connect(this, SIGNAL(savedTerrain()), ror, SLOT(hotovo()));

    connect(ror, SIGNAL(finished()), this , SLOT(removePbar()));
    connect(ror, SIGNAL(finished()), m_thread, SLOT(quit()));
    connect(ror, SIGNAL(finished()), ror, SLOT(deleteLater()));
    connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));

    m_thread->start();
    ror->moveToThread(m_thread);
  }
}
void MainWindow::nextSlice()
{
  m_slides.at(m_slicepos).set_Cloud (m_cloud->get_Cloud());
  m_slicepos++;
  if(m_slicepos==m_slides.size()-1)
    nextSliceEC->setEnabled(false);
  if(m_slicepos>0)
  prevSliceEC->setEnabled(true);

  m_cloud->set_Cloud(m_slides.at(m_slicepos).get_Cloud());
  m_cloud->get_Cloud()->width = m_cloud->get_Cloud()->points.size ();
  m_cloud->get_Cloud()->height = 1;
  m_cloud->get_Cloud()->is_dense = true;
  dispCloud(*m_cloud,220,220,0);
  undopoint.clear();
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::prevSlice()
{
   m_slides.at(m_slicepos).set_Cloud (m_cloud->get_Cloud());

  m_slicepos--;
  if(m_slicepos==0)
    prevSliceEC->setEnabled(false);
  if(m_slicepos<m_slides.size()-1)
    nextSliceEC->setEnabled(true);

  m_cloud->set_Cloud(m_slides.at(m_slicepos).get_Cloud());
  m_cloud->get_Cloud()->width = m_cloud->get_Cloud()->points.size ();
  m_cloud->get_Cloud()->height = 1;
  m_cloud->get_Cloud()->is_dense = true;
  undopoint.clear();
  dispCloud(*m_cloud,220,220,0);
  m_vis->resetCamera();
  qvtkwidget->update();

}
void MainWindow::slice()
{
  InputDialog *in = new InputDialog(this);
  in->set_path(Proj->get_Path());
  in->set_title("Terrain cloud Split");
  in->set_description("\tThis tool creates user-defined strips of terrain for easier point selection.");
  in->set_inputInt("Width of the Slice in [m]:","5");
  in->set_stretch();
  int dl = in->exec();
  if(dl == QDialog::Accepted )
  {
    // vytvorit vektor mracen na zaklade zadaneho parametru
    float dist = in->get_intValue();
    // kolik bude celkem pÃ¡su?
    std::vector< std::vector<int> > indices;
    pcl::PointXYZI minp, maxp; // body ohranicujici vegetaci
    pcl::getMinMax3D(*m_cloud->get_Cloud(),minp, maxp);
    int slice_num = 1+ (maxp.x - minp.x)/dist;
    indices.resize(slice_num);

    for(int i=0; i < m_cloud->get_Cloud()->points.size(); i++)
    {
      // pro kazdy bod zjisti jeho x souradnici odectenou od nejmensi x
      float x = m_cloud->get_Cloud()->points.at(i).x - minp.x;
      int bin = x/dist;
      indices.at(bin).push_back(i);
    }

    for(int u=0; u <indices.size(); u++)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
      for(int z=0; z < indices.at(u).size(); z++)
      {
        pcl::PointXYZI bod = m_cloud->get_Cloud()->points.at(indices.at(u).at(z));
        cloud_->points.push_back(bod);
      }
      cloud_->width = cloud_->points.size ();
      cloud_->height = 1;
      cloud_->is_dense = true;

      Cloud *c = new Cloud();
      c->set_Cloud(cloud_);
      m_slides.push_back(*c);
    }
    m_slicepos=0;
    m_cloud->set_Cloud(m_slides.at(m_slicepos).get_Cloud());
    nextSliceEC->setEnabled(true);
    m_vis->removeAllPointClouds();
    m_editCloud = false;
    undopoint.clear();
    disconnect(sliceEC,SIGNAL(triggered()),this,SLOT(slice()) );
    connect(sliceEC,SIGNAL(triggered()),this,SLOT(sliceStop()) );
    stopE->setEnabled(false);

    dispCloud(*m_cloud,220,220,0);
    m_vis->resetCamera();
    qvtkwidget->update();
  }
 delete in;
}
void MainWindow::sliceStop()
{
  m_slides.at(m_slicepos).set_Cloud (m_cloud->get_Cloud());

  m_cloud->set_Cloud(m_slides.at(m_slicepos).get_Cloud());
  m_cloud->get_Cloud()->width = m_cloud->get_Cloud()->points.size ();
  m_cloud->get_Cloud()->height = 1;
  m_cloud->get_Cloud()->is_dense = true;
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_ (new pcl::PointCloud<pcl::PointXYZI>);
  for(int q=0; q < m_slides.size(); q++)
  {
    *merged_ += *m_slides.at(q).get_Cloud();
  }
  merged_->width = merged_->points.size ();
  merged_->is_dense=true;
  merged_->height=1;

  m_cloud->set_Cloud(merged_);
  undopoint.clear();
  dispCloud(*m_cloud,220,220,0);
  m_vis->resetCamera();
  qvtkwidget->update();
  disconnect(sliceEC,SIGNAL(triggered()),this,SLOT(sliceStop()) );
  connect(sliceEC,SIGNAL(triggered()),this,SLOT(slice()) );
  stopE->setEnabled(true);
}
void MainWindow::slope()
{
    InputDialog *in = new InputDialog(this);
    in->set_title(tr("Terrain Slope"));
    in->set_path(Proj->get_Path());
    in->set_description(tr("This tool computes the slope of terrain based on its neighborhood. The neighborhood can be defined by the number of surrounding points or by the radius."));
    in->set_inputCloud1(tr("Input Terrain Cloud:"), get_terrainNames());
    in->set_outputCloud1(tr("Output Ground Cloud:"),"Slope");
    in->set_inputInt(tr("Number of Surrounding Points:"),"5");
    in->set_inputInt2(tr("Radius in [cm]:"),"10");
    in->set_inputCheckBox("Use radius.");
    in->set_inputCheckBox2("Use percentage.");

    in->set_stretch();
    int dl = in->exec();

    if(dl == QDialog::Rejected)
    { return; }

    if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
    {
        createPBar();
       // float res = in->get_intValue();
        std::cout << "parametry - vystupni cloud name: " << in->get_outputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-teren: " << in->get_inputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-pocet bodu: " << in->get_intValue() << "\n";
        std::cout << "parametry - input-radius: " << in->get_intValue2()/100.0 << "\n";
        std::cout << "parametry - Use radius?: " << in->get_CheckBox() << "\n";
        std::cout << "parametry - Use percent?: " << in->get_CheckBox2() << "\n";

        Slope *s = new Slope();
        s->setTerrainCloud(Proj->get_Cloud(in->get_inputCloud1()));
        s->setOutputName(in->get_outputCloud1());
        s->setNeighbors(in->get_intValue());
        s->setRadius(in->get_intValue2()/100.0);
        s->useRadius(in->get_CheckBox());
        s->setPercent(in->get_CheckBox2());

        m_thread = new QThread();

        connect(m_thread, SIGNAL(started()), s, SLOT(execute()));
        connect(s, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
        connect(s, SIGNAL(sendingoutput(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
        connect(this, SIGNAL(savedTerrain()), s, SLOT(hotovo()));

        connect(s, SIGNAL(finished()), this , SLOT(removePbar()));
        connect(s, SIGNAL(finished()),  m_thread, SLOT(quit()));
        connect(s, SIGNAL(finished()), s, SLOT(deleteLater()));
        connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));

        m_thread->start();
        s->moveToThread(m_thread);

    }
    delete in;
}
void MainWindow::aspect()
{
    InputDialog *in = new InputDialog(this);
    in->set_title(tr("Aspect of Terrain."));
    in->set_path(Proj->get_Path());
    in->set_description(tr("This tool computes the aspect of terrain based on its neighborhood. The neighborhood can be defined by the number of surrounding points or by the radius."));
    in->set_inputCloud1(tr("Input Terrain Cloud:"), get_terrainNames());
    in->set_outputCloud1(tr("Output Ground Cloud:"),"aspect");
    in->set_inputInt(tr("Number of Surrounding Points:"),"5");
    in->set_inputInt2(tr("Radius in [cm]:"),"100");
    in->set_inputCheckBox("Use radius.");
    in->set_inputCheckBox2("Use degrees.");

    in->set_stretch();
    int dl = in->exec();

    if(dl == QDialog::Rejected)
    { return; }

    if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
    {
        createPBar();
       // float res = in->get_intValue();
        std::cout << "parametry - vystupni cloud name: " << in->get_outputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-teren: " << in->get_inputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-pocet bodu: " << in->get_intValue() << "\n";
        std::cout << "parametry - input-radius: " << in->get_intValue2()/100.0 << "\n";
        std::cout << "parametry - Use radius?: " << in->get_CheckBox() << "\n";
        std::cout << "parametry - Use percent?: " << in->get_CheckBox2() << "\n";

        Aspect *s = new Aspect();
        s->setTerrainCloud(Proj->get_Cloud(in->get_inputCloud1()));
        s->setOutputName(in->get_outputCloud1());
        s->setNeighbors(in->get_intValue());
        s->setRadius(in->get_intValue2()/100.0);
        s->useRadius(in->get_CheckBox());
        s->setSmer(in->get_CheckBox2());

        m_thread = new QThread();

        connect(m_thread, SIGNAL(started()), s, SLOT(execute()));
        connect(s, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
        connect(s, SIGNAL(sendingoutput(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
        connect(this, SIGNAL(savedTerrain()), s, SLOT(hotovo()));

        connect(s, SIGNAL(finished()), this , SLOT(removePbar()));
        connect(s, SIGNAL(finished()),  m_thread, SLOT(quit()));
        connect(s, SIGNAL(finished()), s, SLOT(deleteLater()));
        connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));

        m_thread->start();
        s->moveToThread(m_thread);

    }
    delete in;
}
void MainWindow::curvature()
{
    InputDialog *in = new InputDialog(this);
    in->set_title(tr("Terrain Curvature."));
    in->set_path(Proj->get_Path());
    in->set_description(tr("This tool computes the terrain curvature based on its neighborhood. The neighborhood can be defined by the number of surrounding points or by the radius. The terrain curvature is computed as a derivate of intensity value. The terrain curvature input cloud has to be used the slope output cloud as computed by the slope tool."));
    in->set_inputCloud1(tr("Input Cloud:"), get_terrainNames());
    in->set_outputCloud1(tr("Output Ground Cloud:"),"curvature");
    in->set_inputInt(tr("Curvature Limit:"),"5");
    in->set_inputInt2(tr("Radius in [cm]:"),"300");
    in->set_inputCheckBox("Use radius.");

    in->set_stretch();
    int dl = in->exec();

    if(dl == QDialog::Rejected)
    { return; }

    if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
    {
        createPBar();
       // float res = in->get_intValue();
        std::cout << "parametry - vystupni cloud name: " << in->get_outputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-teren: " << in->get_inputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-pocet bodu: " << in->get_intValue() << "\n";
        std::cout << "parametry - input-radius: " << in->get_intValue2()/100.0 << "\n";
        std::cout << "parametry - Use radius?: " << in->get_CheckBox() << "\n";

        Curvature *s = new Curvature();
        s->setTerrainCloud(Proj->get_Cloud(in->get_inputCloud1()));
        s->setOutputName(in->get_outputCloud1());
        s->setNeighbors(in->get_intValue());
        s->setRadius(in->get_intValue2()/100.0);
        s->useRadius(in->get_CheckBox());

        m_thread = new QThread();

        connect(m_thread, SIGNAL(started()), s, SLOT(execute()));
        connect(s, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
        connect(s, SIGNAL(sendingoutput(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
        connect(this, SIGNAL(savedTerrain()), s, SLOT(hotovo()));

        connect(s, SIGNAL(finished()), this , SLOT(removePbar()));
        connect(s, SIGNAL(finished()),  m_thread, SLOT(quit()));
        connect(s, SIGNAL(finished()), s, SLOT(deleteLater()));
        connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));

        m_thread->start();
        s->moveToThread(m_thread);
    }
    delete in;
}
void MainWindow::hillShade()
{
    InputDialog *in = new InputDialog(this);
    in->set_title(tr("Terrain Hillshade"));
    in->set_path(Proj->get_Path());
    in->set_description(tr("This tool computes the hillshade of terrain based on its neighborhood. The neighborhood can be defined by the number of surrounding points or by the radius."));
    in->set_inputCloud1(tr("Input Terrain Cloud:"), get_terrainNames());
    in->set_outputCloud1(tr("Output Ground Cloud of ground:"),"hillShade");
    in->set_inputInt(tr("Number of Surrounding Points:"),"5");
    in->set_inputInt2(tr("Radius in [cm]:"),"10");
    in->set_inputCheckBox("Use radius.");
    
    in->set_stretch();
    int dl = in->exec();
    
    if(dl == QDialog::Rejected)
    { return; }
    
    if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
    {
        createPBar();
        // float res = in->get_intValue();
        std::cout << "parametry - vystupni cloud name: " << in->get_outputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-teren: " << in->get_inputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-pocet bodu: " << in->get_intValue() << "\n";
        std::cout << "parametry - input-radius: " << in->get_intValue2()/100.0 << "\n";
        std::cout << "parametry - Use radius?: " << in->get_CheckBox() << "\n";
        
        HillShade *s = new HillShade();
        s->setTerrainCloud(Proj->get_Cloud(in->get_inputCloud1()));
        s->setOutputName(in->get_outputCloud1());
        s->setNeighbors(in->get_intValue());
        s->setRadius(in->get_intValue2()/100.0);
        s->useRadius(in->get_CheckBox());
        
        m_thread = new QThread();
        
        connect(m_thread, SIGNAL(started()), s, SLOT(execute()));
        connect(s, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
        connect(s, SIGNAL(sendingoutput(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
        connect(this, SIGNAL(savedTerrain()), s, SLOT(hotovo()));
        
        connect(s, SIGNAL(finished()), this , SLOT(removePbar()));
        connect(s, SIGNAL(finished()),  m_thread, SLOT(quit()));
        connect(s, SIGNAL(finished()), s, SLOT(deleteLater()));
        connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));
        
        m_thread->start();
        s->moveToThread(m_thread);
        
    }
    delete in;
}
void MainWindow::pointDensity()
{
    InputDialog *in = new InputDialog(this);
    in->set_title(tr("Cloud Point Density"));
    in->set_path(Proj->get_Path());
    in->set_description(tr("This tool computes the cloud point density based on its neighborhood. The neighborhood can be defined by the number of surrounding points or by the radius."));
    in->set_inputCloud1(tr("Input Terrain Cloud:"), get_terrainNames());
    in->set_outputCloud1(tr("Output Ground Cloud"),"density");
    in->set_inputInt(tr("Number of Surrounding Points:"),"5");
    in->set_inputInt2(tr("Radius in [cm]:"),"100");
    in->set_inputCheckBox("Use radius.");
    
    in->set_stretch();
    int dl = in->exec();
    
    if(dl == QDialog::Rejected)
    { return; }
    
    if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
    {
        createPBar();
        // float res = in->get_intValue();
        std::cout << "parametry - vystupni cloud name: " << in->get_outputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-teren: " << in->get_inputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-pocet bodu: " << in->get_intValue() << "\n";
        std::cout << "parametry - input-radius: " << in->get_intValue2()/100.0 << "\n";
        std::cout << "parametry - Use radius?: " << in->get_CheckBox() << "\n";
        
        PointDensity *s = new PointDensity();
        s->setTerrainCloud(Proj->get_Cloud(in->get_inputCloud1()));
        s->setOutputName(in->get_outputCloud1());
        s->setNeighbors(in->get_intValue());
        s->setRadius(in->get_intValue2()/100.0);
        s->useRadius(in->get_CheckBox());
        
        m_thread = new QThread();
        
        connect(m_thread, SIGNAL(started()), s, SLOT(execute()));
        connect(s, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
        connect(s, SIGNAL(sendingoutput(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
        connect(this, SIGNAL(savedTerrain()), s, SLOT(hotovo()));
        
        connect(s, SIGNAL(finished()), this , SLOT(removePbar()));
        connect(s, SIGNAL(finished()),  m_thread, SLOT(quit()));
        connect(s, SIGNAL(finished()), s, SLOT(deleteLater()));
        connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));
        
        m_thread->start();
        s->moveToThread(m_thread);
        
    }
    delete in;
}
void MainWindow::terrainDiff()
{
    // okno
    InputDialog *in = new InputDialog(this);
    in->set_title(tr("Charcoal sites"));
    in->set_path(Proj->get_Path());
    in->set_description(tr("This tool computes the terrain features (similar places) based on its area concave or convex characteristic, their ratio, number of points with a similar value of intensity, and axis length."));
    in->set_inputCloud1(tr("Input Curvature Cloud:"), get_terrainNames());
    in->set_inputInt(tr("MInimal Intensity Value Limit") ,"-2");
    in->set_inputInt7(tr("Maximal Intenstity Value Limit:"),"4");
    in->set_inputInt2(tr("Minimal Point Size of Feature:"),"35");
    in->set_inputInt10(tr("Maximal Point Size of Feature:"),"350");
    in->set_inputInt3(tr("Minimal Axis Lenght:"),"3");
    in->set_inputInt4(tr("Maximal Axis Length:"),"15");
    in->set_inputInt5(tr("Minimal Area:"),"14");
    in->set_inputInt6(tr("Maximal Area of Feature:"),"100");
    in->set_inputInt8(tr("Min Axis Ratio in [%]:"),"40");
    in->set_inputInt9(tr("Min Area Ratio in [%]:"),"70");
    
    
   // in->set_inputCheckBox("use radius instead of neighbor");
    
    in->set_stretch();
    int dl = in->exec();
    
    if(dl == QDialog::Rejected)
    { return; }
    
    if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
    {
        createPBar();
        // float res = in->get_intValue();
        std::cout << "parametry - vystupni cloud name: " << in->get_outputCloud1().toStdString() << "\n";
        std::cout << "parametry - input-curvature: " << in->get_inputCloud1().toStdString() << "\n";
        std::cout << "parametry - min intensitylimit: " << in->get_intValue() << "\n";
        std::cout << "parametry - point size: " << in->get_intValue2() << "\n";
        std::cout << "parametry - min axis: " << in->get_intValue3() << "\n";
        std::cout << "parametry - max axis: " << in->get_intValue4() << "\n";
        std::cout << "parametry - min area: " << in->get_intValue5() << "\n";
        std::cout << "parametry - max area: " << in->get_intValue6() << "\n";
        std::cout << "parametry - max intensity limit: " << in->get_intValue7() << "\n";
        std::cout << "parametry - axis ratio: " << in->get_intValue8()/100. << "\n";
        std::cout << "parametry - area ratio: " << in->get_intValue9()/100. << "\n";
        
       // std::cout << "parametry - Use radius?: " << in->get_CheckBox() << "\n";
        
        TerrainFeatures *s = new TerrainFeatures();
        s->setTerrainCloud(Proj->get_Cloud(in->get_inputCloud1()));
        //s->setSlopeCloud(Proj->get_Cloud(in->get_inputCloud2()));
        s->setOutputName(in->get_outputCloud1());
        
        s->setMaxBinaryLimit(in->get_intValue7());
        s->setMinBinaryLimit(in->get_intValue());
        
        s->setlowerPointLimit(in->get_intValue2());
        s->setupperPointLimit(in->get_intValue10());
        
        s->setMinAreaLimit(in->get_intValue5());
        s->setMaxAreaLimit(in->get_intValue6());
        s->setAreaRatioLimit(in->get_intValue9()/100.);
        
        s->setMinLenghtLimit(in->get_intValue3());
        s->setMaxLenghtLimit(in->get_intValue4());
        s->setAxisRatioLimit(in->get_intValue8()/100.);
        
        s->setNeighbors(10);
        s->setRadius(300);
      //  s->useRadius(in->get_CheckBox());
        
        m_thread = new QThread();
        
        connect(m_thread, SIGNAL(started()), s, SLOT(execute()));
        connect(s, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
        connect(s, SIGNAL(sendingoutput(Features *)), this, SLOT(saveFeature(Features *)));
        connect(s, SIGNAL(sendingoutputCloud(Cloud *)), this, SLOT(saveTerrain(Cloud *)));
        connect(this, SIGNAL(savedFeature()), s, SLOT(hotovo()));
        
        connect(s, SIGNAL(finished()), this , SLOT(removePbar()));
        connect(s, SIGNAL(finished()),  m_thread, SLOT(quit()));
        connect(s, SIGNAL(finished()), s, SLOT(deleteLater()));
        connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));
        
        m_thread->start();
        s->moveToThread(m_thread);
        
    }
    delete in;
}
void MainWindow::exportFeaturesAtt()
{//input dialog
    // export all features attriubtes into text file

    ExportFeaturesDialog *exdialog = new ExportFeaturesDialog(this);
    exdialog->setDescription("\tThis tool serves for exporting feature parameters into formatted text file. "
                              "The prefix, and the separator within the exported .txt file have to be specified."
                              " Polygon files are using WKT geom field that stores polygons - convave and convex.");
    int dl = exdialog->exec();
    
    if(dl == QDialog::Accepted)
    {
        QString sep =exdialog->getSeparator();
        QString prefix =exdialog->getPrefix();
        QString path =exdialog->getPath();
        int pocetFeatures = Proj->getFeatureSize();
        //std::cout<<"pocet features: "<< pocetFeatures << " sep: "<< sep.toStdString() << " prefix: "<< prefix.toStdString() << " path: " << path.toStdString() << "\n";
        
        if(pocetFeatures ==0)
            return;
        // open file and push header
        // insert values
        // name, centroid X, centroid Y, centroid Z, axis length X, axis length Y, ratio, Convex area, concave area, points size,
        if(exdialog->getFeatureFile() == true) // create tree file
        {
            //open file
            QString filename =QString("%1%2%3featuresAttributes.txt").arg(exdialog->getPath()).arg(QDir::separator()).arg( exdialog->getPrefix());
            QFile file (filename);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out.setRealNumberNotation(QTextStream::FixedNotation);
            out.setRealNumberPrecision(3);
            
            // write header
            out << "ID" << sep <<"Cloud_name" << sep << "X_centroid"<< sep << "Y_centroid"<<sep << "Z_centroid"<< sep <<"Point_size" << sep << "X_Axis_Length_[m]" << sep << "Y_Axis_Length_[m]" << sep << "Axis_ratio" << sep << "Convex_Area_[m3]"<<sep<< "Concave_Area_[m3]" <<"\n";

            // write data
            for(int q=0; q < Proj->getFeatureSize();q++)
            {
                int id =q+1;
                QString name =Proj->getFeature(q).getPointCloud()->get_name();
                int points = Proj->getFeature(q).getPointNumber();
                float xAxis = Proj->getFeature(q).getXlenght();
                float yAxis = Proj->getFeature(q).getYlenght();
                float axisRatio = yAxis/xAxis;
                float convexArea =Proj->getFeature(q).getConvexArea();
                float concaveArea =Proj->getFeature(q).getConcaveArea();
                pcl::PointXYZI bod = Proj->getFeature(q).getCentroid();
                double x = bod.x - Proj->get_Xtransform();
                double y = bod.y - Proj->get_Ytransform();
                double z = bod.z - Proj->get_Ztransform();

                out<< id << sep << name << sep << x << sep << y << sep << z << sep <<  points << sep << xAxis << sep<< yAxis << sep << axisRatio << sep << convexArea<< sep << concaveArea << "\n";
            }
            file.close();
        }
        if(exdialog->getConvexPolygonFile() == true) // create tree file wiht geometry in WKT format and ID
        {
            //open file
            QString filename =QString("%1%2%3featuresConvex.txt").arg(exdialog->getPath()).arg(QDir::separator()).arg( exdialog->getPrefix());
            QFile file (filename);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out.setRealNumberNotation(QTextStream::FixedNotation);
            out.setRealNumberPrecision(3);
            
            // write header
            out << "ID" << sep <<"GEOM\n";
            

            // write data
            for(int q=0; q < Proj->getFeatureSize();q++)
            {
                QString name =Proj->getFeature(q).getPointCloud()->get_name();
                out << name << sep <<"POLYGON((";
                
                for(int j = 0; j < Proj->getFeature(q).getConvexHull().getPolygon()->points.size(); j++)
                {
                    pcl::PointXYZI bod;
                    bod = Proj->getFeature(q).getConvexHull().getPolygon()->points.at(j);
                    double x = bod.x - Proj->get_Xtransform();
                    double y = bod.y - Proj->get_Ytransform();
                    out << x << " " << y<<",";
    
                 }
                pcl::PointXYZI bod;
                bod = Proj->getFeature(q).getConvexHull().getPolygon()->points.at(0);
                double x = bod.x - Proj->get_Xtransform();
                double y = bod.y - Proj->get_Ytransform();
                out << x << " " << y <<"))\n";
            }
            file.close();
        }
        if(exdialog->getConcavePolygonFile() == true) // create tree file
        {
            //open file
            QString filename =QString("%1%2%3featuresConcave.txt").arg(exdialog->getPath()).arg(QDir::separator()).arg( exdialog->getPrefix());
            QFile file (filename);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out.setRealNumberNotation(QTextStream::FixedNotation);
            out.setRealNumberPrecision(3);
            
            // write header
            out << "ID" << sep <<"GEOM\n";

            // write data
            for(int q=0; q < Proj->getFeatureSize();q++)
            {
                QString name =Proj->getFeature(q).getPointCloud()->get_name();
                out << name << sep <<"POLYGON((";
                
                for(int j = 1; j < Proj->getFeature(q).getConcaveHull().getPolygon().get_Cloud()->points.size(); j++)
                {
                    pcl::PointXYZI bod;
                    bod = Proj->getFeature(q).getConcaveHull().getPolygon().get_Cloud()->points.at(j-1);
                    double x = bod.x - Proj->get_Xtransform();
                    double y = bod.y - Proj->get_Ytransform();
                    out << x << " " << y<<",";
                  
                 }
                pcl::PointXYZI bod;
                bod = Proj->getFeature(q).getConvexHull().getPolygon()->points.at(0);
                double x = bod.x - Proj->get_Xtransform();
                double y = bod.y - Proj->get_Ytransform();
                out << x << " " << y <<"))\n";
            }
            file.close();
        }
        
    }
}
//VEGETATION
void MainWindow::manualSelect()
{
  QStringList names ;
  names << get_vegetationNames() << get_ostNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Manual Tree Segmentation");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for manual segmentation of vegetation cloud and creation of single tree clouds. "
                      "The points that do not belong to the target tree can be deleted manually.\n"
                      "\t For editing please press the \'x\' key and draw a selection box by dragging a mouse."
                      " Selected points will be deleted from the view.\n"
                      "\tThe points representing a target tree will be saved as a separate tree cloud by Stop EDIT icon that ends editing mode. "
                      "The filename of the new tree cloud has to be set. Once the segmented tree is saved, another segmentation process can be started (all the removed but unsegmented points will reappear) or terminated. ");
  in->set_inputCloud1("Input Vegetation cloud:",names);
  in->set_outputCloud1("Output Cloud of Deleted Points:","vegetation-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    //set input cloud as m_cloud
    m_cloud->get_Cloud()->clear();
    QStringList name_u = in->get_inputCloud1().split(".pcd");
    m_cloud->set_name(name_u.at(0));
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //set output into m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_outputCloud1());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //spustit editacni listu
    addToolBarBreak ();
    editBar = new QToolBar(tr("Edit Bar"),this);
    editBar->setIconSize(QSize(24,24));
    this->addToolBar(editBar);

    QAction *stopEd = editBar->addAction(QPixmap(":/images/editBar/stopEdit.png"),"Stop Edit");
    connect(stopEd,SIGNAL(triggered()),this,SLOT(manualSelectStop()) );

    QAction *undoAct = editBar->addAction(QPixmap(":/images/editBar/undo.png"),"Undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );

    QAction *displEC = editBar->addAction(QPixmap(":/images/editBar/displayEditCloud.png"),"Display/hide the edit cloud.");
    connect(displEC,SIGNAL(triggered()),this,SLOT(displayHideEditCloud()) );

    QAction *exitEC = editBar->addAction(QPixmap(":/images/editBar/exitEdit.png"),"Stop editing without saving.");
    connect(exitEC,SIGNAL(triggered()),this,SLOT(manualSelectExit()) );



    undopoint.clear();

    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->removeAllPointClouds();
    treeWidget->allItemOFF();
    m_editCloud = false;
    displayHideEditCloud();
    m_vis->resetCamera();
    qvtkwidget->update();

  }
delete in;
}
void MainWindow::manualSelectStop()
{
// save m_cloud as new tree

  QString name;
  bool ok;
  while (name.isEmpty())
  {
    name = QInputDialog::getText(this, tr("New File Name of the Tree"),tr("e.g. tree_id"),QLineEdit::Normal,tr("id_"),&ok );
    if(!ok )
    return;
  }

  QString path = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name);
  QFile treefile(path);
  bool owrt=false;
  while(treefile.exists() && owrt == false)
  {
    QMessageBox::StandardButton rewrite = QMessageBox::question(this,tr("Overwrite file?"),tr("A file with the same name already exists. Do you want to overwrite the file?"),QMessageBox::Yes|QMessageBox::No);
    if(rewrite == QMessageBox::Yes)
      owrt=true;
    else
      return;
  }
  QString a= m_cloud->get_name();
  m_cloud->set_name(name);
  saveCloud(m_cloud, "strom");
  m_cloud->set_name(a);

  //saveTreeCloud(m_cloud->get_Cloud());
  //save m_cloud1 into file
  // if exist update file and memory
  QString file = QString("%1.pcd").arg(m_cloud1->get_name());
 // m_cloud1->set_name(file);
  saveVegeCloud(m_cloud1, true);

//ask if continue
  QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("Continue editing?");
	msgBox->setInformativeText("Do you want to continue editing the cloud?");
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox->setDefaultButton(QMessageBox::Yes);

  if(msgBox->exec() == QMessageBox::Yes)
  {
    treeWidget->allItemOFF();
    m_cloud->get_Cloud()->clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr h_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    *h_cloud += *m_cloud1->get_Cloud();
    m_cloud->set_Cloud(h_cloud);
    m_cloud1->get_Cloud()->points.clear();
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
    h_cloud.reset();
  }
  else
  {
    treeWidget->allItemOFF();
    manualSelectExit();
  }
  delete msgBox;
}

void MainWindow::manualSelectExit()
{
  //remove and clean m_cloud
  removeCloud(m_cloud1->get_name());
  removeCloud(m_cloud->get_name());
  m_cloud->get_Cloud()->points.clear();
  m_cloud1->get_Cloud()->points.clear();
  // remove toolbar
  delete editBar;
  //disconnect selection
  area.disconnect();
  // change vtk ruber band

}

void MainWindow::segmentation()
{
  QStringList names ;
  names << get_vegetationNames() << get_ostNames();

QStringList method ;
    method  << "PCA" << "intensity"<<"slope"<<"PCA*Slope";

  QStringList names2 ;
  names2 << get_terrainNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Automatic Segmentation");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for automatic segmentation of individual trees. There are ten inputs needed to start. "
                      " The vegetation and terrain clouds of interests as input clouds. The voxel size, descriptor type (slope, intensity, PCA), descriptor threshold value (%), "
                      " amount of iterations, number of voxels in element, and the distance from terrain, as well as the method itself, are explained in detail on the wiki section on our GitHub https://github.com/janekT/3DForest. "
                      " The cloud prefix and non-segmented points output name define the output.You have to specify the vegetation cloud containing trees and the terrain cloud for estimating tree base position."
                      " All computations are based on points inside voxel and on distance of voxels. ");
  in->set_inputCloud1("Input Vegetation Cloud:",names);
  in->set_inputCloud2("Input Terrain Cloud:",names2);
  in->set_inputInt("Voxel Size:","4");
  in->set_inputInt2("Descriptor Threshold Value in [%] 0-100:","70");
  in->set_inputInt3("Amount of Iterations:","2");
  in->set_inputInt4("Number of Voxels in Element:","150");
    in->setSboxDouble("Distance from Terrain", 1.6);

  in->set_outputCloud1("Set Cloud Prefix:","ID");
  in->set_outputCloud2("Output of Non-segmented Points:","vegetation-rest");
    in->set_outputType("Descriptor:", method);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted )
  {
      std::cout<<"jedeme\n";
      std::cout << "parametry - prefix: " << in->get_outputCloud1().toStdString() << "\n";
      std::cout << "parametry - vegetace-rest: " << in->get_outputCloud2().toStdString() << "\n";
      std::cout << "parametry - input-vegetace: " << in->get_inputCloud1().toStdString() << "\n";
      std::cout << "parametry - input-veliksot: " << in->get_intValue()/100.0 << "\n";
      std::cout << "parametry - input-range: " << in->get_intValue2()/100.0 << "\n";
      std::cout << "parametry - input-multiplikator: " << in->get_intValue3() << "\n";
      std::cout << "parametry - input-elementy: " << in->get_intValue4() << "\n";
      std::cout << "parametry - input-typ: " << in->get_outputType().toStdString() << "\n";
      std::cout << "parametry - input-distance from terrain: " << in->getSboxValue() << "\n";

      createPBar();
      m_thread = new QThread();
      std::cout<<"thread vytvoren\n";
      Segmentation * seg = new Segmentation(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud(), Proj->get_Cloud(in->get_inputCloud2()).get_Cloud());
      seg->setRestCloudName(in->get_outputCloud2());
      seg->setTreePrefix(in->get_outputCloud1());

      seg->setDistance((float)in->get_intValue()/100.0);
      seg->setMultiplicator(in->get_intValue3());
      seg->setTerrainDistance(in->getSboxValue());
     // seg->setPCAValue(in->get_intValue2());
      seg->setRange(in->get_intValue2()/100.0 );
      seg->setNumOfElements(in->get_intValue4());
      if(in->get_outputType() == "PCA")
          seg->setMethod(1);
      if(in->get_outputType() == "intensity")
          seg->setMethod(2);
      if(in->get_outputType() == "slope")
          seg->setMethod(3);
      if(in->get_outputType() == "PCA*Slope")
          seg->setMethod(4);

   // seg->setMinimalPoint((int) in->get_intValue2());
      std::cout<<"seg nastaveno\n";
    connect(m_thread, SIGNAL(started()), seg, SLOT(execute()));
    connect(seg, SIGNAL(percentage( int )), this, SLOT(showPBarValue( int)));
    connect(seg, SIGNAL(finished()), seg, SLOT(getData()));

    connect(seg, SIGNAL(sendingTree(Cloud *)), this, SLOT(saveTree(Cloud *)),Qt::BlockingQueuedConnection);
    connect(seg, SIGNAL(sendingRest( Cloud *)), this, SLOT(saveVegetation( Cloud *)),Qt::BlockingQueuedConnection);
    connect(seg, SIGNAL(sendingCentr( Cloud *)), this, SLOT(saveVegetation( Cloud *)),Qt::BlockingQueuedConnection);

    connect(seg, SIGNAL(hotovo()),  this, SLOT(removePbar()));
    connect(seg, SIGNAL(hotovo()),  m_thread, SLOT(quit()));
    connect(seg, SIGNAL(hotovo()), seg, SLOT(deleteLater()));
    connect(m_thread, SIGNAL(finished()), m_thread, SLOT(deleteLater()));
      std::cout<<"connect nastaveno\n";
    m_thread->start();
    seg->moveToThread(m_thread);
  }
}
void MainWindow::mergeCloudsByID()
{
    QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("Spoji a vymaze mracna dle ID.");
	msgBox->setInformativeText("pripojeno bude k mracnu s id nebo ID na zacatku. ");
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox->setDefaultButton(QMessageBox::Yes);

    if(msgBox->exec() == QMessageBox::Yes)
    {
        Proj->mergeEraseCloudsByID();
    }
}
void MainWindow::eraseSelectedClouds()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud Erase");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool permanently erases selected clouds from the project and the disc.");
  in->set_inputList("Input Cloud:",get_allNames());
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }
  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    //int v=0;
   // int percent= 100/ selected.size();
    for(int i = 0; i < selected.size() ; i++ )
    {
        removeCloud(selected.at(i));
        Proj->deleteCloudNoQuestions(selected.at(i));
        treeWidget->itemdelete(selected.at(i));
    }
    removePbar();
  }
  delete in;
}

//TREE ATRIBUTES
void MainWindow::treeEdit()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Tree Cloud Edit");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for visual inspection of tree cloud and editing. "
                      " The points that do not belong to the target tree cloud can be deleted manually.\n"
                      "\t For editing please press the \'x\' key and draw a selection box by dragging a mouse."
                      " Selected points will be deleted from tree cloud and saved into a new file.");
  in->set_inputCloud1("Input Tree Cloud:",get_treeNames());
  in->set_outputCloud1("Output Cloud of Deleted Points:","tree-rest");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    //zkopirovat do m_cloud
    m_cloud->get_Cloud()->clear();
    QString name = QString("%1.edit").arg(in->get_inputCloud1());
    m_cloud->set_name(name);
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_outputCloud1());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());


    //spustit editacni listu
    addToolBarBreak ();
    editBar = new QToolBar(tr("Edit Bar"),this);
    editBar->setIconSize(QSize(24,24));
    this->addToolBar(editBar);
    QAction *stopTE = editBar->addAction(QPixmap(":/images/editBar/stopEdit.png"),"Stop Edit");
    connect(stopTE,SIGNAL(triggered()),this,SLOT(treeEditStop()) );
    QAction *undoAct = editBar->addAction(QPixmap(":/images/editBar/undo.png"),"Undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    undopoint.clear();
    QAction *displEC = editBar->addAction(QPixmap(":/images/editBar/displayEditCloud.png"),"Display/hide the edited cloud.");
    connect(displEC,SIGNAL(triggered()),this,SLOT(displayHideEditCloud()) );
    QAction *exitEC = editBar->addAction(QPixmap(":/images/editBar/exitEdit.png"),"Stop editing without saving.");
    connect(exitEC,SIGNAL(triggered()),this,SLOT(manualSelectExit()) );

    treeWidget->allItemOFF();
    m_vis->removeAllPointClouds();
    m_editCloud = false;
    displayHideEditCloud();
    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
delete in;
}
void MainWindow::treeEditStop()
{
  // save m_cloud as new tree - zmenit ulozeni
  QStringList name = m_cloud->get_name().split(".edit");
  if(m_cloud->get_Cloud()->points.size() > 0)
  {
    QStringList name2 = name.at(0).split(".");
    saveTreeCloud(m_cloud->get_Cloud(),name2.at(0),true);
    Proj->get_TreeCloud(name.at(0)).set_Cloud(m_cloud->get_Cloud());
    treeWidget->itemON(name.at(0));
    dispCloud(Proj->get_TreeCloud(name.at(0)));
  }

  if(m_cloud1->get_Cloud()->points.size() > 0)
  {
    saveCloud(m_cloud1, "ost");
  }
  manualSelectExit();
  dispCloud(Proj->get_TreeCloud(name.at(0)));
}
void MainWindow::dbhCloudEdit()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("DBH Cloud Edit");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for the manual editing of the point cloud used for the DBH computation. "
                      "Exclusion of outliers from the DBH cloud can improve the resulted DBH value. "
                      "\t For editing please press the \'x\' key and draw a selection box by dragging a mouse."
                      " Selected points will be temporarily excluded from the DBH computation.");
  in->set_inputCloud1("Input Tree Cloud:",get_treeNames());
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    //check if tree has dbhcloud
    if(Proj->get_TreeCloud(in->get_inputCloud1()).get_pose().z == -1 && Proj->get_TreeCloud(in->get_inputCloud1()).get_pose().y == -1 && Proj->get_TreeCloud(in->get_inputCloud1()).get_pose().z == -1)
    {
      QMessageBox::warning(this, ("Cannot edit the DBH cloud."), ("There is no base position computed for selected tree. Please compute the base position before the DBH estimation or choose different DBH cloud. "));
      return;
    }

    //zkopirovat do m_cloud
    m_cloud->get_Cloud()->clear();
    QString name = QString("%1.edit").arg(in->get_inputCloud1());
    m_cloud->set_name(name);
    m_cloud->set_Cloud(Proj->get_TreeCloud(in->get_inputCloud1()).get_dbhCloud());
    m_cloud->set_Psize(Proj->get_TreeCloud(in->get_inputCloud1()).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name("rest");
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());


    //spustit editacni listu
    addToolBarBreak ();
    editBar = new QToolBar(tr("edit bar"),this);
    this->addToolBar(editBar);
    editBar->setIconSize(QSize(24,24));
    QAction *stopTE = editBar->addAction(QPixmap(":/images/editBar/stopEdit.png"),"Stop Edit");
    connect(stopTE,SIGNAL(triggered()),this,SLOT(treeEditStop()) );
    QAction *undoAct = editBar->addAction(QPixmap(":/images/editBar/undo.png"),"Undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    undopoint.clear();
    QAction *displEC = editBar->addAction(QPixmap(":/images/editBar/displayEditCloud.png"),"Display/hide edited cloud");
    connect(displEC,SIGNAL(triggered()),this,SLOT(displayHideEditCloud()) );
    QAction *exitEC = editBar->addAction(QPixmap(":/images/editBar/exitEdit.png"),"Stop editing without saving.");
    connect(exitEC,SIGNAL(triggered()),this,SLOT(manualSelectExit()) );

    treeWidget->allItemOFF();
    m_vis->removeAllPointClouds();
    m_vis->removeAllShapes();
    displayHideEditCloud();
    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
delete in;
}
void MainWindow::dbhCloudStopEdit()
{
  // save m_cloud as new tree_dbh -
  QStringList name = m_cloud->get_name().split(".edit");

  Proj->set_dbhCloud(name.at(0),m_cloud->get_Cloud());
  Proj->set_treeDBH_HT(name.at(0));
  Proj->set_treeDBH_LSR(name.at(0));
  manualSelectExit();
}
void MainWindow::treeAtributes()
{
//TODO: volit ktere parametry budou zapsany
  ExportAttr *exdialog = new ExportAttr (this);
  exdialog->set_description("\tThis tool serves for exporting tree parameters into formatted text files. "
                              "The trees, the tree parameters, and the separator within the exported .txt file have to be specified. "
                              "The tool exports currently computed values only.");
  //exdialog->set_trees(names);
  exdialog->set_list(get_treeNames());

  int dl = exdialog->exec();
  if(dl == QDialog::Accepted && exdialog->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =exdialog->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();

    QFile file (exdialog->get_outputFile());
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);

    //header
    out << "Cloud_name" ;
    if(exdialog->get_Points() == true)
      out << exdialog->get_separator()<< "Points" ;
    if(exdialog->get_Position() == true)
      out << exdialog->get_separator()<< "X_coord_pos" << exdialog->get_separator()<< "Y_coord_pos" << exdialog->get_separator()<< "Z_coord_pos";
    if(exdialog->get_Height() == true)
      out << exdialog->get_separator()<< "Height" ;
    if(exdialog->get_Length() == true)
      out << exdialog->get_separator()<< "Length" ;
    if(exdialog->get_DBH_HT() == true)
      out << exdialog->get_separator()<< "DBH_HT" << exdialog->get_separator()<< "X_coord_HT"<< exdialog->get_separator()<< "Y_coord_HT";
    if(exdialog->get_DBH_LSR() == true)
      out << exdialog->get_separator()<< "DBH_LSR" << exdialog->get_separator()<< "X_coord_LSR"<< exdialog->get_separator()<< "Y_coord_LSR";
    if(exdialog->get_areaconvex() == true)
      out << exdialog->get_separator()<< "CVex_area" ;
    if(exdialog->get_areaconcave() == true)
      out << exdialog->get_separator()<< "CCave_area" ;
      if(exdialog->get_volume() == true)
          out << exdialog->get_separator()<< "QSM_Volume" ;
      if(exdialog->get_sortiments() == true)
          out << exdialog->get_separator()<< "Sort_1-volume" << exdialog->get_separator()<< "Sort_1-lenght"<< exdialog->get_separator()<< "Sort_2-volume"<< exdialog->get_separator()<< "Sort_2-lenght"<< exdialog->get_separator()<< "Sort_3-volume"<< exdialog->get_separator()<< "Sort_3-lenght"<< exdialog->get_separator()<< "Sort_4-volume"<< exdialog->get_separator()<< "Sort_4-lenght"<< exdialog->get_separator()<< "Sort_5-volume"<< exdialog->get_separator()<< "Sort_5-lenght"<< exdialog->get_separator()<< "Sort_6-volume"<< exdialog->get_separator()<< "Sort_6-lenght";
    out <<"\n";


    for(int i = 0; i < selected.size() ; i++ )
    {
      Tree *c = new Tree(Proj->get_TreeCloud(selected.at(i)));
      out << c->get_name();

      if(exdialog->get_Points() == true)
      {
        out << exdialog->get_separator() << c->get_Cloud()->points.size() ;
      }
      if(exdialog->get_Position() == true)
      {
        //c->set_position();
        pcl::PointXYZI p;
        p = c->get_pose();
        if(p.x ==-1 && p.y == -1 && p.z== -1)
          out << exdialog->get_separator() << "-1" << exdialog->get_separator() <<"-1" <<exdialog->get_separator() <<"-1"  ;
        else
          out << exdialog->get_separator() << (p.x - Proj->get_Xtransform()) << exdialog->get_separator() << (p.y - Proj->get_Ytransform()) << exdialog->get_separator() << (p.z - Proj->get_Ztransform()) ;
      }
      if(exdialog->get_Height() == true)
      {
        //c->set_height();
        out << exdialog->get_separator() << c->get_height();
      }
      if(exdialog->get_Length() == true)
      {
       //c->set_length();
        out << exdialog->get_separator() << c->get_length();
      }
      if(exdialog->get_DBH_HT() == true)
      {
        stred s;
        //c->set_dbhHT();
        s = c->get_dbhHT();
        if(s.a ==-1 && s.b == -1 && s.z== -1 && s.r == -0.5)
          out << exdialog->get_separator() << "-1" << exdialog->get_separator() <<"-1" <<exdialog->get_separator() <<"-1" ;
        else
          out << exdialog->get_separator() << (s.r*2) << exdialog->get_separator() << (s.a - Proj->get_Xtransform()) << exdialog->get_separator() << (s.b - Proj->get_Ytransform()) ;
      }
      if(exdialog->get_DBH_LSR() == true)
      {
        stred s;
        //c->set_dbhLSR();
        s = c->get_dbhLSR();
        if(s.a ==-1 && s.b == -1 && s.z== -1 && s.r == -0.5)
          out << exdialog->get_separator() << "-1" << exdialog->get_separator() <<"-1" <<exdialog->get_separator() <<"-1"  ;
        else
          out << exdialog->get_separator() << (s.r*2) << exdialog->get_separator()<< (s.a - Proj->get_Xtransform()) << exdialog->get_separator() << (s.b - Proj->get_Ytransform());
      }
      if(exdialog->get_areaconvex() == true)
      {
        out << exdialog->get_separator()<< c->getConvexAreaToInfoLine();
      }
      if(exdialog->get_areaconcave() == true)
      {
        out << exdialog->get_separator()<< c->getConcaveAreaToInfoLine();
      }
        if(exdialog->get_volume() == true)
        {
            out << exdialog->get_separator()<< c->getVolume();
        }
        if(exdialog->get_sortiments() == true)
        {
            out << exdialog->get_separator()<< c->getSortimentVolume(1)<< exdialog->get_separator()<< c->getSortimentLenght(1)
            << exdialog->get_separator()<< c->getSortimentVolume(2)<< exdialog->get_separator()<< c->getSortimentLenght(2)
            << exdialog->get_separator()<< c->getSortimentVolume(3)<< exdialog->get_separator()<< c->getSortimentLenght(3)
            << exdialog->get_separator()<< c->getSortimentVolume(4)<< exdialog->get_separator()<< c->getSortimentLenght(4)
            << exdialog->get_separator()<< c->getSortimentVolume(5)<< exdialog->get_separator()<< c->getSortimentLenght(5)
            << exdialog->get_separator()<< c->getSortimentVolume(6)<< exdialog->get_separator()<< c->getSortimentLenght(6);
        }
      out <<"\n";
      delete c;
      showPBarValue(v+= percent);
    }
    file.close();
    removePbar();
  }
  delete exdialog;
}
void MainWindow::convexhull()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Convex Planar Projection");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes the convex planar projection of trees. The method is based on the convex hull of the tree cloud(s) orthogonally projected to the horizontal plane. "
                      "It provides polygon of the maximal projected area occupied by the tree. "
                      "The polygon is displayed at the height of the tree base position.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }
  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    std::cout << " convex hull\n";
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    //#pragma omp parallel for
    for(int i = 0; i < selected.size(); i++ )
    {
      std::cout << selected.at(i).toStdString() << "\n";
      Proj->get_TreeCloud(selected.at(i)).setConvexhull();
     convexhullDisplay(selected.at(i));
      showPBarValue(v+= percent);
    }
   // convexhull_DisplayAll();
    removePbar();

    convexT->setEnabled(true);
    exportCONVEXAct->setEnabled(true);
    refreshAttTable();
  }
  delete in;
}
void MainWindow::convexhullDisplay(QString name)
{
  pcl::PointXYZI bod;
  bod = Proj->get_TreeCloud(name).get_pose();
  if(bod.x ==-1 && bod.y==-1 && bod.z ==-1)
    return;
  //Color
  QColor col = Proj->get_TreeCloud(name).get_color();
 //addAreaText

  QString h = QString("%1 sq m").arg(Proj->get_TreeCloud(name).getConvexAreaToInfoLine(),0,'f',2);
  std::stringstream name2 ;
  name2 << name.toUtf8().constData() << "_vexText";
  m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.2,0.3,0,name2.str());

  //addpolygon
  std::stringstream Pname;
  Pname << name.toUtf8().constData() << "_convex_polygon";
  m_vis->addPolygon<pcl::PointXYZI>(Proj->get_TreeCloud(name).getConvexhull().getPolygon(),col.redF(),col.greenF(), col.blueF(), Pname.str());
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, Pname.str() );
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, Pname.str() );
  m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION , pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE , Pname.str() );

  disconnect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_DisplayAll()));
  connect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_HideAll()));
  convexT->setIcon(QPixmap(":/images/treeBar/convex_sel.png"));
}
void MainWindow::convexhull_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  if(names.size()>0)
  {
    createPBar();

    for(int i = 0; i < names.size(); i++)
    {
      convexhullDisplay(names.at(i));
      pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
    convexT->setIcon(QPixmap(":/images/treeBar/convex_sel.png"));
  }
}
void MainWindow::convexhull_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove polygon
    std::stringstream Pname ;
    Pname  << names.at(i).toUtf8().constData() << "_convex_polygon";
    m_vis->removeShape (Pname.str());

    //remove text
    std::stringstream Tname ;
    Tname  << names.at(i).toUtf8().constData() << "_vexText";
    m_vis->removeText3D (Tname.str());
  }
  disconnect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_HideAll()));
  connect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_DisplayAll()));
   convexT->setIcon(QPixmap(":/images/treeBar/convex.png"));
  qvtkwidget->update();
}
void MainWindow::concavehull()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Concave Planar Projection");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes and displays the concave planar projection of tree(s) using the concave hull "
                      "of all tree points orthogonally projected to the horizontal plane. "
                      "It is based on minimizing the polygon edge based on user-specified maximal distance.\n"
                      "\tThe actual computed area is smaller than the one provided by the convex hull, however, it produces a more detailed results. "
                      "The area can differ according to the used search distance.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_inputInt("Initial Search Distance in [cm]:","100");

  in->set_stretch();
  int dl = in->exec();


  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    std::cout << " concave hull\n";
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
   // #pragma omp parallel for
    for(int i = 0; i < selected.size() ; i++ )
    {
      //std::cout << selected.at(i).toStdString() << "\n";
      Proj->get_TreeCloud(selected.at(i)).setConcavehull((float)in->get_intValue()/100);
      concavehullDisplay(selected.at(i));
      showPBarValue(v+= percent);
    }
   // concavehull_DisplayAll();
    removePbar();

    concaveT->setEnabled(true);
    concaveT->setIcon(QPixmap(":/images/treeBar/concave_sel.png"));
    exportCONCAVEAct->setEnabled(true);
    refreshAttTable();
  }
  delete in;
}
void MainWindow::concavehullDisplay(QString name)
{
    //get color
    QColor col = Proj->get_TreeCloud(name).get_color();
    //addAreaText
    pcl::PointXYZI bod;
    bod = Proj->get_TreeCloud(name).get_pose();

    QString h = QString("%1 sq m").arg(Proj->get_TreeCloud(name).getConcaveAreaToInfoLine(),0,'f',2);
    std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_caveText";

    std::stringstream name3 ;
    name3 << name.toUtf8().constData() << "_caveMesh";
    m_vis->addText3D(h.toUtf8().constData(),bod,0.6,0.2,0.5,0,name2.str());

    pcl::PolygonMesh *p = new pcl::PolygonMesh(Proj->get_TreeCloud(name).getTriangulatedConcaveHull());
    m_vis->addPolygonMesh(*p,name3.str());
    m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, name3.str());
    m_vis->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, col.redF(), col.greenF(), col.blueF(),name3.str());

  disconnect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_DisplayAll()));
  connect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_HideAll()));
  concaveT->setIcon(QPixmap(":/images/treeBar/concave_sel.png"));
}
void MainWindow::concavehull_DisplayAll()
{
    QStringList names;
    names << get_treeNames();

    createPBar();

    for(int i = 0; i < names.size(); i++)
    {
        if(Proj->get_TreeCloud(names.at(i)).getConcaveAreaToInfoLine()>0){
        concavehullDisplay(names.at(i));
        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
        }
    }
    statusBar()->removeWidget(pBar);
    concaveT->setIcon(QPixmap(":/images/treeBar/concave_sel.png"));
}
void MainWindow::concavehull_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {

    //remove mesh
    std::stringstream Pname ;
    Pname  << names.at(i).toUtf8().constData() << "_caveMesh";
    m_vis->removePolygonMesh(Pname.str());

    //remove text
    std::stringstream Tname ;
    Tname  << names.at(i).toUtf8().constData() << "_caveText";
    m_vis->removeText3D (Tname.str());
  }
  disconnect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_HideAll()));
  connect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_DisplayAll()));
  concaveT->setIcon(QPixmap(":/images/treeBar/concave.png"));
  qvtkwidget->update();
}
void MainWindow::dbhCheck()
{
  // pro each tree
  QStringList trees;
  std::vector<float> hts;
  std::vector<float> lsrs;
    for(int i = 0; i < Proj->get_sizeTreeCV() ; i++ )
    {
        stred lsr;
        lsr = Proj->get_TreeCloud(i).get_dbhLSR();
        stred ht;
        ht = Proj->get_TreeCloud(i).get_dbhHT();
        if(ht.r == -0.5 || lsr.r == -0.5)
        continue;
            QString a = QString("HT %1 LSR %2").arg(ht.r).arg(lsr.r);
            QMessageBox::information(0,("WARNING"),a);
        //if the difference is greater that 10cm
        if((ht.r - lsr.r) > 8 || (ht.r - lsr.r) < -8)
        {
            QString a = QString("%1 ").arg(ht.r - lsr.r);
            QMessageBox::information(0,("WARNING"),a);
            //pass name of the tree to the list.
            trees << Proj->get_TreeCloud(i).get_name();
            hts.push_back(ht.r);
            lsrs.push_back(lsr.r);
        }
    }
    if(hts.size() > 0)
    {
        // save the list in the text file.
        QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),Proj->get_Path(),tr("files (*.txt)"));
        //zapisovat jednotlive radky
        QFile file (newFile);
        file.open(QIODevice::WriteOnly | QIODevice::Text);
        QTextStream out(&file);
        out << "tree name HT LSR\n";
        for(int j = 1; j < trees.size(); j++)
        {
            out << trees.at(j) <<" " << hts.at(j) <<" " <<  lsrs.at(j)<<"\n";
        }
        file.close();
    }
}
void MainWindow::dbhHT()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("DBH by Randomized Hough Transformation (RHT)");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes the tree(s) DBH (Diameter at Breast Height)."
                      "The DBH is represented as a circle whose center and diameter are calculated using the Randomized Hough Transform (RHT) algorithm. "
                      "The number of iterations significantly affects the computational time and accuracy. The recommended minimum of iterations is 200. "
                      "The resulting DBH is displayed as the 10 cm high cylinder with the corresponding center and diameter.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_inputInt("Number of Iterations:", "200");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
      std::cout<<"jedeme\n";

      std::cout << "parametry - input-iterace: " << in->get_intValue() << "\n";
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    //#pragma omp parallel for
    for(int i = 0; i < selected.size() ; i++ )
    {
      Proj->get_TreeCloud(selected.at(i)).set_dbhHT((int)in->get_intValue());
      dbhHTDisplay(Proj->get_TreeCloud(selected.at(i)).get_name());
      showPBarValue(v+= percent);
    }
    removePbar();
    dbhthT->setEnabled(true);
    refreshAttTable();

  }
  delete in;
}
void MainWindow::dbhHTDisplay(QString name)
{
  if(Proj->get_TreeCloud(name).get_dbhHT().r == -0.5)
  {
    return;
  }
  stred x;
  x = Proj->get_TreeCloud(name).get_dbhHT();
    // zobrazit cylinder a hodnotu
  if(x.r < 5000 && x.r > 0.5)
  {
    QString cl_name = QString ("%1_dbh").arg(name);
    Cloud *cl_ = new Cloud(Proj->get_TreeCloud(name).get_dbhCloud(),cl_name ) ;
    dispCloud(*cl_,255, 0, 0);
    delete cl_;
          //Coeff
    pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
    coef->values.push_back((float)x.a);
    coef->values.push_back((float)x.b);
    coef->values.push_back((float)x.z-0.05);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0.1);
    coef->values.push_back((float)x.r/100);
    std::stringstream Cname ;
    Cname << name.toUtf8().constData() << "_cylinder_HT";
    m_vis->removeShape(Cname.str());
    m_vis->addCylinder(*coef,Cname.str());
    m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.0, 0, Cname.str() );

      //addtext3D with R
    pcl::PointXYZ bod;
    bod.x=x.a+(float)x.r/100;
    bod.y=x.b;
    bod.z=x.z+0.1;
    QString h= QString("%1").arg(x.r*2.0);
    std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_value_HT";
    m_vis->removeText3D(name2.str());
    m_vis->addText3D(h.toUtf8().constData(),bod,0.8,0.8,0.5,0,name2.str());
    coef.reset();
    }
  else
  {
    QString m = QString("Computed DBH of the tree '%1' is out of range 0 - 50m.\n"
                        "Please check the tree DBH cloud. If needed edit points using cloud editing tool.").arg(name);
    //QMessageBox::information(0,("Warning"),m);
  }
  //disconnect and connect different for treebar
  disconnect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_DisplayAll()));
  connect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_HideAll()));
  dbhthT->setIcon(QPixmap(":/images/treeBar/dbhHT_sel.png"));
}
void MainWindow::dbhHT_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  if(names.size()>0)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);
    for(int i = 0; i < names.size(); i++)
    {
      dbhHTDisplay(names.at(i));
      pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
    dbhthT->setIcon(QPixmap(":/images/treeBar/dbhHT_sel.png"));
  }
}
void MainWindow::dbhHT_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove cylinder
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_cylinder_HT";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_value_HT";
    m_vis->removeText3D (Tname.str());

    //remove dbhCloud
    std::stringstream Clname ;
    Clname  << names.at(i).toUtf8().constData() << "_dbh";
    m_vis->removePointCloud(Clname.str());
  }
  disconnect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_HideAll()));
  connect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_DisplayAll()));
  dbhthT->setIcon(QPixmap(":/images/treeBar/dbhHT.png"));
  qvtkwidget->update();
}
void MainWindow::dbhLSR()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("DBH by Least Square Regression (LSR)");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes the tree(s) DBH (Diameter at Breast Height). "
                      "The DBH is calculated as a circle fitted to the DBH subset of the tree cloud by the Least Squares Regression (LSR). "
                      "The resulting DBH is displayed as the 10 cm high cylinder with the corresponding center and diameter.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }
  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    //#pragma omp parallel for
    for(int i = 0; i < selected.size() ; i++ )
    {
      Proj->get_TreeCloud(selected.at(i)).set_dbhLSR();
      dbhLSRDisplay(Proj->get_TreeCloud(selected.at(i)).get_name());
      showPBarValue(v+= percent);
    }
    removePbar();
    dbhlsrT->setEnabled(true);
    refreshAttTable();
  }
  delete in;
}
void MainWindow::dbhLSRDisplay(QString name)
{
  if(Proj->get_TreeCloud(name).get_dbhLSR().r == -0.5)
  {
    return;
  }
  stred x;
  x = Proj->get_TreeCloud(name).get_dbhLSR();
    // zobrazit cylinder a hodnotu
  if(x.r < 5000 && x.r > 0.5)
  {
    QString cl_name = QString ("%1_dbh").arg(name);
    Cloud *cl_ = new Cloud(Proj->get_TreeCloud(name).get_dbhCloud(),cl_name ) ;
    dispCloud(*cl_,255, 0, 0);
          //Coeff
    pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
    coef->values.push_back((float)x.a);
    coef->values.push_back((float)x.b);
    coef->values.push_back((float)x.z-0.05);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0.1);
    coef->values.push_back((float)x.r/100);
    std::stringstream Cname ;
    Cname << name.toUtf8().constData() << "_cylinder_LSR";
    m_vis->removeShape(Cname.str());
    m_vis->addCylinder(*coef,Cname.str());
    m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.9, 0, Cname.str() );

      //addtext3D with R
    pcl::PointXYZ bod;
    bod.x=x.a+(float)x.r/100;
    bod.y=x.b;
    bod.z=x.z+0.1;
    QString h= QString("%1").arg(x.r*2.0);
    std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_value_LSR";
    m_vis->removeText3D(name2.str());
    m_vis->addText3D(h.toUtf8().constData(),bod,0.8,0.2,0.8,0,name2.str());
    coef.reset();
  }
  else
  {
    QString m = QString("Computed DBH of the tree '%1' is out of range 0 - 50m.\n"
                        "Please check the tree DBH cloud. If needed edit points using DBH cloud editing tool.").arg(name);
    QMessageBox::information(0,("Warning"),m);
  }
  //disconnect and connect different for treebar
  disconnect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_DisplayAll()));
  connect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_HideAll()));
  dbhlsrT->setIcon(QPixmap(":/images/treeBar/dbhLSR_sel.png"));
}
void MainWindow::dbhLSR_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  if(names.size()>0)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);

    for(int i = 0; i < names.size(); i++)
    {
      dbhLSRDisplay(names.at(i));
      pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
    dbhlsrT->setIcon(QPixmap(":/images/treeBar/dbhLSR_sel.png"));
  }
}
void MainWindow::dbhLSR_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove cylinder
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_cylinder_LSR";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_value_LSR";
    m_vis->removeText3D (Tname.str());

    //remove dbhCloud
    std::stringstream Clname ;
    Clname  << names.at(i).toUtf8().constData() << "_dbh";
    m_vis->removePointCloud(Clname.str());
  }
  disconnect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_HideAll()));
  connect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_DisplayAll()));
  dbhlsrT->setIcon(QPixmap(":/images/treeBar/dbhLSR.png"));
  qvtkwidget->update();
}
void MainWindow::height()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Tree Height");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes the tree height as a difference of Z coordinates between the highest point of the tree cloud and the tree base position. "
                      "The result is displayed as a label on the top of the tree. The vertical line at the tree base position depicts the height itself.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }
  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =in->get_inputList();
   // createPBar();
    //int v=0;
   // int percent= 100/ selected.size();
   // #pragma omp parallel for
    for(int i = 0; i < selected.size() ; i++ )
    {
      std::cout<<selected.at(i).toStdString()<< "\n";

      Proj->get_TreeCloud(selected.at(i)).set_height();
      heightDisplay(selected.at(i));
     // showPBarValue(v+= percent);
    }
    //height_DisplayAll();
  //  removePbar();
    heightT->setEnabled(true);
    setCrownManualAct->setEnabled(true);
    setCrownAutomaticAct->setEnabled(true);
    treeReconstructionAct->setEnabled(true);
    refreshAttTable();
  }
  delete in;
}
void MainWindow::heightDisplay(QString name)
{
  if(Proj->get_TreeCloud(name).get_height() == -1)
  {
    return;
  }
  std::stringstream Hname ;
  Hname << name.toUtf8().constData() << "_heightline";
  pcl::PointXYZI minp,maxp;
  minp = Proj->get_TreeCloud(name).get_pose();
  maxp.x = Proj->get_TreeCloud(name).get_pose().x;
  maxp.y = Proj->get_TreeCloud(name).get_pose().y;
  maxp.z = Proj->get_TreeCloud(name).get_pose().z + Proj->get_TreeCloud(name).get_height();
  m_vis->removeShape(Hname.str());
  m_vis->addLine(minp,maxp,Hname.str());

  std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_heighttext";

  QString h = QString("%1").arg(Proj->get_TreeCloud(name).get_height());
  m_vis->removeText3D(name2.str());
  m_vis->addText3D(h.toUtf8().constData(),maxp,0.8,0.6,0.5,0,name2.str());

  heightT->setIcon(QPixmap(":/images/treeBar/height_sel.png"));
  disconnect(heightT,SIGNAL(triggered()),this,SLOT(height_DisplayAll()));
  connect(heightT,SIGNAL(triggered()),this,SLOT(height_HideAll()));
}
void MainWindow::height_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  if(names.size()>0)
  {
 // pBar = new QProgressBar(statusBar());
  //pBar->setMaximumSize(200,16);
  //statusBar()->addWidget(pBar);
  //pBar->setValue(0);
  for(int i = 0; i < names.size(); i++)
  {
    heightDisplay(names.at(i));
    //pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
   // pBar->update();
  }
   // statusBar()->removeWidget(pBar);
    heightT->setIcon(QPixmap(":/images/treeBar/height_sel.png"));
  }
}
void MainWindow::height_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove line
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_heightline";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_heighttext";
    m_vis->removeText3D (Tname.str());

  }
  disconnect(heightT,SIGNAL(triggered()),this,SLOT(height_HideAll()));
  connect(heightT,SIGNAL(triggered()),this,SLOT(height_DisplayAll()));
  heightT->setIcon(QPixmap(":/images/treeBar/height.png"));
  qvtkwidget->update();
}
void MainWindow::length()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud Length");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes Euclidean distance between the most distant points in the tree cloud and displays their connection. "
                      "The cloud length is displayed in meters at the bottom of the tree. "
                      "This tool is suitable for the calculation of the real length of inclined or lying trees.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { delete in;
    return; }
  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    for(int i = 0; i < selected.size() ; i++ )
    {
      Proj->set_length(selected.at(i));
      lengthDisplay(selected.at(i));
      showPBarValue(v+= percent);
    }
    removePbar();
    lengthT->setEnabled(true);
    refreshAttTable();
  }
  delete in;
}
void MainWindow::lengthDisplay(QString name)
{
  if(Proj->get_TreeCloud(name).get_length() == -1)
  {
    return;
  }
  std::stringstream Lname ;
  Lname << name.toUtf8().constData() << "_length";
  m_vis->removeShape(Lname.str());
  m_vis->addLine(Proj->get_TreeCloud(name).get_lpoint(true),Proj->get_TreeCloud(name).get_lpoint(false),Lname.str());
  std::stringstream Lname2 ;
  Lname2 << name.toUtf8().constData() << "_lengthText";
  QString h= QString("%1").arg(Proj->get_TreeCloud(name).get_length());
  m_vis->removeText3D(Lname2.str());
  m_vis->addText3D(h.toUtf8().constData(),Proj->get_TreeCloud(name).get_lpoint(false),0.8,0.6,0.5,0,Lname2.str());

  disconnect(lengthT,SIGNAL(triggered()),this,SLOT(length_DisplayAll()));
  connect(lengthT,SIGNAL(triggered()),this,SLOT(length_HideAll()));
  lengthT->setIcon(QPixmap(":/images/treeBar/length_sel.png"));
}
void MainWindow::length_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  if(names.size()>0)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);
    for(int i = 0; i < names.size(); i++)
    {
      lengthDisplay(names.at(i));
      pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
    lengthT->setIcon(QPixmap(":/images/treeBar/length_sel.png"));
  }
}
void MainWindow::length_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove line
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_length";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_lengthText";
    m_vis->removeText3D (Tname.str());

  }
  disconnect(lengthT,SIGNAL(triggered()),this,SLOT(length_HideAll()));
  connect(lengthT,SIGNAL(triggered()),this,SLOT(length_DisplayAll()));
  lengthT->setIcon(QPixmap(":/images/treeBar/length.png"));
  qvtkwidget->update();
}
void MainWindow::position()
{
  QStringList names;
  names << get_terrainNames();
  names <<"NO_Terrain";

  InputDialog *in = new InputDialog(this);
  in->set_title("Tree Position by Lowest Point");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes the tree base position. "
                      "The XY position is computed as a median coordinate of all points that are lying "
                      "between the lowest point of the tree cloud and the user-defined distance. "
                      "The Z coordinate is defined as the median Z value of N closest points of the terrain at the XY position. If there is no terrain cloud available, the Z value is defined by the lowest tree point."
                      "The tree base position is displayed as a sphere with the center at the position and radius of 5 cm. ");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_inputCloud2("Input Terrain Cloud:",names);
  in->set_inputInt("Height Above the Lowest Point in [cm]:", "60");
  in->set_inputInt2("Number of Points Used for Terrain Height Estimation:", "5");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {

    createPBar();
    QStringList selected;
    selected =in->get_inputList();
    int v=0;
    int percent= 100/ selected.size();
    #pragma omp parallel for
    for(int i = 0; i < selected.size(); i++ )
    {
      if(in->get_inputCloud2() == "NO_Terrain")
        Proj->set_treePosition(selected.at(i), in->get_intValue());
      else
        Proj->set_treePosition(selected.at(i),Proj->get_TerrainCloud(in->get_inputCloud2()),in->get_intValue2(), in->get_intValue());

      positionDisplay(selected.at(i));
      showPBarValue(v+= percent);
    }
    removePbar();
    positionT->setEnabled(true);

    //enable other calculations
    dbhEditAct->setEnabled(true);
    dbhHTAct->setEnabled(true);
    dbhLSRAct->setEnabled(true);
    heightAct->setEnabled(true);
    lengAct->setEnabled(true);
    //treeReconstructionAct->setEnabled(true);
    convexAct->setEnabled(true);
    concaveAct->setEnabled(true);
    stemCurvatureAct->setEnabled(true);
    tAAct->setEnabled(true);

    //actualize all
    stemCurvature_HideAll();
    height_HideAll();
    dbhLSR_HideAll();
    dbhHT_HideAll();
    recomputeAfterTreePosChenge();
    refreshAttTable();
  }
  delete in;
}
void MainWindow::positionDisplay(QString name)
{
  if(Proj->get_TreeCloud(name).get_pose().z == -1 && Proj->get_TreeCloud(name).get_pose().y == -1 && Proj->get_TreeCloud(name).get_pose().z == -1)
  {
    return;
  }

 // QColor col = Proj->get_TreeCloud(name).get_color();

  std::stringstream Sname;
  Sname << name.toUtf8().constData() << "_sphere";

//add text nazev stromu
//
//  std::stringstream name2 ;
//  name2 << name.toUtf8().constData()<< "_p\n" << Proj->get_TreeCloud(name).get_pose().x << "\n" << Proj->get_TreeCloud(name).get_pose().y << "\n"<< Proj->get_TreeCloud(name).get_pose().z;
//  m_vis->removeText3D(name2.str());
//  m_vis->addText3D(name2.str(),Proj->get_TreeCloud(name).get_pose(),0.8,0.6,0.5,0,name2.str());


  if(!m_vis->updateSphere(Proj->get_TreeCloud(name).get_pose(),0.1,1,1,1,Sname.str()))
    m_vis->addSphere(Proj->get_TreeCloud(name).get_pose(),0.1,1,1,1,Sname.str());

  disconnect(positionT,SIGNAL(triggered()),this,SLOT(position_DisplayAll()));
  connect(positionT,SIGNAL(triggered()),this,SLOT(position_HideAll()));
  positionT->setIcon(QPixmap(":/images/treeBar/position_sel.png"));
}
void MainWindow::position_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  if(names.size()>0)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);
    for(int i = 0; i < names.size(); i++)
    {
      positionDisplay(names.at(i));
      pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
    positionT->setIcon(QPixmap(":/images/treeBar/position_sel.png"));
  }
}
void MainWindow::position_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove sphere
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_sphere";
    m_vis->removeShape (Cname.str());

    std::stringstream name2 ;
    name2 << names.at(i).toUtf8().constData()<< "_p";
    m_vis->removeText3D(name2.str());


  }
  disconnect(positionT,SIGNAL(triggered()),this,SLOT(position_HideAll()));
  connect(positionT,SIGNAL(triggered()),this,SLOT(position_DisplayAll()));
  positionT->setIcon(QPixmap(":/images/treeBar/position.png"));
  qvtkwidget->update();
}
void MainWindow::positionHT()
{
  QStringList names;
  names << get_terrainNames();
  names <<"NO_Terrain";

  InputDialog *in = new InputDialog(this);
  in->set_title("Tree Position by Randomized Hough Transformation (RHT)");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes the tree base position using centers of two circles fitted by RHT to the stem at 1.3 and 0.65 m above the initial or previously computed position. "
                      "The RHT position is then defined as the intersection of the vector aiming from the upper circle center (at 1.3 m) to the lower circle center (at 0.65 m) and "
                      "the horizontal plane defined by the median Z value of the N closest points of the terrain cloud.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_inputCloud2("Input Terrain Cloud:",names);
  in->set_inputInt("Number of RHT Iterations:","200");
  in->set_inputInt2("Number of Points for Terrain Height Estimation:","5");
  //in->set_inputCheckBox("Recalculate parameters (DBH cloud, Height) based on tree position?");

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    createPBar();
    QStringList selected;
    selected =in->get_inputList();
    int v=0;
    int percent= 100/ selected.size();
    for(int i = 1; i < selected.size(); i++ )
    {
      if(in->get_inputCloud2() == "NO_Terrain")
        Proj->set_treePositionHT(selected.at(i), in->get_intValue());
      else
        Proj->set_treePositionHT(selected.at(i),Proj->get_TerrainCloud(in->get_inputCloud2()),in->get_intValue2(), in->get_intValue());

      positionDisplay(selected.at(i));
      showPBarValue(v+= percent);
    }
    removePbar();
    positionT->setEnabled(true);
    //enable other calculations
    dbhEditAct->setEnabled(true);
    dbhHTAct->setEnabled(true);
    dbhLSRAct->setEnabled(true);
    heightAct->setEnabled(true);
    lengAct->setEnabled(true);
    treeReconstructionAct->setEnabled(true);
    convexAct->setEnabled(true);
    concaveAct->setEnabled(true);
    stemCurvatureAct->setEnabled(true);
    tAAct->setEnabled(true);

    //hide all dependencies on position
    stemCurvature_HideAll();
    height_HideAll();
    dbhLSR_HideAll();
    dbhHT_HideAll();
    recomputeAfterTreePosChenge();
    refreshAttTable();
  }
  delete in;
}
void MainWindow::treeReconstruction()
{
    InputDialog *in = new InputDialog(this);
    in->set_title("Tree Reconstruction");
    in->set_path(Proj->get_Path());
    in->set_description("This tool reconstructs tree into stem and branches. Results are saved as intensity field. ");
    in->set_inputList("Input Tree Cloud:",get_treeNames());
    in->set_inputInt("Voxel Size:", "4");
    in->setSboxDouble("Multiplicator:", 3.0);
    in->set_stretch();
    int dl = in->exec();

    if(dl == QDialog::Rejected)
    { return; }

    if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
    {
        QStringList selected;
        selected =in->get_inputList();
        int v=0;
        int percent= 100/ selected.size();
        for(int i=0; i < in->get_inputList().size(); i++)
        {
            std::cout<< selected.at(i).toStdString()<<"\n";
            Proj->setSkeleton(selected.at(i), in->get_intValue()/100.0, in->getSboxValue());
            showPBarValue(v+= percent);
            treeReconstructionDisplay(selected.at(i));
        }
         statusBar()->removeWidget(pBar);
        reconstructionAct->setEnabled(true);
        exportQSMAct->setEnabled(true);
        refreshAttTable();
    }
    delete in;
}
void MainWindow::sortimenty()
{

    InputDialog *in = new InputDialog(this);
    in->set_title("Tree Assortment");
    in->set_path(Proj->get_Path());
    in->set_description("This tool evaluates attributes of the tree based on convergence, skewness, diameter, length, and number of connected branches. ");
    in->set_inputList("Input Tree Cloud:",get_treeNames());
    in->set_inputCheckBox("Display assortment?");
    in->set_stretch();
    int dl = in->exec();

    if(dl == QDialog::Rejected)
    { return; }

    if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
    {
        QStringList selected;
        selected =in->get_inputList();
        int v=0;
        int percent= 100/ selected.size();
        for(int i=0; i < selected.size(); i++)
        {
            std::cout<<"sortimenty tree: "<< selected.at(i).toStdString() << "\n";
            Proj->setSortimens(selected.at(i), in->get_CheckBox());
            showPBarValue(v+= percent);
            sortimentDisplay(selected.at(i));
        }
        statusBar()->removeWidget(pBar);
        reconstructionAct->setEnabled(true);
        refreshAttTable();
    }
    delete in;
}
void MainWindow::sortimentDisplay(QString name)
{
    std::vector< std::shared_ptr <Sortiment>> sortimenty =  Proj->get_TreeCloud(name).getSortiments();
   // std::cout<< "celkovy pocet sortimentu stromu: " << sortimenty.size()<<"\n";
    for(int i =0; i < sortimenty.size(); i++)
    {
        int quality = sortimenty.at(i)->getGrade();
        if (quality >5 )
            continue;
      //  std::cout<<"sortiment "<< i << " kvality "<< quality <<  " ma " <<sortimenty.at(i)->getCylinders().size() << " cylinderu\n";
        for(int q=0; q < sortimenty.at(i)->getCylinders().size(); q++)
        {
            std::stringstream Cname ;
            Cname << name.toUtf8().constData()<< "_" << i << "_cylinders_sortiments"<< q ;
            m_vis->removeShape(Cname.str());
            m_vis->addCylinder(*sortimenty.at(i)->getCylinders().at(q),Cname.str());

            if(quality == 1)
            {
                m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.84, 0, Cname.str() ); //Gold
                //std::cout<< " sortiment trida " <<Proj->get_TreeCloud(name).getSortiment(i) << "\n";
            }
            else if (quality == 2)
            {
                m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.75, 0.78, 0.73, Cname.str() ); //silver
                //std::cout<< " sortiment trida " <<Proj->get_TreeCloud(name).getSortiment(i) << "\n";
            }
            else if (quality == 3)
            {
                m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.545, 0.27, 0.07, Cname.str() ); //brown
                //std::cout<< " sortiment trida " <<Proj->get_TreeCloud(name).getSortiment(i) << "\n";
            }
            else if (quality == 4)
            {
                m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.99, 0.69, Cname.str() ); //cyan
                //std::cout<< " sortiment trida " <<Proj->get_TreeCloud(name).getSortiment(i) << "\n";
            }
            else
            {
                m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.58, 0.93, 0.17, Cname.str() ); //green
                //std::cout<< " sortiment trida " <<Proj->get_TreeCloud(name).getSortiment(i) << "\n";
            }
        }
    }
        disconnect(sortimentT,SIGNAL(triggered()),this,SLOT(sortimentDisplay_DisplayAll()));
        connect(sortimentT,SIGNAL(triggered()),this,SLOT(sortimentDisplay_HideAll()));
        sortimentT->setIcon(QPixmap(":/images/treeBar/sortiment_sel.png"));
        sortimentT->setEnabled(true);
    qvtkwidget->update();
}
void MainWindow::sortimentDisplay_DisplayAll()
{
    QStringList names;
    names << get_treeNames();
    for(int i=0; i < names.size();i++)
    {
        sortimentDisplay(names.at(i));
    }
    qvtkwidget->update();
}
void MainWindow::sortimentDisplay_HideAll()
{
    QStringList names;
    names << get_treeNames();
    for(int q=0; q < names.size();q++)
    {
        std::vector< std::shared_ptr <Sortiment>> sortimenty =  Proj->get_TreeCloud(names.at(q)).getSortiments();
        for(int i =0; i < sortimenty.size(); i++)
        {


            for(int e=0; e < sortimenty.at(i)->getCylinders().size(); e++)
            {
                std::stringstream Cname ;
                Cname << names.at(q).toUtf8().constData()<< "_" << i << "_cylinders_sortiments"<< e ;
                m_vis->removeShape(Cname.str());
            }
        }
    }

    disconnect(sortimentT,SIGNAL(triggered()),this,SLOT(sortimentDisplay_HideAll()));
    connect(sortimentT,SIGNAL(triggered()),this,SLOT(sortimentDisplay_DisplayAll()));
    sortimentT->setIcon(QPixmap(":/images/treeBar/sortiment.png"));
    qvtkwidget->update();
}
void MainWindow::treeReconstructionDisplay(QString name)
{
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI,std::vector<float>> setCloud(c->getCloud(),c->getIntensityVector());

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color(Proj->get_Cloud(name).get_Cloud(), "intensity");

    if(!m_vis->updatePointCloud<pcl::PointXYZI>(Proj->get_Cloud(name).get_Cloud(), color, name.toStdString()))
        m_vis->addPointCloud<pcl::PointXYZI>(Proj->get_Cloud(name).get_Cloud(), color, name.toStdString());
    m_vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::PCL_VISUALIZER_LUT_JET, name.toStdString());

    //    PCL_VISUALIZER_LUT_JET
    //    PCL_VISUALIZER_LUT_JET_INVERSE
    //    PCL_VISUALIZER_LUT_HSV
    //    PCL_VISUALIZER_LUT_HSV_INVERSE
    //    PCL_VISUALIZER_LUT_GREY
    //    PCL_VISUALIZER_LUT_BLUE2RED
    //    PCL_VISUALIZER_LUT_RANGE_AUTO

    m_vis->updatePointCloud<pcl::PointXYZI>(Proj->get_Cloud(name).get_Cloud(), color, name.toStdString());
    qvtkwidget->update();

}
void MainWindow::treeReconstructionDisplayAll()
{
//skeletonT->setIcon(QPixmap(":/images/treeBar/skeleton_sel.png"));
}
void MainWindow::treeReconstructionHideAll()
{
//skeletonT->setIcon(QPixmap(":/images/treeBar/skeleton.png"));
}
void MainWindow::stemCurvature()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Tree Stem Curve");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool computes stem profile. "
                      "The stem centers and diameters are estimated by the randomized Hough transformation algorithm in defined sections above the tree base. "
                      "The sections are 0.65 m, 1.3 m, 2 m and then every meter. The amount of RHT iterations is defined by the user. "
                      "The high number of iterations significantly increases the computational time and improves accuracy. "
                      "At least 200 iterations are recommended. The result is displayed as cylinders fitted on stems in corresponding heights.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_inputInt("Number of RHT Iterations:","200");
  in->set_inputInt2("Section Height:","50");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }
  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    //#pragma omp parallel for
    for(int i = 0; i < selected.size() ; i++ )
    {
      Proj->set_treeStemCurvature(selected.at(i), in->get_intValue(),in->get_intValue2() );
      stemCurvatureDisplay(selected.at(i));
      showPBarValue(v+= percent);
    }
    removePbar();
    stemCurveT->setEnabled(true);
    exportStemCurvAct->setEnabled(true);
    refreshAttTable();
  }
  delete in;
}
void MainWindow::stemCurvatureDisplay(QString name)
{
  if(Proj->get_TreeCloud(name).get_stemCurvature().empty())
  {
    return;
  }
  std::vector<stred> streds = Proj->get_TreeCloud(name).get_stemCurvature();

  for(int i =0; i <streds.size(); i++)
  {
    if(streds.at(i).r < 1 || streds.at(i).r >500 )
      continue;
    //Coeff
    pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
    coef->values.push_back((float)streds.at(i).a);
    coef->values.push_back((float)streds.at(i).b);
    coef->values.push_back((float)streds.at(i).z-0.035);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0);
    coef->values.push_back((float)0.05);
    coef->values.push_back((float)streds.at(i).r/100);
    std::stringstream Cname ;
    Cname << name.toUtf8().constData() << "_ST_"<< i ;
    m_vis->removeShape(Cname.str());
    m_vis->addCylinder(*coef,Cname.str());

    m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.78, 0.38, 0.27, Cname.str() );
    coef.reset();
    pcl::PointXYZI bod;
    bod.x = streds.at(i).a;
    bod.y = streds.at(i).b;
    bod.z = streds.at(i).z;
  }
  disconnect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_DisplayAll()));
  connect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_HideAll()));
  stemCurveT->setIcon(QPixmap(":/images/treeBar/stemCurve_sel.png"));
}
void MainWindow::stemCurvature_DisplayAll()
{
  QStringList names;
  names << get_treeNames();
  if(names.size()>0)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);
    for(int i = 0; i < names.size(); i++)
    {
      stemCurvatureDisplay(names.at(i));
      pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
      pBar->update();
    }
    statusBar()->removeWidget(pBar);
    qvtkwidget->update();
  }
}
void MainWindow::stemCurvature_HideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int j = 0; j < names.size(); j++)
  {
    std::vector<stred> streds = Proj->get_TreeCloud(names.at(j)).get_stemCurvature();
    for(int i =0; i <streds.size(); i++)
    {
      std::stringstream Cname ;
      Cname << names.at(j).toUtf8().constData() << "_ST_"<< i ;
      m_vis->removeShape(Cname.str());
    }
  }
  disconnect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_HideAll()));
  connect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_DisplayAll()));
  stemCurveT->setIcon(QPixmap(":/images/treeBar/stemCurve.png"));
  qvtkwidget->update();
}
void MainWindow::stemCurvatureExport()
{
// for given tree
// write all ring into file in format: tree_name first row diameter second row X third Y fourth Z
  QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Stem Curvature Export");
  in->set_path(Proj->get_Path());
  in->set_description("This tool exports previously computed stem curvature into .txt file.");
  in->set_inputCloud1("Input Tree Cloud:",names);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setValue(0);
    //vybrat jmeno noveho souboru
    QString newFile = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("files (*.txt)"));
   //zapisovat jednotlive radky
    QFile file (newFile);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);

    if(in->get_inputCloud1() == "All_trees" )
    {
      for(int i = 1; i < names.size(); i++)
      {

        std::vector<stred> streds = Proj->get_TreeCloud(names.at(i)).get_stemCurvature();
        out << names.at(i) << " diameter:";
        for(int j = 0; j <streds.size(); j++)
        {
          out << " " << streds.at(j).r *2;
        }
        out << "\n";
        out << names.at(i) << " x:";
        for(int j = 0; j <streds.size(); j++)
        {
          if (streds.at(j).a == -1)
            out << " " << streds.at(j).a;
          else
          out << " " << streds.at(j).a - Proj->get_Xtransform();
        }
        out << "\n";
        out << names.at(i) << " y:";
        for(int j = 0; j <streds.size(); j++)
        {
          if (streds.at(j).a == -1)
            out << " " << streds.at(j).b;
          else
            out << " " << streds.at(j).b - Proj->get_Ytransform();
        }
        out << "\n";
        out << names.at(i) << " z:";
        for(int j = 0; j < streds.size(); j++)
        {
          if (streds.at(j).a == -1)
            out << " " << streds.at(j).z;
          else
          out << " " << streds.at(j).z - Proj->get_Ztransform();
        }
        out << "\n";

        pBar->setValue((i+1)*100/Proj->get_sizeTreeCV());
        pBar->update();
      }
    }
    else
    {
      QString dia_line = in->get_inputCloud1();
      QString x_line = in->get_inputCloud1();
      QString y_line = in->get_inputCloud1();
      QString z_line = in->get_inputCloud1();
      std::vector<stred> streds = Proj->get_TreeCloud(in->get_inputCloud1()).get_stemCurvature();
      for(int i =0; i <streds.size(); i++)
      {
        QString dia = QString(" %1").arg(streds.at(i).r *2);
        QString x = QString(" %1").arg(QString::number(streds.at(i).a - Proj->get_Xtransform()));
        QString y = QString(" %1").arg(QString::number(streds.at(i).b - Proj->get_Ytransform()));
        QString z = QString(" %1").arg(QString::number(streds.at(i).z - Proj->get_Ztransform()));
        dia_line.append(dia);
        x_line.append(x);
        y_line.append(y);
        z_line.append(z);
      }
      out << dia_line << "\n" << x_line << "\n" << y_line << "\n" << z_line << "\n" ;
      pBar->setValue(100);
      pBar->update();
    }
    file.close();
    statusBar()->removeWidget(pBar);
  }
  delete in;
}
void MainWindow::exportConvexTxt()
{
 QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Convex Planar Projection Export");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool exports polygon of the convex planar projection into text file. "
                      "The .txt file includes coordinates of the convex hull vertices of the tree planar projection.");
  //in->set_inputCloud1("Input Cloud:",names);
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_outputPath("Name and Location of Exported File:","c:\\exported_convex_clouds.txt", "Text file (*.txt)");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    // prepare file
    QFile file (in->get_outputPath());
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);
    out << "ID;Xstart;Ystart;Xend;Yend;Z \n";
    //#pragma omp parallel for
    for(int i = 0; i < selected.size() ; i++ )
    {
      for(int j = 1; j < Proj->get_TreeCloud(selected.at(i)).getConvexhull().getPolygon()->points.size(); j++)
      {
        pcl::PointXYZI bod;
        bod = Proj->get_TreeCloud(selected.at(i)).getConvexhull().getPolygon()->points.at(j-1);
        double x = bod.x - Proj->get_Xtransform();
        double y = bod.y - Proj->get_Ytransform();
        out << selected.at(i) << ";" << x << ";" << y ;
        bod = Proj->get_TreeCloud(selected.at(i)).getConvexhull().getPolygon()->points.at(j);
        double xend = bod.x - Proj->get_Xtransform();
        double yend = bod.y - Proj->get_Ytransform();
        double z = bod.z;
        out << ";" << xend << ";" << yend << ";" << z << endl;
      }
      showPBarValue(v+= percent);
    }
    removePbar();
  }
  delete in;
}
void MainWindow::exportConcaveTxt()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Concave Planar Projection Export");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool exports polygon of the concave planar projection into text file. "
                      "The .txt file includes coordinates of the concave hull vertices of the tree planar projection.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_outputPath("Name and Location of Exported File:","c:\\exported_concave_clouds.txt", "Text file (*.txt)");
  in->set_stretch();
  int dl = in->exec();


  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    // prepare file
    QFile file (in->get_outputPath());
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);
    out << "ID;Xstart;Ystart;Xend;Yend;Z \n";
    //#pragma omp parallel for
    for(int i = 0; i < selected.size() ; i++ )
    {
      for(int j = 1; j < Proj->get_TreeCloud(selected.at(i)).getConcavehull().getPolygon().get_Cloud()->points.size(); j++)
      {
        pcl::PointXYZI bod;
        bod = Proj->get_TreeCloud(selected.at(i)).getConcavehull().getPolygon().get_Cloud()->points.at(j-1);
        double x = bod.x - Proj->get_Xtransform();
        double y = bod.y - Proj->get_Ytransform();
        out << selected.at(i) << ";" << x << ";" << y ;
        bod = Proj->get_TreeCloud(selected.at(i)).getConcavehull().getPolygon().get_Cloud()->points.at(j);
        double xend = bod.x - Proj->get_Xtransform();
        double yend = bod.y - Proj->get_Ytransform();
        double z = bod.z;
        out << ";" << xend << ";" << yend << ";" << z << endl;
      }
      showPBarValue(v+= percent);
    }
    removePbar();
  }
  delete in;
}
void MainWindow::qsmModel()
{
    InputDialog *in = new InputDialog(this);
    in->set_title("QSM Reconstruction");
    in->set_path(Proj->get_Path());
    in->set_description("\tThis tool reconstructs a tree as a cylindrical object and computes its volume. ");
    in->set_inputList("Input Tree Cloud:",get_treeNames());
    in->set_inputInt("Set Number of Iteration in Hough Transformation", "150");
    in->set_inputInt2("Set Size of Cylinder in [cm]:", "20");
    in->setSboxDouble("Set Minimal Diameter of Displayed Cylinder in [cm]:",0);
    in->set_inputInt3 ("Set Order to Compute: ","2");
    in->set_inputInt4 ("Set Minimal Lenght of Branch in [cm]: ","100");
    in->set_inputCheckBox("Compute stem profile too?");
  //in->set_outputCloud1("Name of output cloud:","Centroids");
    in->set_stretch();
    int dl = in->exec();

    if(dl == QDialog::Rejected)
    { return; }

    if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
    {
        std::cout << "parametry - pocet iterace: " << in->get_intValue() << "\n";
        std::cout << "parametry - velikost cylinderu: " << in->get_intValue2()/100.0<< "\n";
        std::cout << "parametry - do ktereho radu: " << in->get_intValue3() << "\n";
        std::cout << "parametry - imix delka vetve: " << in->get_intValue4()/100.0 << "\n";
        std::cout << "parametry - min diameter = limit: " << in->getSboxValue()/200.0 << "\n";
        std::cout << "parametry - compute stem profile: " << in->get_CheckBox() << "\n";

        QStringList selected;
        selected =in->get_inputList();
        int v=0;
        int percent= 100/ selected.size();
        for(int i=0; i < in->get_inputList().size(); i++)
        {
            //(QString name, int iterations, float CylinderHeight, float limit, float branchLength, int order);
            Proj->setCylinders(selected.at(i),in->get_intValue(), in->get_intValue2()/100.0, in->getSboxValue()/100.0, in->get_intValue4()/100.0, in->get_intValue3(), in->get_CheckBox() );
            showPBarValue(v+= percent);
            qsmDisplay(selected.at(i));
            if(in->get_CheckBox() == true)
            {
                stemCurveT->setEnabled(true);
                stemCurvatureDisplay(selected.at(i));
            }

        }
        statusBar()->removeWidget(pBar);
        sortimentAct->setEnabled(true);
        refreshAttTable();

    }
    delete in;
}
void MainWindow::qsmDisplay(QString name)
{
    for(int i =0; i < Proj->get_TreeCloud(name).getCylinderSize(); i++)
    {
        std::stringstream Cname ;
        Cname << name.toUtf8().constData() << "_cylinders_"<< i ;
        m_vis->removeShape(Cname.str());
        m_vis->addCylinder(*Proj->get_TreeCloud(name).getCylinder(i),Cname.str());

        m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.78, 0.78, 0, Cname.str() );
    }

//    // pro kazdy segment zobraz kouli v connpoint
//    for(int q=0; q < Proj->get_TreeCloud(name).getBranches().size();q++)
//    {
//        std::stringstream Sname;
//        Sname << name.toUtf8().constData() << "_sphere" << q;
//
//        if(!m_vis->updateSphere(Proj->get_TreeCloud(name).getBranches().at(q)->getConnPoint(),0.05,0.3,1,0.7,Sname.str()))
//            m_vis->addSphere(Proj->get_TreeCloud(name).getBranches().at(q)->getConnPoint(),0.05,0.3,1,0.7,Sname.str());
//    }
//
    ///QSM -DBH

//    stred x;
//    x = Proj->get_TreeCloud(name).getDBH_QSM();
//    // zobrazit cylinder a hodnotu
//        QString cl_name = QString ("%1_dbhQSM").arg(name);
//        Cloud *cl_ = new Cloud(Proj->get_TreeCloud(name).get_dbhCloud(),cl_name ) ;
//        dispCloud(*cl_,255, 0, 0);
//        delete cl_;
//        //Coeff
//        pcl::ModelCoefficients::Ptr coef (new pcl::ModelCoefficients ());
//        coef->values.push_back((float)x.a);
//        coef->values.push_back((float)x.b);
//        coef->values.push_back((float)x.z-0.05);
//        coef->values.push_back((float)0);
//        coef->values.push_back((float)0);
//        coef->values.push_back((float)0.1);
//        coef->values.push_back((float)x.r/100);
//        std::stringstream Cname ;
//        Cname << name.toUtf8().constData() << "_cylinder_QSM";
//        m_vis->removeShape(Cname.str());
//        m_vis->addCylinder(*coef,Cname.str());
//        m_vis->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.3, 0.8, Cname.str() );

    //
    disconnect(skeletonT,SIGNAL(triggered()),this,SLOT(qsmDisplay_DisplayAll()));
    connect(skeletonT,SIGNAL(triggered()),this,SLOT(qsmDisplay_HideAll()));
    skeletonT->setIcon(QPixmap(":/images/treeBar/skeleton_sel.png"));
    skeletonT->setEnabled(true);
}
void MainWindow::qsmDisplay_DisplayAll()
{
    QStringList names;
    names << get_treeNames();
    for(int i=0; i < names.size();i++)
    {
        qsmDisplay(names.at(i));
    }
    qvtkwidget->update();
}
void MainWindow::qsmDisplay_HideAll()
{
    QStringList names;
    names << get_treeNames();
    for(int j = 0; j < names.size(); j++)
    {
        int streds = Proj->get_TreeCloud(names.at(j)).getCylinderSize() + 1000;
        for(int i =0; i <streds; i++)
        {
            std::stringstream Cname ;
            Cname << names.at(j).toStdString()<< "_cylinders_"<< i ;
            m_vis->removeShape(Cname.str());
        }
    }

    disconnect(skeletonT,SIGNAL(triggered()),this,SLOT(qsmDisplay_HideAll()));
    connect(skeletonT,SIGNAL(triggered()),this,SLOT(qsmDisplay_DisplayAll()));
    skeletonT->setIcon(QPixmap(":/images/treeBar/skeleton.png"));
    qvtkwidget->update();
}
//CROWN
void MainWindow::set_CrownManual()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Crown Cloud Adjustment");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for editing point cloud representing a single tree crown. For editing please press the \'x\' key and draw a selection box by dragging a mouse."
                        " Selected points will be deleted from the cloud representing the tree crown.");
  in->set_inputCloud1("Input Tree Cloud:",get_treeNames());
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

   if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    QString treename = in->get_inputCloud1();
    //zkopirovat do m_cloud
    m_cloud->get_Cloud()->clear();
    QString name = QString("%1.edit").arg(in->get_inputCloud1());
    m_cloud->set_name(name);
    m_cloud->set_Cloud(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
    m_cloud->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //zkopirovat do m_cloud1
    m_cloud1->get_Cloud()->clear();
    m_cloud1->set_name(in->get_outputCloud1());
    m_cloud1->set_Psize(Proj->get_Cloud(in->get_inputCloud1()).get_Psize());

    //spustit editacni listu
    addToolBarBreak ();
    editBar = new QToolBar(tr("edit bar"),this);
    editBar->setIconSize(QSize(16,16));
    this->addToolBar(editBar);
    QAction *stopTE = editBar->addAction(QPixmap(":/images/stopEdit.png"),"Stop EDIT");
    connect(stopTE,SIGNAL(triggered()),this,SLOT(CrownManualStop()));
    QAction *undoAct = editBar->addAction(QPixmap(":/images/undo.png"),"undo");
    connect(undoAct,SIGNAL(triggered()),this,SLOT(undo()) );
    undopoint.clear();

    treeWidget->allItemOFF();
    m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
    area = m_vis->registerAreaPickingCallback (&MainWindow::AreaEvent, *this );
    m_vis->resetCamera();
    qvtkwidget->update();
  }
  crownDisplayHideT->setEnabled(true);
  crownHeightsDisplyHideT->setEnabled(true);
  crownPositionDisplayHideT->setEnabled(true);
  setVolumeByVoxAct->setEnabled(true);
  exportAttributesAct->setEnabled(true);
  convexHull3DAct->setEnabled(true);
  setCrownSectionsAct->setEnabled(true);
  crownExternalPtsT->setEnabled(true);
  delete in;
}
void MainWindow::CrownManualStop()
{
  // save m_cloud as new tree crown
  QStringList name = m_cloud->get_name().split(".edit");
  if(m_cloud1->get_Cloud()->points.size() > 1)
  {
    QStringList name2 = name.at(0).split(".");
    Proj->get_TreeCloud(name.at(0)).set_TreeCrownManual(CloudOperations::getCloudCopy(m_cloud->get_Cloud()),CloudOperations::getCloudCopy(m_cloud1->get_Cloud()));
    treeWidget->itemON(name.at(0));
    refreshAttTable();
  }
  removeCloud(m_cloud1->get_name());
  removeCloud(m_cloud->get_name());
  crownDisplay(name.at(0));
  m_cloud->get_Cloud()->clear();
  m_cloud1->get_Cloud()->clear();
  delete editBar;
  area.disconnect();

  disconnect(crownDisplayHideT,SIGNAL(triggered()),this,SLOT(crown_DisplayAll()));
  connect(crownDisplayHideT,SIGNAL(triggered()),this,SLOT(crown_HideAll()));
}
void MainWindow::set_CrownAutomatic()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Tree Crown Selection");
  in->set_path(Proj->get_Path());
  in->set_description("This tool serves for automatic selection of the tree cloud subset representing the tree crown.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =in->get_inputList();

   // createPBar();
   // int v=0;
   // int percent= 100/ selected.size();
    #pragma omp parallel for
    for(int i = 0; i < selected.size(); i++ )
    {
      std::cout << selected.at(i).toStdString() << "\n";
      Proj->get_TreeCloud(selected.at(i)).set_TreeCrownAutomatic();
      //
    }
    std::cout<< "hotovo omp\n";
    //showPBarValue(v+= percent);
  }
  //showPBarValue(percent);
  //removePbar();
  std::cout<<"hotovo smycka\n";
  delete in;



  crownDisplayHideT->setEnabled(true);
  crownHeightsDisplyHideT->setEnabled(true);
  crownPositionDisplayHideT->setEnabled(true);
  setVolumeByVoxAct->setEnabled(true);
  exportAttributesAct->setEnabled(true);
  convexHull3DAct->setEnabled(true);
  setCrownSectionsAct->setEnabled(true);
  crownExternalPtsT->setEnabled(true);
  //refreshAttTable();
}
void MainWindow::crownDisplay(QString name)
{
  QColor col = Proj->get_TreeCloud(name).get_color();
  dispCloud(Proj->get_TreeCloud(name).get_TreeCrown().getCrownCloud(),col.red(),col.green(), col.blue());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, Proj->get_TreeCloud(name).get_TreeCrown().get_name().toUtf8().constData());
  disconnect(crownDisplayHideT,SIGNAL(triggered()),this,SLOT(crown_DisplayAll()));
  connect(crownDisplayHideT,SIGNAL(triggered()),this,SLOT(crown_HideAll()));
  crownDisplayHideT->setIcon(QPixmap(":/images/crownBar/crown_sel.png"));
}
void MainWindow::crown_DisplayAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
       if(Proj->get_TreeCloud(i).isCrownExist() == true)
        {
            QString name = Proj->get_TreeCloud(i).get_name();
            crownDisplay(name);
        }
    }
}
void MainWindow::crown_HideAll()
{
   for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist() == true)
        {
            QString name = Proj->get_TreeCloud(i).get_TreeCrown().get_name();
            removeCloud(name);
        }
    }
  disconnect(crownDisplayHideT,SIGNAL(triggered()),this,SLOT(crown_HideAll()));
  connect(crownDisplayHideT,SIGNAL(triggered()),this,SLOT(crown_DisplayAll()));
  crownDisplayHideT->setIcon(QPixmap(":/images/crownBar/crown.png"));
  qvtkwidget->update();
}
void MainWindow::CrownHeightDisplay(QString name)
{
  std::stringstream Hname ;
  Hname << name.toUtf8().constData() << "_heightlineCrown";
  pcl::PointXYZI minp,maxp;
  maxp = Proj->get_TreeCloud(name).get_TreeCrown().getCrownPosition();
  maxp.z = Proj->get_TreeCloud(name).get_pose().z + Proj->get_TreeCloud(name).get_height();

  minp = maxp;
  minp.z = Proj->get_TreeCloud(name).get_pose().z + Proj->get_TreeCloud(name).get_TreeCrown().getCrownBottomHeight();
  m_vis->addLine(minp,maxp,Hname.str());

  std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_heighttextCrown";
  std::stringstream name3 ;
    name3 << name.toUtf8().constData() << "_bottomheighttextCrown";

  QString ch = QString("%1 m\n%2 m").arg(Proj->get_TreeCloud(name).get_TreeCrown().getCrownHeight()).arg(Proj->get_TreeCloud(name).get_TreeCrown().getCrownTotalHeight());
  m_vis->addText3D(ch.toUtf8().constData(),maxp,0.6,0.6,0.5,0,name2.str());

  QString bh = QString("%1 m").arg(Proj->get_TreeCloud(name).get_TreeCrown().getCrownBottomHeight());
  m_vis->addText3D(bh.toUtf8().constData(),minp,0.6,0.6,0.5,0,name3.str());

}
void MainWindow::crownHeightsDisplayAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist() == true)
        {
            QString name = Proj->get_TreeCloud(i).get_name();
            CrownHeightDisplay(name);
        }
    }
    disconnect(crownHeightsDisplyHideT,SIGNAL(triggered()),this,SLOT(crownHeightsDisplayAll()));
    connect(crownHeightsDisplyHideT,SIGNAL(triggered()),this,SLOT(crownHeightsHideAll()));
    crownHeightsDisplyHideT->setIcon(QPixmap(":/images/crownBar/heightCrown_sel.png"));
    qvtkwidget->update();
}
void MainWindow::crownHeightsHideAll()
{
  QStringList names;
  names << get_treeNames();
  for(int i = 0; i < names.size(); i++)
  {
    //remove line
    std::stringstream Cname ;
    Cname << names.at(i).toUtf8().constData() << "_heightlineCrown";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << names.at(i).toUtf8().constData() << "_heighttextCrown";
    m_vis->removeText3D (Tname.str());
    //remove text 2
    std::stringstream Rname ;
    Rname << names.at(i).toUtf8().constData() << "_bottomheighttextCrown";
    m_vis->removeText3D (Rname.str());

  }
  disconnect(crownHeightsDisplyHideT,SIGNAL(triggered()),this,SLOT(crownHeightsHideAll()));
  connect(crownHeightsDisplyHideT,SIGNAL(triggered()),this,SLOT(crownHeightsDisplayAll()));
  crownHeightsDisplyHideT->setIcon(QPixmap(":/images/crownBar/heightCrown.png"));
  qvtkwidget->update();
}
void MainWindow::crownPositionDisplayAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist() == true)
        {
        QString name = Proj->get_TreeCloud(i).get_name();
        crownPositionDisplay(name);
        }
    }
}
void MainWindow::crownPositionDisplay(QString name)
{
    std::stringstream Sname;
    Sname << name.toUtf8().constData() << "_sphereCrown";
    QColor col = Proj->get_TreeCloud(name).get_color();
    pcl::PointXYZI point = Proj->get_TreeCloud(name).get_TreeCrown().getCrownPosition();
    m_vis->addSphere(point,0.2,0.95,0.32,0.26,Sname.str());

    std::stringstream Pname;
    Pname << name.toUtf8().constData() << "_sphereCrownAtTreePos";
    pcl::PointXYZI p = Proj->get_TreeCloud(name).get_pose();
    p.x = point.x;
    p.y = point.y;
    m_vis->addSphere(p,0.2,col.redF(),col.greenF(),col.blueF(),Pname.str());

    std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_distTextCrown";
    float a = Proj->get_TreeCloud(name).get_TreeCrown().getAzimuth();
    float d = Proj->get_TreeCloud(name).get_TreeCrown().getPosDist();
    QString dist = QString("%1 m\n%2 deg\n").arg(d).arg(a);
    m_vis->addText3D(dist.toUtf8().constData(),p,0.6,0.6,0.5,0,name2.str());

    disconnect(crownPositionDisplayHideT,SIGNAL(triggered()),this,SLOT(crownPositionDisplayAll()));
    connect(crownPositionDisplayHideT,SIGNAL(triggered()),this,SLOT(crownPositionHideAll()));
    crownPositionDisplayHideT->setIcon(QPixmap(":/images/crownBar/crownPos_sel.png"));
    qvtkwidget->update();
}
void MainWindow::crownPositionHideAll()
{
  for(int i = 0; i < Proj->get_sizeTreeCV(); i++)
  {
      QString name = Proj->get_TreeCloud(i).get_name();
    //remove sphere
    std::stringstream Cname ;
    Cname << name.toUtf8().constData() << "_sphereCrown";
    m_vis->removeShape (Cname.str());

    //remove text
    std::stringstream Tname ;
    Tname << name.toUtf8().constData() << "_distTextCrown";
    m_vis->removeText3D (Tname.str());

    std::stringstream Pname;
    Pname << name.toUtf8().constData() << "_sphereCrownAtTreePos";
    m_vis->removeShape (Pname.str());
  }
  disconnect(crownPositionDisplayHideT,SIGNAL(triggered()),this,SLOT(crownPositionHideAll()));
  connect(crownPositionDisplayHideT,SIGNAL(triggered()),this,SLOT(crownPositionDisplayAll()));
  crownPositionDisplayHideT->setIcon(QPixmap(":/images/crownBar/crownPos.png"));
  qvtkwidget->update();
}
void MainWindow::setSectionsVolumeSurfacePosition()
{
     QStringList names;
  names <<"All_trees";
  names << get_treeNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("Crown Volume and Surface by Concave Polyhedron");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for computing and visualization of crown volume, surface, and space occupation using horizontal sections of the crown"
                    "The cloud representing the tree crown is divided into horizontal sections. These sections are bounded by concave hulls used for volume calculation and crown surface triangulation. "
                     "Section height and concaveness (maximal length of the edge in the hull) of the section concave hull can be set by the user.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  //in->set_inputCloud1("Input Tree Cloud:",names);
  in->set_inputInt("Set Section Height in [cm]:","100");
  in->set_inputInt2("Set Initial Threshold Distance for Section Concave Hull in [cm]:","100");

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected = in->get_inputList();

    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    //#pragma omp parallel for
    for(int i = 0; i < selected.size(); i++ )
    {
      std::cout << selected.at(i).toStdString() << "\n";
      if( Proj->get_TreeCloud(selected.at(i)).isCrownExist()== true)
      {
        Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().setSectionHeight((float)in->get_intValue()/100);
        Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().setThresholdDistanceForSectionsHull((float)in->get_intValue2()/100);
        Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().computeSectionsAttributes(Proj->get_TreeCloud(selected.at(i)).get_pose());
        crownSurfaceBySectionsDisplayName(selected.at(i));
      }
      else
      {
        QString a = QString("For tree -- %1 -- cannot be computed volume, since crown is not set.\n Please set position, height and crown for this tree.").arg(selected.at(i));

        QMessageBox::warning(0 ,("Warning"), a);
      }
      showPBarValue(v+= percent);
    }
    removePbar();
    //QMessageBox::information(0,(""),("spocitano")));
    crownSurfaceBySectionsT->setEnabled(true);
    refreshAttTable();
  }

  delete in;
}
void MainWindow::crownSurfaceBySectionsHideAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist() == true)
        {
            std::stringstream Pname;
            Pname <<Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_surfaceMesh";
            m_vis->removePolygonMesh(Pname.str());

            std::stringstream Rname ;
            Rname << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_meshTextCrownSections";
            m_vis->removeText3D (Rname.str());
        }
    }
    disconnect(crownSurfaceBySectionsT,SIGNAL(triggered()),this,SLOT(crownSurfaceBySectionsHideAll()));
    connect(crownSurfaceBySectionsT,SIGNAL(triggered()),this,SLOT(crownSurfaceBySectionsDisplayAll()));
    crownSurfaceBySectionsT->setIcon(QPixmap(":/images/crownBar/crownSections.png"));
    qvtkwidget->update();
}
void MainWindow::crownSurfaceBySectionsDisplayAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist() == true && Proj->get_TreeCloud(i).get_TreeCrown().isSectionsPolyhedronExist() == true)
        {
            crownSurfaceBySectionsDisplayName(Proj->get_TreeCloud(i).get_name());
        }
    }
}
void MainWindow::crownSurfaceBySectionsDisplayName(QString name)
{
    QColor col = Proj->get_TreeCloud(name).get_color();
    PolyhedronFromSections p = Proj->get_TreeCloud(name).get_TreeCrown().getPolyhedronFromSections();
    std::stringstream Pname;
    Pname << name.toUtf8().constData() << "_surfaceMesh";
    pcl::PolygonMesh *mesh = new pcl::PolygonMesh(p.getMesh());
    m_vis->addPolygonMesh(*mesh,Pname.str());
    m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, Pname.str());
    m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, col.redF(), col.greenF(), col.blueF(),Pname.str());

    std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_meshTextCrownSections";
    float vol = Proj->get_TreeCloud(name).get_TreeCrown().getVolumeSections();
    float surf = Proj->get_TreeCloud(name).get_TreeCrown().getPolyhedronFromSections().getSurfaceArea();
    QString text = QString("%1 m3\n%2 sq. m\n").arg(vol).arg(surf);
    pcl::PointXYZI pos = Proj->get_TreeCloud(name).get_TreeCrown().getCrownPosition();
    pos.z = Proj->get_TreeCloud(name).get_pose().z + Proj->get_TreeCloud(name).get_height()+0.2;
    m_vis->addText3D(text.toUtf8().constData(),pos,0.5,0.5,0.4,0,name2.str());

    disconnect(crownSurfaceBySectionsT,SIGNAL(triggered()),this,SLOT(crownSurfaceBySectionsDisplayAll()));
    connect(crownSurfaceBySectionsT,SIGNAL(triggered()),this,SLOT(crownSurfaceBySectionsHideAll()));
    crownSurfaceBySectionsT->setIcon(QPixmap(":/images/crownBar/crownSections_sel.png"));
    qvtkwidget->update();
}
void MainWindow::create3DConvexull()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Volume and Surface by the 3D Convex Hull ");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for computing a 3D convex hull of the crown. "
                      "Implicitly, all points of the crown cloud of the two lowest and the two highest one meter high sections are used in calculations. "
                      "Concerning the other sections, the border points only (computed by 2D hulls) are used to accelerate the computations.");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_inputCheckBox("Use all ponts from crown cloud? (high computational time demands)");

  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    QStringList selected;
    selected = in->get_inputList();

    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    //#pragma omp parallel for
    for(int i = 0; i < selected.size(); i++ )
    {
      std::cout << selected.at(i).toStdString() << "\n";
      if( Proj->get_TreeCloud(selected.at(i)).isCrownExist()== true)
        Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().computeConvexhull3D(in->get_CheckBox());
      showPBarValue(v+= percent);
    }
    removePbar();
    crownSurface3DHullDisplayAll();
    crownSurfaceBy3DHullT->setEnabled(true);
    intersectionAct->setEnabled(true);
  refreshAttTable();
  }
  delete in;
}
void MainWindow::crownSurface3DHullHideAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist() == true)
        {
            std::stringstream Pname;
            Pname <<Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_surfaceMesh3DC";
            m_vis->removePolygonMesh(Pname.str());

            std::stringstream Rname ;
            Rname << Proj->get_TreeCloud(i).get_name().toUtf8().constData() << "_meshTextCrownSections3DC";
            m_vis->removeText3D (Rname.str());
        }
    }
    disconnect(crownSurfaceBy3DHullT,SIGNAL(triggered()),this,SLOT(crownSurface3DhullHideAll()));
    connect(crownSurfaceBy3DHullT,SIGNAL(triggered()),this,SLOT(crownSurface3DHullDisplayAll()));
    crownSurfaceBy3DHullT->setIcon(QPixmap(":/images/crownBar/crownConvex.png"));
    qvtkwidget->update();
}
void MainWindow::crownSurface3DHullDisplayAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist() == true && Proj->get_TreeCloud(i).get_TreeCrown().isConvexhull3DExist() == true)
        {
            crownSurface3DHullDisplayName(Proj->get_TreeCloud(i).get_name());
        }
    }
}
void MainWindow::crownSurface3DHullDisplayName(QString name)
{
    QColor col = Proj->get_TreeCloud(name).get_color();
    std::stringstream Pname;
    Pname << name.toUtf8().constData() << "_surfaceMesh3DC";
    pcl::PolygonMesh *mesh = new pcl::PolygonMesh(Proj->get_TreeCloud(name).get_TreeCrown().get3DConvexhull().getMesh());
    m_vis->addPolygonMesh(*mesh,Pname.str());
    m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, Pname.str());
    m_vis->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_REPRESENTATION , pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME , Pname.str() );
    m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, col.redF(), col.greenF(), col.blueF(),Pname.str());

    std::stringstream name2 ;
    name2 << name.toUtf8().constData() << "_meshTextCrownSections3DC";
    float vol = Proj->get_TreeCloud(name).get_TreeCrown().get3DConvexhull().getVolume();
    float surf = Proj->get_TreeCloud(name).get_TreeCrown().get3DConvexhull().getSurfaceArea();
    QString text = QString("%1 m3\n%2 sq. m\n").arg(vol).arg(surf);
    pcl::PointXYZI pos = Proj->get_TreeCloud(name).get_TreeCrown().getCrownPosition();
    pos.z = Proj->get_TreeCloud(name).get_pose().z + Proj->get_TreeCloud(name).get_height()+0.2;
    m_vis->addText3D(text.toUtf8().constData(),pos,0.4,0.4,0.3,0,name2.str());

    disconnect(crownSurfaceBy3DHullT,SIGNAL(triggered()),this,SLOT(crownSurface3DHullDisplayAll()));
    connect(crownSurfaceBy3DHullT,SIGNAL(triggered()),this,SLOT(crownSurface3DHullHideAll()));
    crownSurfaceBy3DHullT->setIcon(QPixmap(":/images/crownBar/crownConvex_sel.png"));
    qvtkwidget->update();
}
void MainWindow::crownExternalPtsDisplayAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
       if(Proj->get_TreeCloud(i).isCrownExist() == true)
        {
            QString name = Proj->get_TreeCloud(i).get_name();
            QColor col = Proj->get_TreeCloud(name).get_color();
            QString epName = QString("%1_externalPts").arg(name);
            Cloud *ep = new Cloud(Proj->get_TreeCloud(name).get_TreeCrown().getExternalPoints(),epName,col);
            ep->set_Psize(3);
            dispCloud(*ep);
        }
    }
    crownExternalPtsT->setIcon(QPixmap(":/images/crownBar/crownEP_sel.png"));
    disconnect(crownExternalPtsT,SIGNAL(triggered()),this,SLOT(crownExternalPtsDisplayAll()));
    connect(crownExternalPtsT,SIGNAL(triggered()),this,SLOT(crownExternalPtsHideAll()));
}
void MainWindow::crownExternalPtsHideAll()
{
    for (int i =0; i<Proj->get_sizeTreeCV(); i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist() == true)
        {
            QString name = QString("%1_externalPts").arg(Proj->get_TreeCloud(i).get_name());
            removeCloud(name);
        }
    }
  disconnect(crownExternalPtsT,SIGNAL(triggered()),this,SLOT(crownExternalPtsHideAll()));
  connect(crownExternalPtsT,SIGNAL(triggered()),this,SLOT(crownExternalPtsDisplayAll()));
  crownExternalPtsT->setIcon(QPixmap(":/images/crownBar/crownEP.png"));
  qvtkwidget->update();
}
void MainWindow::computeCrownsIntersections()
{

    QMessageBox *msgBox =  new QMessageBox(this);
	msgBox->setText("Crown Intersections.");
	msgBox->setInformativeText("Intersections are computed as Boolean AND in the 3D space for intersecting pairs of crowns."
                              "The intersections are based on the 3D convex hulls of the two crowns.");
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox->setDefaultButton(QMessageBox::Yes);

    if(msgBox->exec() == QMessageBox::Yes)
    {
        Proj->computeCrownIntersections();
        crownSurface3DHullHideAll();
        intersectionsShowAll();
        crownIntersectionTableT->setEnabled(true);
        crownIntersectionsT->setEnabled(true);
        exportIntersectionAct->setEnabled(true);
    }
}
void MainWindow::intersectionsShowAll()
{
    for(int i = 0; i<Proj->getIntersectionsSize();i++)
    {
        QString name1 = Proj->getIntersectionsAt(i).getName1();
        QString name2 = Proj->getIntersectionsAt(i).getName2();
        std::stringstream Iname;
        Iname << name1.toUtf8().constData()<< name2.toUtf8().constData()<<"_intersectionMesh";
        pcl::PolygonMesh *mesh = new pcl::PolygonMesh(Proj->getIntersectionsAt(i).getMesh());
        m_vis->addPolygonMesh(*mesh,Iname.str());

        m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 255, 255, 0,Iname.str());
        m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY,1, Iname.str() );

        std::stringstream cName1;
        cName1 << name1.toUtf8().constData()<< "_intersectionCrown1";
        pcl::PolygonMesh *mesh1 = new pcl::PolygonMesh(Proj->get_TreeCloud(name1).get_TreeCrown().get3DConvexhull().getMesh());
        m_vis->addPolygonMesh(*mesh1,cName1.str());
       // QColor col = Proj->get_TreeCloud(name1).get_color();
        m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, cName1.str() );

        std::stringstream cName2;
        cName2 << name2.toUtf8().constData()<< "_intersectionCrown2";
        pcl::PolygonMesh *mesh2 = new pcl::PolygonMesh(Proj->get_TreeCloud(name2).get_TreeCrown().get3DConvexhull().getMesh());
        m_vis->addPolygonMesh(*mesh2,cName2.str());
      //  QColor col2 = Proj->get_TreeCloud(name2).get_color();
        m_vis->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, cName2.str() );
    }
    disconnect(crownIntersectionsT,SIGNAL(triggered()),this,SLOT(intersectionsShowAll()));
    connect(crownIntersectionsT,SIGNAL(triggered()),this,SLOT(intersectionsHideAll()));
    crownIntersectionsT->setIcon(QPixmap(":/images/crownBar/crownIntersection_sel.png"));
    qvtkwidget->update();
}
void MainWindow::intersectionsHideAll()
{
    for (int i =0; i<Proj->getIntersectionsSize(); i++)
    {
        QString name1 = Proj->getIntersectionsAt(i).getName1();
        QString name2 = Proj->getIntersectionsAt(i).getName2();
        std::stringstream Iname;
        Iname << name1.toUtf8().constData()<< name2.toUtf8().constData()<< "_intersectionMesh";
        m_vis->removePolygonMesh(Iname.str());

        std::stringstream cName1;
        cName1 << name1.toUtf8().constData()<< "_intersectionCrown1";
        m_vis->removePolygonMesh(cName1.str());

        std::stringstream cName2;
        cName2 << name2.toUtf8().constData()<< "_intersectionCrown2";
        m_vis->removePolygonMesh(cName2.str());
    }
    disconnect(crownIntersectionsT,SIGNAL(triggered()),this,SLOT(intersectionsHideAll()));
    connect(crownIntersectionsT,SIGNAL(triggered()),this,SLOT(intersectionsShowAll()));
    crownIntersectionsT->setIcon(QPixmap(":/images/crownBar/crownIntersection.png"));

    qvtkwidget->update();
}
void MainWindow::showCrownIntersectionsTable()
{
  createITable();
  if(visIT == false)
  {
    visIT=true;
    DockITableWidget->show();
  }
  else
  {
    DockITableWidget->hide();
    visIT=false;
  }
}
//QQSM
void MainWindow::exportQSM()
{
    // dialog pro novy soubor - novy adresar
    // nazev predpony souboru
    //vyber stromu
    // separator

    ExportQSMDialog *d = new ExportQSMDialog(this);
    d->setDescription("\tThis tool serves for exporting tree parameters into formatted text files. "
                              "The name, the location, and the separator within the exported .txt file have to be specified. "
                              "The tool exports currently computed values only.");
    d->setList(get_treeNames());
    d->setPrefix("set prefix of the created file: ", "qsm_");

    int dl = d->exec();
    if(dl == QDialog::Accepted && d->getInputList().size() > 0)
    {
        std::cout<<"parametry: \n";
        std::cout<< "cesta \t\t" << d->getPath().toStdString()<<"\n";
        std::cout<< "prefix \t\t" << d->getPrefix().toStdString()<<"\n";
        std::cout<< "separator \t\t" << d->getSeparator().toStdString()<<"\n";
        std::cout<< "seznam \t\t" << d->getInputList().at(0).toStdString()<<"\n";
        std::cout<< "trees \t\t" << d->getTrees()<<"\n";
        std::cout<< "assortment \t\t" << d->getSortiments()<<"\n";
        std::cout<< "branches \t\t" << d->getBranches()<<"\n";
        std::cout<< "profile \t\t" << d->getProfile()<<"\n";


        //sloÅ¾enÃ­ souboru stromy: strom jmeno, DBH, vyÅ¡ka, vyÅ¡ka paÅezu, celkovy objem, objem hroubi,
        if(d->getTrees() == true) // create tree file
        {
            //open file
            QString filename =QString("%1%2%3trees.txt").arg(d->getPath()).arg(QDir::separator()).arg( d->getPrefix());
            QFile file (filename);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out.setRealNumberNotation(QTextStream::FixedNotation);
            out.setRealNumberPrecision(3);
            // write header

            out << "ID" << d->getSeparator() <<"Cloud_name" << d->getSeparator() << "DBH_[cm]" << d->getSeparator() << "Height_[m]" << d->getSeparator() << "H_base_[m]" << d->getSeparator() << "Total_volume_[m^3]" << d->getSeparator() << "Volume_Hroubi_[m^3]\n";

            // write data
            for(int q=0; q < d->getInputList().size();q++)
            {
                int id =q+1;
                float dbh = Proj->get_TreeCloud(d->getInputList().at(q)).getQSMDBH()/10;
                float height =Proj->get_TreeCloud(d->getInputList().at(q)).get_height();
                float totalV = Proj->get_TreeCloud(d->getInputList().at(q)).getQSMVolume();
                float hroubi = Proj->get_TreeCloud(d->getInputList().at(q)).getQSMVolumeHroubi();

                out<< id << d->getSeparator() << d->getInputList().at(q) << d->getSeparator() <<  dbh << d->getSeparator() << height << d->getSeparator() << height/100 << d->getSeparator() << totalV << d->getSeparator() << hroubi << "\n";
            }
            file.close();
        }
        // slozeni souboru kmeny:  pro kazdy strom: jmeno, poradove Äislo, vypsat rad vetve, objem
        if(d->getBranches() == true)
        {
            //open file
            QString filename =QString("%1%2%3branches.txt").arg(d->getPath()).arg(QDir::separator()).arg( d->getPrefix());
            QFile file (filename);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out.setRealNumberNotation(QTextStream::FixedNotation);
            out.setRealNumberPrecision(3);
            // write header
            out << "Cloud_name" << d->getSeparator() << "ID" << d->getSeparator() << "Order" << d->getSeparator() << "Length" << d->getSeparator() << "Length_Hroubi" << d->getSeparator()<< "Total_volume_[m^3]" << d->getSeparator() << "Volume_Hroubi_[m^3]\n";

            for(int q=0; q < d->getInputList().size();q++)
            {
                // zistak branches
                // pro kazdou branches zistak informace
                std::vector<std::shared_ptr<Segment> > branches = Proj->get_TreeCloud(d->getInputList().at(q)).getBranches();
                int id = 1;
                for(int w=0;w < branches.size(); w++)
                {
                    int order = branches.at(w)->getOrder();
                    float length = branches.at(w)->getLenght();
                    float delkaHroubi = branches.at(w)->getHroubiLength();
                    float Tvolume = branches.at(w)->getTotalVolume();
                    float Hvolume = branches.at(w)->getHroubiVolume();
                    if(Hvolume < 0.001 || delkaHroubi < 1)
                        continue;
                    out << d->getInputList().at(q) << d->getSeparator() << id << d->getSeparator() << order << d->getSeparator()<< length << d->getSeparator() << delkaHroubi << d->getSeparator()<< Tvolume << d->getSeparator() << Hvolume<< "\n";
                    id++;
                }
            }
            file.close();
        }
        // sloÅ¾enÃ­ souboru sekce: pro kazdy strom: jmeno, poradove cislo sekce, vyÅ¡ka sekce, DBH [mm]
        if(d->getProfile() == true)
        {
            //open file
            QString filename =QString("%1%2%3stemProfile.txt").arg(d->getPath()).arg(QDir::separator()).arg( d->getPrefix());
            QFile file (filename);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out.setRealNumberNotation(QTextStream::FixedNotation);
            out.setRealNumberPrecision(3);
            // write header
            out << "Cloud_name" << d->getSeparator() << "ID" << d->getSeparator() << "height_[m]" << d->getSeparator()  << "DBH_[mm]\n";
            for(int q=0; q < d->getInputList().size();q++)
            {
                std::vector<std::shared_ptr<Segment> > branches = Proj->get_TreeCloud(d->getInputList().at(q)).getBranches();
                for(int w=0;w < branches.size(); w++)
                {
                    if(branches.at(w)->getOrder() == 0)
                    {
                        float vyska=0;
                        float krok=0.5;
                        float maxvyska = Proj->get_TreeCloud(d->getInputList().at(q)).get_height();
                        // get cylinders
                        std::vector<pcl::ModelCoefficientsPtr> cyl= branches.at(w)->getCylinders();
                        int id=0;
                        for(vyska;vyska < maxvyska; vyska +=krok )
                        {
                            id++;
                            if(vyska == 1)
                                krok = 1;
                            float vzdal=10000000000;
                            float dbh;
                            for(int r=0; r < cyl.size(); r++)
                            {
                                float height = cyl.at(r)->values.at(2) - cyl.at(0)->values.at(2);
                                if(std::abs(height-vyska) < vzdal)
                                {
                                    vzdal =std::abs(height-vyska);
                                    dbh = cyl.at(r)->values.at(6) * 2000;
                                }
                            }
                            out << d->getInputList().at(q) << d->getSeparator() <<id <<d->getSeparator()<<vyska << d->getSeparator() << dbh <<"\n";
                        }
                    }
                }
            }
            file.close();
        }
        // sloÅ¾enÃ­ souboru vyrezy: pro kazdy strom: jmeno, cislo vyrezu, celkova delka, celo, cep, objem,
        if(d->getSortiments() == true)
        {
            //open file
            QString filename =QString("%1%2%3sortiments.txt").arg(d->getPath()).arg(QDir::separator()).arg( d->getPrefix());
            QFile file (filename);
            file.open(QIODevice::WriteOnly | QIODevice::Text);
            QTextStream out(&file);
            out.setRealNumberNotation(QTextStream::FixedNotation);
            out.setRealNumberPrecision(3);
            // write header
            out << "Cloud_name" << d->getSeparator() << "ID" << d->getSeparator() << "total_height" << d->getSeparator()  << "lowerDBH_[mm]"<< d->getSeparator()  <<"higherDBH_[mm]" << d->getSeparator() << "volume_[m^3]" << d->getSeparator() <<"grade"<< d->getSeparator()<< "\n";

            for(int q=0; q < d->getInputList().size();q++)
            {
                std::vector< std::shared_ptr <Sortiment>> sortimenty =  Proj->get_TreeCloud(d->getInputList().at(q)).getSortiments();
                int id=1;
                for(int w=0; w < sortimenty.size();w++)
                {
                    float volume = sortimenty.at(w)->getVolume();
                    float height = sortimenty.at(w)->getLength();
                    float lowerDBH = sortimenty.at(w)->getSmallerEndDBH()*2000;
                    float biggerDBH = sortimenty.at(w)->getBiggerEndDBH()*2000;
                    int grade = sortimenty.at(w)->getGrade();
                    if(grade ==6)
                        continue;

                    out << d->getInputList().at(q) << d->getSeparator() << id << d->getSeparator() << height << d->getSeparator() << lowerDBH << d->getSeparator() << biggerDBH << d->getSeparator() << volume << d->getSeparator() << grade << "\n";
                    id++;
                }
            }
            file.close();
        }
    }
    delete d;
}


void MainWindow::createITable()
{
  int rows = Proj->getIntersectionsSize();
  m_intersectionTable->setModel(getIntersectionModel());
  for(int i = 0; i < rows; i++)
  {
    m_intersectionTable->setRowHeight(i,20);
  }
  for(int i=0;i<4;i++)
  {
    m_intersectionTable->setColumnWidth(i,100);
  }
}
QStandardItemModel* MainWindow::getIntersectionModel()
{
  int rows = Proj->getIntersectionsSize();
  QStandardItemModel *model = new QStandardItemModel(rows,4,this);
  QStringList headers;
  headers <<"Name 1"<<"Name 2"<<"Shered space\nvolume cubic. m"<<"Shered space\nsurface sq. m";
  model->setHorizontalHeaderLabels(headers);
  for(int row = 0; row < rows; row++)
  {
    QModelIndex index= model->index(row,0,QModelIndex());
    model->setData(index,Proj->getIntersectionsAt(row).getName1());

    index= model->index(row,1,QModelIndex());
    model->setData(index,Proj->getIntersectionsAt(row).getName2());

    index= model->index(row,2,QModelIndex());
    model->setData(index,Proj->getIntersectionsAt(row).getVolume());

    index= model->index(row,3,QModelIndex());
    model->setData(index,Proj->getIntersectionsAt(row).getSurface());
  }
  return model;
}
void MainWindow::exportIntersections()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Intersections Parameters Export");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for exporting the table of intersection attributes into .txt file. "
                      "All currently computed intersection values are exported. The semicolon is used as a separator. ");
  in->set_outputPath("Path to the new file:","c:\\exported_cloud.txt","Text file (*.txt)");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
  //zapisovat jednotlive radky
    QFile file (in->get_outputPath());
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out <<"name1 name2 volume surface\n";
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);
    for(int i=0;i<Proj->getIntersectionsSize();i++)
    {

      out << Proj->getIntersectionsAt(i).getName1() << " ";
      out << Proj->getIntersectionsAt(i).getName2() << " ";
      out << Proj->getIntersectionsAt(i).getVolume() << " ";
      out << Proj->getIntersectionsAt(i).getSurface() << "\n";
    }
    file.close();
   // cloud.reset();
  }

  delete in;
}
void MainWindow::exportCrownAttributes()
{
  ExportCrownAttr *exdialog = new ExportCrownAttr (this);
  exdialog->set_description("\tThis tool serves for exporting crown attributes into formatted text file. "
                            "The tree, the attributes, and the separator within the exported .txt file have to be specified. "
                            "The tool exports currently computed values only. ");
  exdialog->set_list(get_treeNames());
  int dl = exdialog->exec();

  if(dl == QDialog::Accepted && exdialog->get_inputList().size() > 0)
  {
    QStringList selected;
    selected =exdialog->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ selected.size();

    QFile file (exdialog->get_outputFile());
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(3);

    //header
    out << "Cloud_name" ;
    if(exdialog->getPoints() == true)
      out << exdialog->get_separator()<< "Points" ;
    if(exdialog->getHeight() == true)
      out << exdialog->get_separator()<< "Height" ;
    if(exdialog->getBottomHeight() == true)
      out << exdialog->get_separator()<< "Bottom_height" ;
    if(exdialog->getTotalHeight() == true)
      out << exdialog->get_separator()<< "Total_height" ;
    if(exdialog->getLength() == true)
      out << exdialog->get_separator()<< "Length" ;
    if(exdialog->getWidth() == true)
      out << exdialog->get_separator()<< "Width" ;
    if(exdialog->getPositionDeviance() == true)
      out << exdialog->get_separator()<< "Pos_dev_dist" << exdialog->get_separator()<< "Pos_dev_azimuth";
    if(exdialog->getPositionXYZ() == true)
      out << exdialog->get_separator()<< "X_coord_pos" << exdialog->get_separator()<< "Y_coord_pos" << exdialog->get_separator()<< "Z_coord_pos";
    if(exdialog->getVolVoxels() == true)
      out << exdialog->get_separator()<< "Vol_voxels" ;
    if(exdialog->getVolSections() == true)
      out << exdialog->get_separator()<< "Vol_sections" ;
    if(exdialog->getSurface() == true)
      out << exdialog->get_separator()<< "Surface_area" ;
    if(exdialog->getVol3DCH() == true)
      out << exdialog->get_separator()<< "Vol_3D_CH" ;
    if(exdialog->getSurf3DCH() == true)
      out << exdialog->get_separator()<< "Surface_area_3D_CH" ;
    if(exdialog->getSectionHeight() == true)
      out << exdialog->get_separator()<< "Section_height" ;
    if(exdialog->getThresholdDistance() == true)
      out << exdialog->get_separator()<< "Threshlod_distance" ;
    out <<"\n";

    for(int i = 0; i < selected.size() ; i++ )
    {

      if(Proj->get_TreeCloud(selected.at(i)).isCrownExist()==true)
      {
        out << selected.at(i);
        if(exdialog->getPoints() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().get_Cloud()->points.size();
        if(exdialog->getHeight() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getCrownHeight();
        if(exdialog->getBottomHeight() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getCrownBottomHeight();
        if(exdialog->getTotalHeight() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getCrownTotalHeight();
        if(exdialog->getLength() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getCrownLenghtXY();
        if(exdialog->getWidth() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getCrownWidthXY();
        if(exdialog->getPositionDeviance() == true)
        {
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getPosDist();
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getAzimuth();
        }
        if(exdialog->getPositionXYZ() == true)
        {
          pcl::PointXYZI p = Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getCrownPosition();
          out << exdialog->get_separator() << p.x << exdialog->get_separator() << p.y << exdialog->get_separator() << p.z;
        }
        if(exdialog->getVolVoxels() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getVolumeVoxels();
        if(exdialog->getVolSections() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getVolumeSections();

        if(exdialog->getSurface() == true && Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().isSectionsPolyhedronExist()==true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getPolyhedronFromSections().getSurfaceArea();
        else
          out << exdialog->get_separator() <<"0";

        if(exdialog->getVol3DCH() == true && Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().isConvexhull3DExist()==true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().get3DConvexhull().getVolume();
        else
          out << exdialog->get_separator() <<"0";

        if(exdialog->getSurf3DCH() == true && Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().isConvexhull3DExist()==true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().get3DConvexhull().getSurfaceArea();
        else
          out << exdialog->get_separator() <<"0";

        if(exdialog->getSectionHeight() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getSectionHeight();
        if(exdialog->getThresholdDistance() == true)
          out << exdialog->get_separator() << Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().getThresholdDistanceForsectionsHull();
          out <<"\n";
      }

      showPBarValue(v+= percent);
    }
    removePbar();
    file.close();
  }
  delete exdialog;
}
void MainWindow::recomputeAfterTreePosChenge()
{
    for(int i=0;i<Proj->get_sizeTreeCV();i++)
    {
        if(Proj->get_TreeCloud(i).isCrownExist()==true)
        {
            Proj->get_TreeCloud(i).get_TreeCrown().recomputeHeightsAndPositionDev(Proj->get_TreeCloud(i).get_pose());
            crownPositionHideAll();
            crownHeightsHideAll();
        }
    }
}
void MainWindow::crownVolumeByVoxels()
{

  InputDialog *in = new InputDialog(this);
  in->set_title("Crown Volume by Voxels.");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool serves for computing crown volume using voxels of a given resolution.\n"
                      " Voxels are not visualized, crown volumes are recorded in the project attribute table");
  in->set_inputList("Input Tree Cloud:",get_treeNames());
  in->set_inputInt("Set Voxel Size in [cm]:","25");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Accepted && in->get_inputList().size() > 0)
  {
    float resolution = (float)in->get_intValue()/100;
    QStringList selected;
    selected = in->get_inputList();

    createPBar();
    int v=0;
    int percent= 100/ selected.size();
    //#pragma omp parallel for
    for(int i = 0; i < selected.size(); i++ )
    {
      if( Proj->get_TreeCloud(selected.at(i)).isCrownExist()== true)
        Proj->get_TreeCloud(selected.at(i)).get_TreeCrown().computeVolumeByVoxels(resolution);
      showPBarValue(v+= percent);
    }
    removePbar();
    refreshAttTable();
  }
  delete in;
}
//MISCELLANEOUS
void MainWindow::mergeClouds()
{
  // por zatim najit vsechny body v polomeru  41 m od daneho bodu a ulozit je jako tren
  QStringList types;
  types << "Tree" << "Base cloud" << "Terrain cloud" << "Vegetation cloud" << "Other";

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud Merge");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool merges selected clouds into a new one. This tool may be used when one object is to be made of more separated clouds. "
                      "The result is one cloud with a given name and of a given type.\n"
                      "\t Please select desired clouds in the list and set a name and type of the new cloud.");
  in->set_inputList("cloud Selection:",get_allNames());
  in->set_outputCloud1("Output Cloud Name:","cloud");
  in->set_outputType("Type of the Output Cloud:", types);
  in->set_inputCheckBox("Remove selected file?");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputList().isEmpty())
  {
    QString type;
    if(in->get_outputType() == "Base cloud")
      type = "cloud";
    if(in->get_outputType() == "Terrain cloud")
      type = "teren";
    if(in->get_outputType() == "Vegetation cloud")
      type = "vege";
    if(in->get_outputType() == "Tree")
      type = "strom";
    if(in->get_outputType() == "Other")
      type = "ost";


    QList<QString> clouds;
    clouds = in->get_inputList();
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_ (new pcl::PointCloud<pcl::PointXYZI>);
    for(int q=0; q < clouds.size(); q++)
    {
      *merged_ += *Proj->get_Cloud(clouds.at(q)).get_Cloud();
    }
    merged_->width = merged_->points.size ();
    merged_->is_dense=true;
    merged_->height=1;

    Cloud *r = new Cloud();
    r->set_Cloud(merged_);
    r->set_name(in->get_outputCloud1());
    saveCloud(r, type);

    if(in->get_CheckBox()==true)
    {
      for(int w=0; w < clouds.size(); w++)
      {
        treeWidget->itemdelete(clouds.at(w));
        removeCloud(clouds.at(w));
        Proj->delete_Cloud(clouds.at(w));
      }
    }
  }
  delete in;
}
void MainWindow::labelClouds()
{
  QStringList types;
  types << "Base clouds" << "Terrain clouds" << "Vegetation clouds" <<"Trees" << "Others";

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud Label");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool labels all clouds of a given type.");
  in->set_inputCloud1("Set Type of Clouds to Be Labeled:",types);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    {
      delete in;
      return; }

  if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
  {
    QStringList names;
    if(in->get_inputCloud1() == "Base clouds")
      names << get_baseNames();
    if(in->get_inputCloud1() == "Terrain clouds")
      names << get_terrainNames();
    if(in->get_inputCloud1() == "Vegetation clouds")
      names << get_vegetationNames();
    if(in->get_inputCloud1() == "Trees")
      names << get_treeNames();
    if(in->get_inputCloud1() == "Others")
      names << get_ostNames();
#pragma omp parallel for
    for(int p=0; p < names.size(); p++)
    {
      pcl::PointXYZI minp,maxp;
      pcl::getMinMax3D(*Proj->get_Cloud(names.at(p)).get_Cloud(),minp, maxp);

       //addtext3D with R
      pcl::PointXYZ bod;
      bod.x=(minp.x +maxp.x)/2;
      bod.y=(minp.y +maxp.y)/2;
      bod.z=(minp.z +maxp.z)/2;;
      QString h= QString("%1").arg(names.at(p));
      std::stringstream name2 ;
      name2 << names.at(p).toUtf8().constData() << "_label";
      m_vis->removeText3D(name2.str());
      m_vis->addText3D(h.toUtf8().constData(),bod,0.5,0.9,0.1,0,name2.str());
    }
  }
  delete in;
}
void MainWindow::removePoints(){
    // por zatim najit vsechny body v polomeru  41 m od daneho bodu a ulozit je jako tren
    QStringList types;
    types << "Tree" << "Base cloud" << "Terrain cloud" << "Vegetation cloud" << "Other";
    QStringList fields;
    fields << "Intensity" << "X" << "Y" << "Z" ;

    InputDialog *in = new InputDialog(this);
    in->set_title("Remove Points");
    in->set_path(Proj->get_Path());
    in->set_description("\tThis tool enables to select point from cloud with specified intensity values."
                        "The result is one cloud with a given name and of a given type. \n"
                        "\tClouds of interest has to be selected in the list. A name and type of the new cloud has to be specified.");
    //in->set_inputList("cloud Selection:",get_allNames());
    in->set_inputCloud1("Cloud:", get_allNames());
    in->set_outputCloud1("Output Cloud Name:","cloud");
    in->set_outputType("Type of the Output Cloud:", types);
    in->set_outputType2("Type of Field:", fields);
    in->set_inputInt("Smallest Point Input Value", "0");
    in->set_inputInt2("Largest Point Input Value", "5");
    in->set_stretch();
    int dl = in->exec();

    if(dl == QDialog::Rejected)
    { return;}

    if(dl == QDialog::Accepted && !in->get_inputCloud1().isEmpty())
    {
        QString type;
        if(in->get_outputType() == "Base cloud")
            type = "cloud";
        if(in->get_outputType() == "Terrain cloud")
            type = "teren";
        if(in->get_outputType() == "Vegetation cloud")
            type = "vege";
        if(in->get_outputType() == "Tree")
            type = "strom";
        if(in->get_outputType() == "Other")
            type = "ost";
        QString field = in->get_outputType2();
        float value1 =in->get_intValue();
        float value2 =in->get_intValue2();
        
        std::cout<< "Input cloud: "<< in->get_inputCloud1().toStdString()<<"\n";
        std::cout<< "Output cloud: "<< in->get_outputCloud1().toStdString()<<"\n";
        std::cout<< "type: "<< in->get_outputType().toStdString()<<"\n";
        std::cout<< "field: "<< field.toStdString()<<"\n";
        std::cout<< "value 1: "<< in->get_intValue()<<"\n";
        std::cout<< "value 2: "<< in->get_intValue2()<<"\n";

        createPBar();
        float v= 0;
        float percent =100/Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.size() ;
        pcl::PointCloud<pcl::PointXYZI>::Ptr merged_ (new pcl::PointCloud<pcl::PointXYZI>);
        
        if(field =="Z")
        {
            std::cout<< "for Z field \n";
            for(int i=0; i < Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.size();i++){
                if( Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i).z > value1 &&  Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i).z < value2 )
                    merged_->points.push_back(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i));
                showPBarValue(v+= percent);
            }
        }
        else if(field =="X")
        {
            std::cout<< "for Y field \n";
            for(int i=0; i < Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.size();i++){
                if( Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i).x > value1 &&  Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i).x < value2 )
                    merged_->points.push_back(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i));
                showPBarValue(v+= percent);
            }
        }
        else if(field =="Y")
        {
            std::cout<< "for Y field \n";
            for(int i=0; i < Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.size();i++)
            {
                if( Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i).y > value1 &&  Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i).y < value2 )
                    merged_->points.push_back(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i));
                showPBarValue(v+= percent);
            }
        }
        else
        {
            std::cout<< "for intensity field \n";
            for(int i=0; i < Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.size();i++)
            {
                if( Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i).intensity > value1 &&  Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i).intensity < value2 )
                    merged_->points.push_back(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i));
                showPBarValue(v+= percent);
            }
        }
        std::cout<< "Pocet bodu: "<< merged_->points.size()<<"\n";
        Cloud *cV = new Cloud(merged_,in->get_outputCloud1());
        saveCloud(cV, type);
        removePbar();
    }
}
void MainWindow::labelCloudsOFF()
{
  QStringList names;
  names << get_allNames();
  for(int p=0; p < names.size(); p++)
  {
    std::stringstream name2 ;
    name2 << names.at(p).toUtf8().constData() << "_label";
    m_vis->removeText3D(name2.str());
  }
}
void MainWindow::radiusOutlierRemoval()
{
    QStringList types;
    types << "Tree" << "Base cloud" << "Terrain cloud" << "Vegetation cloud" << "Other";

    InputDialog *in = new InputDialog(this);
    in->set_title("Radius Outlier Removal");
    in->set_path(Proj->get_Path());
    in->set_description("\tThis tool enables to remove isolated points from the input cloud. ");
    in->set_inputList("Cloud Selection:",get_allNames());
    //in->set_outputCloud1("New Cloud without Filtered Points:","cloud_Filter");
    in->set_outputCloud1("Set Postfix of Clouds:","_filterROR");
    in->set_inputInt("Radius [cm]","25");
    in->set_inputInt2("Minimum Neighbors in Radius","100");
    in->set_outputType("Output Cloud Type:", types);
    in->set_inputCheckBox("Remove input file?");
    in->set_stretch();
    int dl = in->exec();

    if(dl == QDialog::Rejected)
    { return; }

    if(dl == QDialog::Accepted && !in->get_inputList().isEmpty())
    {
        
        
        QString type;
        if(in->get_outputType() == "Base cloud")
            type = "cloud";
        if(in->get_outputType() == "Terrain cloud")
            type = "teren";
        if(in->get_outputType() == "Vegetation cloud")
            type = "vege";
        if(in->get_outputType() == "Tree")
            type = "strom";
        if(in->get_outputType() == "Other")
            type = "ost";

        float res = in->get_intValue()/100.0;
        QList<QString> clouds;
        clouds = in->get_inputList();
        for(int q=0; q < clouds.size(); q++)
        {
            //std::cout<<"clouds.at(q) "<< clouds.at(q).toStdString()<<"\n";
            QString nameS = clouds.at(q);
            nameS.replace(".pcd","");
            QString name = QString("%1%2").arg(nameS).arg(in->get_outputCloud1());
            RadiusOutlierRemoval *ror = new RadiusOutlierRemoval();
            ror->setBaseCloud(Proj->get_Cloud(clouds.at(q)));
            ror->setOutputName(name);
            ror->setRadius(res);
            ror->setType(type);
            ror->setNeighborhood(in->get_intValue2());
            connect(ror, SIGNAL(sendingoutput(Cloud *, QString)), this, SLOT(saveCloud(Cloud *, QString)));

            ror->execute();
            delete ror;
            if(in->get_CheckBox() == true)
            {
                treeWidget->itemdelete(clouds.at(q));
                removeCloud(clouds.at(q));
                Proj->delete_Cloud(clouds.at(q));
            }
        }
    }
}
void MainWindow::minusCloud()
{
  QStringList types;
  types << "Tree" << "Base cloud" << "Terrain cloud" << "Vegetation cloud" << "Other";
  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud Subtraction");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool enables to compare two clouds and remove identical points from the bigger one. "
                      "The result is saved as a new cloud consisting from unique points of the bigger cloud. "
                      "The tool may be used to reduce duplicate points within point clouds.");
  in->set_inputCloud1("1st Input Cloud:",get_allNames());
  in->set_inputCloud2("2nd Input Cloud:",get_allNames());
  in->set_outputCloud1("Output Cloud Name:","cloud_subtraction");
 //in->set_outputCloud2("Output Cloud Name:","truenegative");
  in->set_outputType("Output Cloud Type:", types);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    QString type;
    if(in->get_outputType() == "Base cloud")
      type = "cloud";
    if(in->get_outputType() == "Terrain cloud")
      type = "teren";
    if(in->get_outputType() == "Vegetation cloud")
      type = "vege";
    if(in->get_outputType() == "Tree")
      type = "strom";
    if(in->get_outputType() == "Other")
      type = "ost";
    //porovnat velikost
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_1 = Proj->get_Cloud(in->get_inputCloud1()).get_Cloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_2 = Proj->get_Cloud(in->get_inputCloud2()).get_Cloud();

    Cloud *c1 = new Cloud();
    Cloud *c2 = new Cloud();


    if(cloud_1->points.size() > cloud_2->points.size())
    {
      c1->set_Cloud(cloud_1);
      c1->set_name(in->get_inputCloud1());
      c2->set_Cloud(cloud_2);
      c2->set_name(in->get_inputCloud2());
    }
    else
    {
      c1->set_Cloud(cloud_2);
      c1->set_name(in->get_inputCloud2());
      c2->set_Cloud(cloud_1);
      c2->set_name(in->get_inputCloud1());
    }

    cloud_1.reset();
    cloud_2.reset();
    // pro kazdy bod v cloud1
    // pokud neni jejich vzdalenost mensi nez 0,001
    //ulozit do noveho cloudu
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_TN(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud (c2->get_Cloud());

    for(int i=0; i < c1->get_Cloud()->points.size();i++)
    {
      pcl::PointXYZI searchPointV;
      searchPointV=c1->get_Cloud()->points.at(i);
      std::vector<int> pointIDv;
      std::vector<float> pointSDv;

      if(kdtree.radiusSearch(searchPointV,0.001,pointIDv,pointSDv) <1)
        cloud_out->points.push_back(searchPointV);
    }
    cloud_out->width = cloud_out->points.size ();
    cloud_out->is_dense=true;
    cloud_out->height=1;

    Cloud *cV = new Cloud(cloud_out,in->get_outputCloud1());
    saveCloud(cV, type);

    cloud_out.reset();
    delete c1;
    delete c2;
    delete cV;
  }
  delete in;
}
void MainWindow::voxelize()
{

  InputDialog *in = new InputDialog(this);
  in->set_title("Voxelize Input Cloud");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool creates a voxelized cloud of a given resolution (voxel size). "
                      "Resulting point cloud consists of centroids of original points within individual voxels. "
                      "The tool may be used for the point cloud density reduction.");
  in->set_inputCloud1("Input Cloud:",get_allNames());
  in->set_outputCloud1("Output Cloud Name:","voxel");
  in->set_inputInt("Voxel Size in [cm]:","10");
  in->set_inputCheckBox("Should centroids be aligned?");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    float res =in->get_intValue()/100.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxels (new pcl::PointCloud<pcl::PointXYZI>);
    if(in->get_CheckBox()==false)
    {
      pcl::VoxelGrid<pcl::PointXYZI> vox;
      vox.setInputCloud (Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
      vox.setLeafSize (res, res, res);
      vox.filter (*cloud_voxels);
    }
    else
    {
      float res =in->get_intValue()/100.0;
      pcl::octree::OctreePointCloud<pcl::PointXYZI> oc (res);
      oc.setInputCloud (Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());
      oc.addPointsFromInputCloud ();
      // zjistit vsechny voxely
      std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > voxels;
      oc.getOccupiedVoxelCenters(voxels);
      oc.deleteTree();

      cloud_voxels->points.resize(voxels.size());
      #pragma omp parallel for
      for(int r=0; r < voxels.size(); r++)
      {
        cloud_voxels->points.at(r) = voxels.at(r);
      }
    }
    cloud_voxels->width = cloud_voxels->points.size ();
    cloud_voxels->height = 1;
    cloud_voxels->is_dense = true;
    Proj->save_newCloud("ost",in->get_outputCloud1(),cloud_voxels);
    QString fullnameV = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(in->get_outputCloud1());
    openCloudFile(fullnameV, "ost");
    cloud_voxels.reset();

  }
  delete in;
}
void MainWindow::duplicatePoints()
{
  InputDialog *in = new InputDialog(this);
  in->set_title("Duplicate Points Removal");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool finds and erases duplicate points within the cloud.");
  in->set_inputList("Input Clouds:",get_allNames());
    in->setSboxDouble("Minimal Distance between Points in [m]:", 0.002);
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted && !in->get_inputList().isEmpty())
  {
    QList<QString> clouds;
    clouds = in->get_inputList();
    createPBar();
    int v=0;
    int percent= 100/ clouds.size();
      float res = in->getSboxValue();
    #pragma omp parallel for
    for(int q=0; q < clouds.size(); q++)
    {

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::VoxelGrid<pcl::PointXYZI> vox;
      vox.setInputCloud (Proj->get_Cloud(clouds.at(q)).get_Cloud());
      vox.setLeafSize (res, res, res);
      vox.filter (*cloud_);

      if( cloud_->points.size () < Proj->get_Cloud(clouds.at(q)).get_Cloud()->points.size ())
      {
        cloud_->width = cloud_->points.size ();
        cloud_->height = 1;
        cloud_->is_dense = true;
        QString name = clouds.at(q);
        name.replace(".pcd","");
        Proj->get_Cloud(clouds.at(q)).set_Cloud(cloud_);
        Proj->save_Cloud(name,cloud_);
        cloud_.reset();
      }
      showPBarValue(v+= percent);
    }
    removePbar();
  }
  delete in;
}
void MainWindow::splitCloud()
{
  QStringList field;
  field << "x" << "y"<< "z";

  InputDialog *in = new InputDialog(this);
  in->set_title("Cloud Subtraction");
  in->set_path(Proj->get_Path());
  in->set_description("This tool splits cloud into two clouds in the middle of the range of selected coordinate.");
  in->set_inputCloud1("1st Input Cloud:",get_allNames());
  in->set_inputCloud2("Field for Split:",field);
  in->set_outputCloud1("Output Cloud:","left");
  in->set_outputCloud2("Output Cloud:","right");
  in->set_stretch();
  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    pcl::PointXYZI minp,maxp;
    pcl::getMinMax3D(*Proj->get_Cloud(in->get_inputCloud1()).get_Cloud(),minp,maxp);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZI>);
    float half;

    if(in->get_inputCloud2() == "x")
      half = (minp.x + maxp.x)/2;
    if(in->get_inputCloud2() == "y")
      half = (minp.y + maxp.y)/2;
    if(in->get_inputCloud2() == "z")
      half = (minp.z + maxp.z)/2;

    for(int i = 0; i <Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.size(); i++)
    {
      pcl::PointXYZI bod;
      bod = Proj->get_Cloud(in->get_inputCloud1()).get_Cloud()->points.at(i);
      float h;
      if(in->get_inputCloud2() == "x")
        h = bod.x;
      if(in->get_inputCloud2() == "y")
        h = bod.y;
      if(in->get_inputCloud2() == "z")
        h = bod.z;

      if(h > half)
        cloud_left->points.push_back(bod);
      else
        cloud_right->points.push_back(bod);
    }

    if(cloud_left->points.size() > 0)
    {
      cloud_left->width = cloud_left->points.size ();
      cloud_left->is_dense=true;
      cloud_left->height=1;

      Cloud *left = new Cloud(cloud_left,in->get_outputCloud1());
      saveCloud (left, "vege");
      delete left;
    }

    if(cloud_right->points.size() > 0)
    {
      cloud_right->width = cloud_right->points.size ();
      cloud_right->is_dense=true;
      cloud_right->height=1;

      Cloud *right = new Cloud(cloud_right,in->get_outputCloud2());
      saveCloud (right, "vege");
      delete right;
    }
    cloud_right.reset();
    cloud_left.reset();
  }
  delete in;
}
void MainWindow::set_ConcaveCloud()
{
QStringList names;
  names << get_allNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("2D Concave Hull");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool creates convex 2D hull of the input cloud. "
                      "It creates a new cloud consisting only from points that form the convex 2D hull of planar projection of the input cloud.");
  in->set_inputCloud1("Input Cloud:",get_allNames());
  in->set_outputCloud1("Output Cloud Name:","hull");
  in->set_inputInt("Initial Search Distance in [cm]:","100");
  in->set_stretch();

  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    createPBar();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    ConcaveHull *c = new ConcaveHull(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud(),"name",(float)in->get_intValue()/100);

    Proj->save_newCloud("ost",in->get_outputCloud1(),CloudOperations::getCloudCopy(c->getPolygon().get_Cloud()));
    QString fullnameV = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(in->get_outputCloud1());
    openCloudFile(fullnameV, "ost");

    cloud_.reset();
    delete c;

    showProgressBarAt(pBar,100);
  }
}
void MainWindow::set_ConvexCloud()
{
  QStringList names;
  names << get_allNames();

  InputDialog *in = new InputDialog(this);
  in->set_title("2D Convex Hull ");
  in->set_path(Proj->get_Path());
  in->set_description("\tThis tool creates convex 2D hull of the input cloud. "
                      "It creates a new cloud consisting only from points that form the convex 2D hull of planar projection of the input cloud.");
  in->set_inputCloud1("Input Cloud:",get_allNames());
  in->set_outputCloud1("Output Cloud Name:","hull");
  in->set_stretch();

  int dl = in->exec();

  if(dl == QDialog::Rejected)
    { return; }

  if(dl == QDialog::Accepted)
  {
    createPBar();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    ConvexHull *c = new ConvexHull(Proj->get_Cloud(in->get_inputCloud1()).get_Cloud());

    Proj->save_newCloud("ost",in->get_outputCloud1(),CloudOperations::getCloudCopy(c->getPolygon()));
    QString fullnameV = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(in->get_outputCloud1());
    openCloudFile(fullnameV, "ost");

    cloud_.reset();
    delete c;

    showProgressBarAt(pBar,100);
  }
}
void MainWindow::save_tiff()
{
  QString filename = QFileDialog::getSaveFileName(this,("Insert file name"),"",tr("PNG files (*.png *.PNG)"));
  if(!filename.isEmpty())
  {

      vtkSmartPointer<vtkRenderWindow> renderWindow = m_vis->getRenderWindow();


      vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
      windowToImageFilter->SetInput(renderWindow);

      windowToImageFilter->Update();

      vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
      writer->SetFileName(filename.toUtf8().constData());
      writer->SetInputConnection(windowToImageFilter->GetOutputPort());
      writer->Write();
      //m_vis->spinOnce();


//    vtkSmartPointer<vtkRenderWindow> renderWindow = m_vis->getRenderWindow();
//    renderWindow->SetOffScreenRendering(1);
//
//    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
//    vtkSmartPointer<vtkWindowToImageFilter>::New();
//    windowToImageFilter->SetInput(renderWindow);
//    windowToImageFilter->SetMagnification(3);
//
//    vtkSmartPointer<vtkTIFFWriter> tiffWriter =
//    vtkSmartPointer<vtkTIFFWriter>::New();
//    tiffWriter->SetFileName(filename.toUtf8().constData());
//    tiffWriter->SetCompressionToDeflate ();
//    tiffWriter->SetInputConnection(windowToImageFilter->GetOutputPort());
//    tiffWriter->Write();
//
//    renderWindow->SetOffScreenRendering(0);
    qvtkwidget->update();

    //windowToImageFilter->Delete();
  //  tiffWriter->Delete();
  }
}
void MainWindow::bgColor()
{
  QColor c = QColorDialog::getColor(Qt::white,this);
  if(c.isValid())
  {
    m_vis->setBackgroundColor(c.redF(),c.greenF(),c.blueF()); // Background color
  }
}
void MainWindow::accuracy()
{
  // sensitivity dataset
  boost::mt19937 seed( (float)std::time(NULL));
  boost::uniform_real<> dist(-1.2,12.45);
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(seed,dist);

  //definovat stred kruznice
  float stredx=5.5, stredy=4.5, stredz=1.31;
  int rr=1;
  //#pragma omp parallel for
  for(int r=1; r < 501 ; r+=rr) //radius
  {
    if(r ==10)
      rr=10;
    if(r==100)
      rr=100;


    float radius = (float)r/200.0;
    for(int u=360; u>0; u-=36) //uhel
    {
      float uh= u*100/360;
      for(int i=0;i <100; i+=10) //noise
      {
        int nn=1;
        for(int n=3; n < 501; n+=nn)//pocet bodu
        {
          if(n ==10)
            nn=10;
          if(n == 100)
            nn=100;
          // vypoÄet uhlu kde umisti body - radiany
          float uhel = ((float)u * M_PI/180)/(float)n;

          //vypocet poÄtu noise bodÅ¯
          float noiseNum = ((float)n /(1-((float)i/100))) - (float)n;
          // mracno do ktereho se ulozi vsechny body
          pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZI>);// mracno s centroidy vsech clusterÅ¯
          // pozice
          pcl::PointXYZI pose;
          pose.x=stredx;
          pose.y=stredy;
          pose.z=0.01;
          pose.intensity=0;
          cloud_->points.push_back(pose);
          // body kruhu
          for(float s=0; s < u * M_PI/180; s+=uhel)
          {
            pcl::PointXYZI bod;
            bod.x = radius * std::cos(s) + stredx;
            bod.y = radius * std::sin(s) + stredy;
            bod.z = stredz;
            bod.intensity=0;
            cloud_->points.push_back(bod);
          }
          // noise body
          for(int no=0; no < noiseNum; no++)
          {
            pcl::PointXYZI bod;
            bod.x = (float)random();
            bod.y = (float)random();
            bod.z = stredz;
            bod.intensity=0;
            cloud_->points.push_back(bod);
          }
          cloud_->width = cloud_->points.size ();
          cloud_->height = 1;
          cloud_->is_dense = true;

          QString name = QString("Diameter_%1_Angle_%2_Noise_%3_Points_%4").arg(r).arg(uh).arg(i).arg(n);
          Cloud *c = new Cloud(cloud_, name);
          saveCloud(c, "strom");
QString namePCD =QString("%1.pcd").arg(name);
          // pozice
          Proj->set_treePosition(namePCD,2);
          //RHT
          Proj->get_TreeCloud(namePCD).set_dbhHT(100);
          // LSR
          Proj->get_TreeCloud(namePCD).set_dbhLSR();
          delete c;
          qWarning()<<namePCD<<" done";
        }
      }
    }
  }

//   InputDialog *in = new InputDialog(this);
//  in->set_title("Compute tree DBH using Randomized Hough Transform (RHT)");
//  in->set_path(Proj->get_Path());
//  in->set_description("\tblbblalblablablbalablablabl ");
//  in->set_inputList("Input Tree cloud:",get_allNames());
//  in->set_outputPath("Path to the new file:","c:\\exported_cloud.txt","Text file (*.txt)");
//  in->set_stretch();
//  int dl = in->exec();
//
//  if(dl == QDialog::Rejected)
//    { return; }
//
//  if(dl == QDialog::Accepted && in->get_inputList().size() >0)
//  {
//     QFile file (in->get_outputPath());
//      file.open(QIODevice::WriteOnly | QIODevice::Text);
//      QTextStream out(&file);
//      out << "name1\tbodu\tname2\tbodu\tTP\tFN\tFP\n";
//      out.setRealNumberNotation(QTextStream::FixedNotation);
//      out.setRealNumberPrecision(3);
//
//    createPBar();
//
//    QList<QString> clouds;
//    clouds = in->get_inputList();
//    float v=0;
//    float percent = 100/ clouds.size();
//    showPBarValue(v);
//    for(int q=0; q < clouds.size(); q++)
//    {
//      pcl::PointXYZI minp, maxp,mean; // body ohranicujici vegetaci
//      int closest;
//      float distance = 9999999999;
//      pcl::getMinMax3D(*Proj->get_Cloud(clouds.at(q)).get_Cloud(),minp, maxp);
//      mean.x=(maxp.x+minp.x)/2;
//      mean.y=(maxp.y+minp.y)/2;
//      mean.z=(maxp.z+minp.z)/2;
//      // najdi nejblizsi podle pozice?
//      // pro kazdy strom pokud neni stejne jmeno
//     // zjistit pÅekryv boundingboxu a ten s nejvetsim prekryvem pouzit
//      for(int a=0; a < Proj->get_sizeTreeCV();a++)
//      {
//        if(Proj->get_TreeCloud(a).get_name() == clouds.at(q) || Proj->get_TreeCloud(a).get_name().startsWith("AUT") !=true)
//          continue;
//        pcl::PointXYZI tminp, tmaxp, tmean; // body ohranicujici vegetaci
//        pcl::getMinMax3D(*Proj->get_TreeCloud(a).get_Cloud(),tminp, tmaxp);
//        //zjisti bounding box a prumernoy souradnici, porovnat s tou nahore a najit nejblizsi
//        tmean.x=(tmaxp.x+tminp.x)/2;
//        tmean.y=(tmaxp.y+tminp.y)/2;
//        tmean.z=(tmaxp.z+tminp.z)/2;
//        float dis = GeomCalc::computeDistance3D(tmean,mean);
//        if(dis < distance)
//        {
//          distance = dis;
//          closest = a;
//        }
//      }

//
//      int truepositive=0; // shoda
//      int falsenegative=0; // body v ID, ale nejsou v AUT
//      int falsepositive=0; // body v AUt ale nejsou v ID
//
//
//      // pro zadany strom
//      pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> ocs (0.1);
//
//      ocs.setInputCloud (Proj->get_TreeCloud(closest).get_Cloud());
//      ocs.addPointsFromInputCloud ();
//      for(int s=0; s< Proj->get_Cloud(clouds.at(q)).get_Cloud()->points.size(); s++)// spatne strom ID
//      {
//        std::vector<int> IDs;
//        std::vector<float>dists;
//        pcl::PointXYZI bod = Proj->get_Cloud(clouds.at(q)).get_Cloud()->points.at(s);
//        if(ocs.radiusSearch(bod, 0.0001, IDs, dists)>0) // nalezena shoda
//           truepositive++;
//        else
//          falsenegative++;
//      }
//
//      falsepositive = Proj->get_TreeCloud(closest).get_Cloud()->points.size() - truepositive;
//
//
//      QString row=QString("%1\t%2\t%3\t%4\t%5\t%6\t%7\n") .arg(clouds.at(q))
//                                                  .arg(Proj->get_Cloud(clouds.at(q)).get_Cloud()->points.size())
//                                                  .arg(Proj->get_TreeCloud(closest).get_name())
//                                                  .arg(Proj->get_TreeCloud(closest).get_Cloud()->points.size())
//                                                  .arg(truepositive)
//                                                  .arg(falsenegative)
//                                                  .arg(falsepositive);
//      out << row;
//      showPBarValue(v+=percent);
//    }
//    file.close();
//    removePbar();
//  }
}
void MainWindow::topView()
{
  m_vis->setCameraPosition(0,0,100000000 ,0,1,0);
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::bottomView()
{
  m_vis->setCameraPosition(0,0,-100000000 ,0,1,0);
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::frontView()
{
  m_vis->setCameraPosition(10000000000,0,0,0,0,1);
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::sideAView()
{
  m_vis->setCameraPosition(0,10000000000,0,0,0,1);
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::backView()
{
  m_vis->setCameraPosition(-10000000000,0,0,0,0,1);
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::sideBView()
{
  m_vis->setCameraPosition(0,-10000000000,0,0,0,1);
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::perspective()
{
  m_vis->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(0);
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::ortho()
{
  m_vis->getRenderWindow ()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(1);
  m_vis->resetCamera();
  qvtkwidget->update();
}
void MainWindow::about()
{
  QMessageBox::about(this, tr("About 3D Forest Application"),tr(" The 3D Forest application is presented in version 0.52.\n"
      "SOme parts like QSM or CharCoal are implemented into 3D Forest thanks to various grants. Thanks you for such help. "
                                                             "Application serves for extraction of tree parameters like tree position, dbh, or advanced QSM models from TLS data in a forest environment."
                                                             "3D Forest is released under terms of GPL v3.\n"
                                                             "More information can be found on web site www.3dforest.eu or on the wiki section on our GitHub https://github.com/janekT/3DForest. \n\n"
                                                             "  AUTHORS:\n "
                                                             "\tJan Trochta j.trochta@gmail.com \n"
                                                             "\tMartin Krucek krucek.martin@gmail.com\n"
                                                             "\tKamil Kral kamil.kral@vukoz.cz"));

  return;

 }

//ACTION and MENUS
void MainWindow::createActions()
{
//FILE
  new_projectAct = new QAction(QPixmap(":/images/projectBar/new.png"), tr("New Project"), this);
  new_projectAct->setStatusTip(tr("Creates Project."));
  connect(new_projectAct, SIGNAL(triggered()), this, SLOT(newProject()));

  open_projectAct = new QAction(QPixmap(":/images/projectBar/open.png"), tr("Open Project"), this);
  open_projectAct->setStatusTip(tr("Opens Project."));
  connect(open_projectAct, SIGNAL(triggered()), this, SLOT(openProject()));

  close_projectAct = new QAction(QPixmap(":/images/projectBar/close.png"), tr("Close Project"), this);
  close_projectAct->setStatusTip(tr("Closes Project."));
  connect(close_projectAct, SIGNAL(triggered()), this, SLOT(closeProject()));

  import_projectAct = new QAction(QPixmap(":/images/projectBar/import.png"), tr("Import Project"), this);
  import_projectAct->setStatusTip(tr("Imports existing project into new folder."));
  connect(import_projectAct, SIGNAL(triggered()), this, SLOT(importProject()));

  importBaseAct = new QAction(tr("Base Cloud Import"), this);
  importBaseAct->setStatusTip(tr("Imports new base cloud into project. Various formats are available."));
  connect(importBaseAct, SIGNAL(triggered()), this, SLOT(importBaseCloud()));

  importTerenAct = new QAction(tr("Terrain Cloud Import"), this);
  importTerenAct->setStatusTip(tr("Imports new Terrain cloud into project. Various formats are available."));
  connect(importTerenAct, SIGNAL(triggered()), this, SLOT(importTerrainFile()));

  importVegeAct = new QAction(tr("Vegetation Cloud Import"), this);
  importVegeAct->setStatusTip(tr("Imports new vegetation cloud into project. Various formats are available."));
  connect(importVegeAct, SIGNAL(triggered()), this, SLOT(importVegeCloud()));

  importTreeAct = new QAction(tr("Tree Cloud Import"), this);
  importTreeAct->setStatusTip(tr("Imports new tree cloud into project. Various formats are available."));
  connect(importTreeAct, SIGNAL(triggered()), this, SLOT(importTreeCloud()));

  exportTXTAct = new QAction(tr("Cloud Export"), this);
  exportTXTAct->setStatusTip(tr("Exports selected clouds into TXT file, PLY file or PTS file."));
  connect(exportTXTAct, SIGNAL(triggered()), this, SLOT(exportCloud()));

  exitAct = new QAction(QPixmap(":/images/exit.png"),tr("Exit"), this);
  exitAct->setShortcuts(QKeySequence::Quit);
  exitAct->setStatusTip(tr("Terminates the application."));
  connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

//TERRAIN
  voxelAct = new QAction(tr("Terrain by Voxels"), this);
  voxelAct->setStatusTip(tr("Automatic terrain extraction method using voxelized derivate of input cloud; result is voxelized."));
 // voxelAct->setEnabled(false);
  connect(voxelAct, SIGNAL(triggered()), this, SLOT(voxelgrid()));

  octreeAct = new QAction(tr("Terrain by Octree"), this);
  octreeAct->setStatusTip(tr("Automatic terrain extraction method searching for lowest points in octree search."));
  //octreeAct->setEnabled(false);
  connect(octreeAct, SIGNAL(triggered()), this, SLOT(octreeSlot()));

  manualADAct = new QAction(tr("Manual Adjustment"), this);
  manualADAct->setStatusTip(tr("Manual adjustment of selected terrain cloud; serves for deletion of non-ground points from a terrain cloud."));
 // manualADAct->setEnabled(false);
  connect(manualADAct, SIGNAL(triggered()), this, SLOT(manualAdjust()));

  IDWAct = new QAction(tr("IDW"), this);
  IDWAct->setStatusTip(tr("IDW method for interpolating points of terrain. As a boundary minimal and maximal coordinates of input Terrain cloud are used. "));
  connect(IDWAct, SIGNAL(triggered()), this, SLOT(IDWslot()));

  statisticalOutlierRemovalAct = new QAction(tr("Statistical Outlier Removal"), this);
  statisticalOutlierRemovalAct->setStatusTip(tr("Removes outliers by mean k-nearest neighbor distance."));
 // statisticalOutlierRemovalAct->setEnabled(false);
  connect(statisticalOutlierRemovalAct, SIGNAL(triggered()), this, SLOT(statisticalOutlierRemovalTerrain()));

  radiusOutlierRemovalAct = new QAction(tr("Radius Outlier Removal"), this);
  radiusOutlierRemovalAct->setStatusTip(tr("Removes outliers by amount of neighbors inside the radius."));
 //radiusOutlierRemovalAct->setEnabled(false);
  connect(radiusOutlierRemovalAct, SIGNAL(triggered()), this, SLOT(radiusOutlierRemovalTerrain()));

    slopeAct = new QAction(tr("Slope"), this);
    slopeAct->setStatusTip(tr("Computes slope of terrain."));
    //radiusOutlierRemovalAct->setEnabled(false);
    connect(slopeAct, SIGNAL(triggered()), this, SLOT(slope()));
    
    aspectAct = new QAction(tr("Aspect"), this);
    aspectAct->setStatusTip(tr("Computes terrain aspect."));
    //radiusOutlierRemovalAct->setEnabled(false);
    connect(aspectAct, SIGNAL(triggered()), this, SLOT(aspect()));
    
    curvatureAct = new QAction(tr("Curvature"), this);
    curvatureAct->setStatusTip(tr("Computes terrain curvature."));
    //radiusOutlierRemovalAct->setEnabled(false);
    connect(curvatureAct, SIGNAL(triggered()), this, SLOT(curvature()));
    
    hillShadeAct = new QAction(tr("Hillshade"), this);
    hillShadeAct->setStatusTip(tr("Computes hillshade of terrain."));
    //radiusOutlierRemovalAct->setEnabled(false);
    connect(hillShadeAct, SIGNAL(triggered()), this, SLOT(hillShade()));
    
    pointDensityAct = new QAction(tr("Point Density"), this);
    pointDensityAct->setStatusTip(tr("Computes point denisty."));
    //radiusOutlierRemovalAct->setEnabled(false);
    connect(pointDensityAct, SIGNAL(triggered()), this, SLOT(pointDensity()));
    //MILIRE
    terrainDiffAct = new QAction(tr("Charcoal sites"), this);
    terrainDiffAct->setStatusTip(tr("Computes features representing similar spots from terrain based on input parameters"));
    //radiusOutlierRemovalAct->setEnabled(false);
    connect(terrainDiffAct, SIGNAL(triggered()), this, SLOT(terrainDiff()));
    
    featureTableAct = new QAction(QPixmap(":/images/icon.png"),tr("&Feature table"), this);
     featureTableAct->setStatusTip(tr("Shows information about 3D Forest application."));
     connect(featureTableAct, SIGNAL(triggered()), this, SLOT(showFeatureTable()));
    
    exportFeaturesAct = new QAction(tr("Export charcoal sites"), this);
    exportFeaturesAct->setStatusTip(tr("Exports features into text file with all parameteres and file with convex and concave hulls"));
    //radiusOutlierRemovalAct->setEnabled(false);
    connect(exportFeaturesAct, SIGNAL(triggered()), this, SLOT(exportFeaturesAtt()));

//VEGETATION
  manualSelAct = new QAction(tr("Manual Tree Selection"), this);
  manualSelAct->setStatusTip(tr("Manual segmentation of individual trees from vegetation cloud. One can iteratively delete points that do not belong to the tree."));
  connect(manualSelAct, SIGNAL(triggered()), this, SLOT(manualSelect()));

  segmentAct = new QAction(tr("Automatic Tree Segmentation"), this);
  segmentAct->setStatusTip(tr("Automatic segmentation of vegetation cloud into individual trees."));
  connect(segmentAct, SIGNAL(triggered()), this, SLOT(segmentation()));

  mergeCloudsAct = new QAction(tr("Merge Clouds by ID"), this);
  mergeCloudsAct->setStatusTip(tr("Automatic merging of clouds by ID."));
  connect(mergeCloudsAct, SIGNAL(triggered()), this, SLOT(mergeCloudsByID()));

  eraseSelectedCloudsAct = new QAction(tr("Erase Selected Clouds"), this);
  eraseSelectedCloudsAct->setStatusTip(tr("Erase all selected clouds from project and disc."));
  connect(eraseSelectedCloudsAct, SIGNAL(triggered()), this, SLOT(eraseSelectedClouds()));

//TREE ATRIBUTES
  tAAct = new QAction(tr("Export Tree Parameters"), this);
  tAAct->setStatusTip(tr("Exports Tree parameters into text file. Select which attributes should be exported and how they should be separated in the text file."));
  tAAct->setEnabled(false);
  connect(tAAct, SIGNAL(triggered()), this, SLOT(treeAtributes()));

  dbhHTAct = new QAction(QPixmap(":/images/treeBar/dbhHT.png"),tr("DBH RHT"), this);
  dbhHTAct->setStatusTip(tr("Computes and displays DBH using Randomized Hough Transform method; displays tree cloud subset used for computing DBH and estimated cylinder with DBH value."));
  dbhHTAct->setEnabled(false);
  connect(dbhHTAct, SIGNAL(triggered()), this, SLOT(dbhHT()));

  dbhLSRAct = new QAction(QPixmap(":/images/treeBar/dbhLSR.png"), tr("DBH LSR"), this);
  dbhLSRAct->setStatusTip(tr("Computes and displays DBH using Least Squares Regression method; display tree cloud subset used for computing DBH and estimated cylinder with DBH value."));
  dbhLSRAct->setEnabled(false);
  connect(dbhLSRAct, SIGNAL(triggered()), this, SLOT(dbhLSR()));

  heightAct = new QAction(QPixmap(":/images/treeBar/height.png"),tr("Height"), this);
  heightAct->setStatusTip(tr("Computes and displays height of the tree as a line starting at tree base position and follows Z axis to the height of highest point of the tree."));
  heightAct->setEnabled(false);
  connect(heightAct, SIGNAL(triggered()), this, SLOT(height()));

  posAct = new QAction(QPixmap(":/images/treeBar/position.png"), tr("Position Lowest Points"), this);
  posAct->setStatusTip(tr("Computes and displays position of the tree as a sphere with diameter 10cm and center at tree position."));
  connect(posAct, SIGNAL(triggered()), this, SLOT(position()));

  posHTAct = new QAction(QPixmap(":/images/treeBar/position.png"), tr("Position RHT"), this);
  posHTAct->setStatusTip(tr("Computes and displays position of the tree as a sphere with diameter 10cm and center at tree position. "));
  connect(posHTAct, SIGNAL(triggered()), this, SLOT(positionHT()));

  treeEditAct = new QAction(tr("Tree Cloud Edit"), this);
  treeEditAct->setStatusTip(tr("Edits tree cloud and saves deleted parts in a new file."));
  connect(treeEditAct, SIGNAL(triggered()), this, SLOT(treeEdit()));

  dbhEditAct = new QAction(tr("DBH Cloud Edit"), this);
  dbhEditAct->setStatusTip(tr("Edits tree cloud used for computing DBH."));
  dbhEditAct->setEnabled(false);
  connect(dbhEditAct, SIGNAL(triggered()), this, SLOT(dbhCloudEdit()));

  lengAct = new QAction(QPixmap(":/images/treeBar/length.png"),tr("Length"), this);
  lengAct->setStatusTip(tr("Computes and displays length of the tree as a line between two points of the tree cloud with the greatest distance."));
  lengAct->setEnabled(false);
  connect(lengAct, SIGNAL(triggered()), this, SLOT(length()));



  convexAct = new QAction(QPixmap(":/images/treeBar/convex.png"),tr("Convex Planar Projection"), this);
  convexAct->setStatusTip(tr("Computes and displays convex planar projection of the tree as a polygon in the color of the tree cloud with 50% opacity and with value of the polygon area."));
  convexAct->setEnabled(false);
  connect(convexAct, SIGNAL(triggered()), this, SLOT(convexhull()));

  concaveAct = new QAction(QPixmap(":/images/treeBar/concave.png"),tr("Concave planar projection"), this);
  concaveAct->setStatusTip(tr("Compute and display concave planar projection of the tree as a polygon in the color of the tree cloud with 50% opacity and with value of polygon area."));
  concaveAct->setEnabled(false);
  connect(concaveAct, SIGNAL(triggered()), this, SLOT(concavehull()));

  stemCurvatureAct = new QAction(QPixmap(":/images/treeBar/stemCurve.png"),tr("Stem Curve"), this);
  stemCurvatureAct->setStatusTip(tr("Computes and displays cylinders fitting the stem in meter sections."));
  stemCurvatureAct->setEnabled(false);
  connect(stemCurvatureAct, SIGNAL(triggered()), this, SLOT(stemCurvature()));

  exportStemCurvAct = new QAction(tr("Export Stem Curve"), this);
  exportStemCurvAct->setStatusTip(tr("Exports center coordinates and diameters of stem curve rings into text file."));
  exportStemCurvAct->setEnabled(false);
  connect(exportStemCurvAct, SIGNAL(triggered()), this, SLOT(stemCurvatureExport()));

  exportCONVEXAct = new QAction(tr("Export Convex Planar Projection (txt)"), this);
  exportCONVEXAct->setStatusTip(tr("Exports convex planar projection of selected tree(s) into the text file."));
  exportCONVEXAct->setEnabled(false);
  connect(exportCONVEXAct, SIGNAL(triggered()), this, SLOT(exportConvexTxt()));

  exportCONCAVEAct = new QAction(tr("Export Concave Planar Projection (txt)"), this);
  exportCONCAVEAct->setStatusTip(tr("Exports concave planar projection of selected tree(s) into the text file."));
  exportCONCAVEAct->setEnabled(false);
  connect(exportCONCAVEAct, SIGNAL(triggered()), this, SLOT(exportConcaveTxt()));




//CROWN             new QAction(QPixmap(":/images/treeBar/length.png"),tr("Length"), this);
  setCrownManualAct= new QAction(tr("Set Manual"), this);
  setCrownManualAct->setStatusTip(tr("Set crown by manual selection of points representing the tree crown."));
  connect(setCrownManualAct, SIGNAL(triggered()), this, SLOT(set_CrownManual()));
  setCrownManualAct->setEnabled(false);

  setCrownAutomaticAct= new QAction(tr("Set Automatic"), this);
  setCrownAutomaticAct->setStatusTip(tr("Set the points representing the tree crown automatically."));
  connect(setCrownAutomaticAct, SIGNAL(triggered()), this, SLOT(set_CrownAutomatic()));
  setCrownAutomaticAct->setEnabled(false);

  setCrownSectionsAct= new QAction(QPixmap(":/images/crownBar/crownSections.png"),tr("Volume and Surface by Sections"), this);
  setCrownSectionsAct->setStatusTip(tr("Computes and displays crown surface and volume using horizontal cross sections."));
  connect(setCrownSectionsAct, SIGNAL(triggered()), this, SLOT(setSectionsVolumeSurfacePosition()));
  setCrownSectionsAct->setEnabled(false);

  setVolumeByVoxAct= new QAction(tr("Volume by Voxels"), this);
  setVolumeByVoxAct->setStatusTip(tr("Computes crown volume by voxels of given resolution."));
  connect(setVolumeByVoxAct, SIGNAL(triggered()), this, SLOT(crownVolumeByVoxels()));
  setVolumeByVoxAct->setEnabled(false);

  exportAttributesAct= new QAction(tr("Export Attributes"), this);
  exportAttributesAct->setStatusTip(tr("Exports tree crown attributes into txt file."));
  connect(exportAttributesAct, SIGNAL(triggered()), this, SLOT(exportCrownAttributes()));
  exportAttributesAct->setEnabled(false);

  convexHull3DAct= new QAction(QPixmap(":/images/crownBar/crownConvex.png"),tr("Compute 3D ConvexHull"), this);
  convexHull3DAct->setStatusTip(tr("Creates and displays the 3D convex hull of the crown and its volume and surface area."));
  connect(convexHull3DAct, SIGNAL(triggered()), this, SLOT(create3DConvexull()));
  convexHull3DAct->setEnabled(false);

  intersectionAct= new QAction(QPixmap(":/images/crownBar/crownIntersection.png"),tr("Intersections"), this);
  intersectionAct->setStatusTip(tr("Computes and displays intersection between two crowns, all possible intersections are verified."));
  connect(intersectionAct, SIGNAL(triggered()), this, SLOT(computeCrownsIntersections()));
  intersectionAct->setEnabled(false);

  exportIntersectionAct= new QAction(tr("Export Intersections Attributes"), this);
  exportIntersectionAct->setStatusTip(tr("Exports crown intersection attributes into txt file."));
  connect(exportIntersectionAct, SIGNAL(triggered()), this, SLOT(exportIntersections()));
  exportIntersectionAct->setEnabled(false);

    //QSM

    reconstructionAct = new QAction(tr("Tree Volume QSM"), this);
    reconstructionAct->setStatusTip(tr("Computes cylindrical volume of tree based on recostructed tree."));
    reconstructionAct->setEnabled(false);
    connect(reconstructionAct, SIGNAL(triggered()), this, SLOT(qsmModel()));

    sortimentAct = new QAction(tr("Tree Assortment"), this);
    sortimentAct->setStatusTip(tr("Cylindrical volume of recostructed tree."));
    sortimentAct->setEnabled(false);
    connect(sortimentAct, SIGNAL(triggered()), this, SLOT(sortimenty()));

    treeReconstructionAct = new QAction(tr("Tree Reconstruction"), this);
    treeReconstructionAct->setStatusTip(tr("Computes branches  and stem."));
    treeReconstructionAct->setEnabled(false);
    connect(treeReconstructionAct, SIGNAL(triggered()), this, SLOT(treeReconstruction()));

    exportQSMAct = new QAction(tr("QSM Data Export"), this);
    exportQSMAct->setStatusTip(tr("Exports into text file data about tree parts, QSM and assortment."));
    exportQSMAct->setEnabled(false);
    connect(exportQSMAct, SIGNAL(triggered()), this, SLOT(exportQSM()));

//MISC
  multipleMergeAct = new QAction(QPixmap(":/images/merge.png"), tr("Cloud Merge"), this);
  multipleMergeAct->setStatusTip(tr("Merges selected clouds into single cloud and save the result as a new cloud of desired type."));
  connect(multipleMergeAct, SIGNAL(triggered()), this, SLOT(mergeClouds()));

  voxAct = new QAction(tr("Voxelize Cloud"), this);
  voxAct->setStatusTip(tr("Creates new voxelized cloud of defined resolution from any selected cloud."));
  connect(voxAct, SIGNAL(triggered()), this, SLOT(voxelize()));

  minusAct = new QAction(QPixmap(":/images/substraction.png"), tr("Cloud Subtraction"), this);
  minusAct->setStatusTip(tr("Removes common points from bigger cloud and save the rest into new cloud."));
  connect(minusAct, SIGNAL(triggered()), this, SLOT(minusCloud()));

  convexCloudAct = new QAction(tr("Create Convex Hull"), this);
  convexCloudAct->setStatusTip(tr("Computes and displays convex planar projection of selected cloud. Saved as a new point cloud."));
  connect(convexCloudAct, SIGNAL(triggered()), this, SLOT(set_ConvexCloud()));

  concaveCloudAct = new QAction(tr("Create Concave Hull"), this);
  concaveCloudAct->setStatusTip(tr("Computes and displays concave planar projection of selected cloud. Saved as a new point cloud."));
  connect(concaveCloudAct, SIGNAL(triggered()), this, SLOT(set_ConcaveCloud()));

  tiffAct = new QAction(tr("Save Diplay into png"), this);
  tiffAct->setStatusTip(tr("Saves displayed clouds into tiff file."));
  connect(tiffAct, SIGNAL(triggered()), this, SLOT(save_tiff()));

  bgcolorAct = new QAction(tr("Change Background Color"), this);
  bgcolorAct->setStatusTip(tr("Changes background color of viewer to user defined color."));
  connect(bgcolorAct, SIGNAL(triggered()), this, SLOT(bgColor()));

  spitCloudAct = new QAction(QPixmap(":/images/substraction.png"),tr("Split Cloud"), this);
  spitCloudAct->setStatusTip(tr("Splits input cloud in the middle of selected coordinate and save into two separate clouds."));
  connect(spitCloudAct, SIGNAL(triggered()), this, SLOT(splitCloud()));

  labelONAct= new QAction(tr("Labels ON"), this);
  connect(labelONAct, SIGNAL(triggered()), this, SLOT(labelClouds()));

  labelOFFAct= new QAction(tr("Labels OFF"), this);
  connect(labelOFFAct, SIGNAL(triggered()), this, SLOT(labelCloudsOFF()));

  acuracyAct = new QAction(tr("Accuracy"), this);
  connect(acuracyAct, SIGNAL(triggered()), this, SLOT(accuracy()));

    filterRadiusAct = new QAction(tr("Filter - Radius Removal Filter."), this);
    connect(filterRadiusAct, SIGNAL(triggered()), this, SLOT(radiusOutlierRemoval()));

  duplicateAct = new QAction(tr("Remove Duplicate Points from Cloud."), this);
  connect(duplicateAct, SIGNAL(triggered()), this, SLOT(duplicatePoints()));
    
    removePointsAct= new QAction(tr("Remove Selected Points from Cloud."), this);
    connect(removePointsAct, SIGNAL(triggered()), this, SLOT(removePoints()));

//ABOUT
  aboutAct = new QAction(QPixmap(":/images/icon.png"),tr("&About"), this);
  aboutAct->setStatusTip(tr("Shows information about 3D Forest application."));
  connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
    
    
    

//EVENTS treewidget
  connect(treeWidget, SIGNAL(checkedON(QString)), this, SLOT(removeCloud(QString)));
  connect(treeWidget, SIGNAL(checkedOFF(QString)), this, SLOT(dispCloud(QString)));
  connect(treeWidget, SIGNAL(deleteItem(QString)), this, SLOT(deleteCloud(QString)));
  connect(treeWidget, SIGNAL(colorItem(QString)), this, SLOT(colorCloud(QString)));
  connect(treeWidget, SIGNAL(colorItemField(QString)), this, SLOT(colorCloudField(QString)));
  connect(treeWidget, SIGNAL(psize(QString)), this, SLOT(PointSize(QString)));
}
void MainWindow::createMenus()
{
//PROJECT
  fileMenu = menuBar()->addMenu(tr("&Project"));
  fileMenu->addAction(new_projectAct);
  fileMenu->addAction(open_projectAct);
  fileMenu->addAction(close_projectAct);
  fileMenu->addAction(import_projectAct);
  fileMenu->addSeparator();
  importMenu =  fileMenu->addMenu(tr("Import"));
  importMenu->addAction(importBaseAct);
  importMenu->addAction(importTerenAct);
  importMenu->addAction(importVegeAct);
  importMenu->addAction(importTreeAct);
  exportMenu = fileMenu->addMenu(tr("Export"));
  exportMenu->addAction(exportTXTAct);
  fileMenu->addSeparator();
  fileMenu->addAction(exitAct);

//TEREN
  terenMenu = menuBar()->addMenu(tr("&Terrain"));
  terenMenu->addAction(octreeAct);
  terenMenu->addAction(voxelAct);
  terenMenu->addSeparator();
  terenMenu->addAction(statisticalOutlierRemovalAct);
  terenMenu->addAction(radiusOutlierRemovalAct);
  terenMenu->addAction(manualADAct);
  terenMenu->addSeparator();
  terenMenu->addAction(IDWAct);
    terenMenu->addAction(slopeAct);
    terenMenu->addAction(aspectAct);
    terenMenu->addAction(curvatureAct);
    terenMenu->addAction(hillShadeAct);
    terenMenu->addAction(pointDensityAct);
    

//VEGETATION
  vegeMenu = menuBar()->addMenu(tr("&Vegetation"));
  vegeMenu->addAction(segmentAct);
  vegeMenu->addAction(manualSelAct);


//TREE ATRIBUTES
  treeMenu = menuBar()->addMenu(tr("&Trees"));
  treeMenu->addAction(treeEditAct);
  treeMenu->addAction(dbhEditAct);
 //treeMenu->addAction(dbhCheckAct);
  treeMenu->addSeparator();
  treeMenu->addAction(posAct);
  treeMenu->addAction(posHTAct);
  treeMenu->addAction(dbhHTAct);
  treeMenu->addAction(dbhLSRAct);
  treeMenu->addAction(heightAct);
  treeMenu->addAction(lengAct);
  treeMenu->addAction(stemCurvatureAct);
  treeMenu->addAction(convexAct);
  treeMenu->addAction(concaveAct);

  treeMenu->addSeparator();
  treeMenu->addAction(tAAct);
  treeMenu->addAction(exportStemCurvAct);
  treeMenu->addAction(exportCONVEXAct);
  treeMenu->addAction(exportCONCAVEAct);


//CROWN
  crownMenu = menuBar()->addMenu(tr("Crowns"));
  crownMenu->addAction(setCrownManualAct);
  crownMenu->addAction(setCrownAutomaticAct);
  crownMenu->addSeparator();
  crownMenu->addAction(setVolumeByVoxAct);
  crownMenu->addAction(setCrownSectionsAct);
  crownMenu->addAction(convexHull3DAct);
  crownMenu->addAction(intersectionAct);
  crownMenu->addSeparator();
  crownMenu->addAction(exportAttributesAct);
  crownMenu->addAction(exportIntersectionAct);

    //QSM
    qsmMenu = menuBar()->addMenu(tr("QSM"));
    qsmMenu->addAction(treeReconstructionAct);
    qsmMenu->addAction(reconstructionAct);
    qsmMenu->addAction(sortimentAct);
    qsmMenu->addAction(exportQSMAct);
//MILIRE
    milireMenu = menuBar()->addMenu(tr("CharCoal"));
    milireMenu->addAction(terrainDiffAct);
    milireMenu->addAction(featureTableAct);
    milireMenu->addAction(exportFeaturesAct);
    
//MISC
  miscMenu = menuBar()->addMenu(tr("Other features"));
  miscMenu->addAction(multipleMergeAct);
  miscMenu->addAction(minusAct);
  miscMenu->addAction(voxAct);
  miscMenu->addAction(convexCloudAct);
  miscMenu->addAction(concaveCloudAct);
  miscMenu->addSeparator();
  miscMenu->addAction(tiffAct);
  miscMenu->addAction(bgcolorAct);
  miscMenu->addSeparator();
 // miscMenu->addSeparator();
  miscMenu->addAction(filterRadiusAct);
  //miscMenu->addAction(eraseSelectedCloudsAct);
  //miscMenu->addAction(acuracyAct);
 // miscMenu->addAction(labelOFFAct);
  miscMenu->addAction(spitCloudAct);
  miscMenu->addAction(duplicateAct);
    miscMenu->addAction(removePointsAct);
//ABOUT
  helpMenu = menuBar()->addMenu(tr("&About"));
  helpMenu->addAction(aboutAct);
 }
void MainWindow::createToolbars()
{
//Project toolbar
  QToolBar *Projectbar = addToolBar("Project Toolbar");
  //Projectbar->setMaximumHeight(24);
  Projectbar->resize(24,80);
  Projectbar->setIconSize(QSize(24,24));
  QAction *newProjectT = Projectbar->addAction(QPixmap(":/images/projectBar/new.png"),"New Project");
  connect(newProjectT,SIGNAL(triggered()),this,SLOT(newProject()));
  QAction *openProjectT = Projectbar->addAction(QPixmap(":/images/projectBar/open.png"),"Open Project");
  connect(openProjectT,SIGNAL(triggered()),this,SLOT(openProject()));
  QAction *closeProjectT = Projectbar->addAction(QPixmap(":/images/projectBar/close.png"),"Close Project");
  connect(closeProjectT,SIGNAL(triggered()),this,SLOT(closeProject()));
  QAction *importProjectT = Projectbar->addAction(QPixmap(":/images/projectBar/import.png"),"Import Project");
  connect(importProjectT,SIGNAL(triggered()),this,SLOT(importProject()));
//ATTRIBUTE TABLE
  showAttributTableT = Projectbar->addAction(QPixmap(":/images/projectBar/attTable.png"),"Attribute table");
  connect(showAttributTableT,SIGNAL(triggered()),this,SLOT(showAttributeTable()));
  showAttributTableT->setEnabled(false);

//ViewBar
  QToolBar *viewBar = addToolBar("View Toolbar");          /**< Toolbar with setting for display*/
  viewBar->resize(24,80);
  viewBar->setIconSize(QSize(24,24));
  frontViewAct = viewBar->addAction(QPixmap(":/images/viewBar/front.png"),"Front view");
  connect(frontViewAct,SIGNAL(triggered()),this,SLOT(frontView()));
  backViewAct = viewBar->addAction(QPixmap(":/images/viewBar/back.png"),"Back view");
  connect(backViewAct,SIGNAL(triggered()),this,SLOT(backView()));
  topViewAct = viewBar->addAction(QPixmap(":/images/viewBar/top.png"),"Top view");
  connect(topViewAct,SIGNAL(triggered()),this,SLOT(topView()));
  bottomViewAct = viewBar->addAction(QPixmap(":/images/viewBar/bottom.png"),"Bottom view");
  connect(bottomViewAct,SIGNAL(triggered()),this,SLOT(bottomView()));
  sideBViewAct = viewBar->addAction(QPixmap(":/images/viewBar/sideB.png"),"Side view");
  connect(sideBViewAct,SIGNAL(triggered()),this,SLOT(sideBView()));
  sideAViewAct = viewBar->addAction(QPixmap(":/images/viewBar/sideA.png"),"Side view");
  connect(sideAViewAct,SIGNAL(triggered()),this,SLOT(sideAView()));
  perspectiveAct = viewBar->addAction(QPixmap(":/images/viewBar/perspective.png"),"Perspective view");
  connect(perspectiveAct,SIGNAL(triggered()),this,SLOT(perspective()));
  orthoAct = viewBar->addAction(QPixmap(":/images/viewBar/ortho.png"),"Orthographic view");
  connect(orthoAct,SIGNAL(triggered()),this,SLOT(ortho()));


// Tree toolbar
  treeBar = addToolBar("Tree Toolbar");
  //treeBar->setMaximumHeight(24);
  treeBar->setIconSize(QSize(24,24));

  positionT  = new QAction(QPixmap(":/images/treeBar/position.png"),"Display/Hide Tree position",0);
  treeBar->addAction(positionT);
  positionT->setEnabled(false);
  connect(positionT,SIGNAL(triggered()),this,SLOT(position_DisplayAll()));

  dbhthT  = new QAction(QPixmap(":/images/treeBar/dbhHT.png"),"Display/Hide DBH Hough Transform",0);
  treeBar->addAction(dbhthT);
  dbhthT->setEnabled(false);
  connect(dbhthT,SIGNAL(triggered()),this,SLOT(dbhHT_DisplayAll()));

  dbhlsrT  = new QAction(QPixmap(":/images/treeBar/dbhLSR.png"),"Display/Hide DBH Least Square Regression",0);
  treeBar->addAction(dbhlsrT);
  dbhlsrT->setEnabled(false);
  connect(dbhlsrT,SIGNAL(triggered()),this,SLOT(dbhLSR_DisplayAll()));

  heightT  = new QAction(QPixmap(":/images/treeBar/height.png"),"Display/Hide Tree height",0);
  treeBar->addAction(heightT);
  heightT->setEnabled(false);
  connect(heightT,SIGNAL(triggered()),this,SLOT(height_DisplayAll()));

  lengthT  = new QAction(QPixmap(":/images/treeBar/length.png"),"Display/Hide Tree length",0);
  treeBar->addAction(lengthT);
  lengthT->setEnabled(false);
  connect(lengthT,SIGNAL(triggered()),this,SLOT(length_DisplayAll()));

  stemCurveT  = new QAction(QPixmap(":/images/treeBar/stemCurve.png"),"Display/Hide tree skeleton",0);
  treeBar->addAction(stemCurveT);
  stemCurveT->setEnabled(false);
  connect(stemCurveT,SIGNAL(triggered()),this,SLOT(stemCurvature_DisplayAll()));

  convexT  = new QAction(QPixmap(":/images/treeBar/convex.png"),"Display/Hide convex projection",0);
  treeBar->addAction(convexT);
  convexT->setEnabled(false);
  connect(convexT,SIGNAL(triggered()),this,SLOT(convexhull_DisplayAll()));

  concaveT  = new QAction(QPixmap(":/images/treeBar/concave.png"),"Display/Hide concave projection",0);
  treeBar->addAction(concaveT);
  concaveT->setEnabled(false);
  connect(concaveT,SIGNAL(triggered()),this,SLOT(concavehull_DisplayAll()));

  skeletonT  = new QAction(QPixmap(":/images/treeBar/skeleton.png"),"Display/Hide tree QSM",0);
  treeBar->addAction(skeletonT);
  skeletonT->setEnabled(false);
  connect(skeletonT,SIGNAL(triggered()),this,SLOT(qsmDisplay_DisplayAll()));

    sortimentT  = new QAction(QPixmap(":/images/treeBar/sortiment.png"),"Display/Hide tree sortiments",0);
    treeBar->addAction(sortimentT);
    sortimentT->setEnabled(false);
    connect(sortimentT,SIGNAL(triggered()),this,SLOT(sortimentDisplay_DisplayAll()));

//CROWN BAR
  crownBar = addToolBar("Crown Toolbar");
  Projectbar->resize(24,144);
  crownBar->setIconSize(QSize(24,24));

  crownDisplayHideT = new QAction(QPixmap(":/images/crownBar/crown.png"),"Display/Hide Crown",0);
  crownBar->addAction(crownDisplayHideT);
  crownDisplayHideT->setEnabled(false);
  connect(crownDisplayHideT,SIGNAL(triggered()),this,SLOT(crown_DisplayAll()));

  crownHeightsDisplyHideT = new QAction(QPixmap(":/images/crownBar/heightCrown.png"),"Display/Hide Crown heights",0);
  crownBar->addAction(crownHeightsDisplyHideT);
  crownHeightsDisplyHideT->setEnabled(false);
  connect(crownHeightsDisplyHideT,SIGNAL(triggered()),this,SLOT(crownHeightsDisplayAll()));

  crownPositionDisplayHideT = new QAction(QPixmap(":/images/crownBar/crownPos.png"),"Display/Hide Crown position",0);
  crownBar->addAction(crownPositionDisplayHideT);
  crownPositionDisplayHideT->setEnabled(false);
  connect(crownPositionDisplayHideT,SIGNAL(triggered()),this,SLOT(crownPositionDisplayAll()));

  crownExternalPtsT = new QAction(QPixmap(":/images/crownBar/crownEP.png"),"Display/Hide Crown external points",0);
  crownBar->addAction(crownExternalPtsT);
  crownExternalPtsT->setEnabled(false);
  connect(crownExternalPtsT,SIGNAL(triggered()),this,SLOT(crownExternalPtsDisplayAll()));

  crownSurfaceBySectionsT = new QAction(QPixmap(":/images/crownBar/crownSections.png"),"Display/Hide Crown surface and volume by sections",0);
  crownBar->addAction(crownSurfaceBySectionsT);
  crownSurfaceBySectionsT->setEnabled(false);
  connect(crownSurfaceBySectionsT,SIGNAL(triggered()),this,SLOT(crownSurfaceBySectionsDisplayAll()));

  crownSurfaceBy3DHullT = new QAction(QPixmap(":/images/crownBar/crownConvex.png"),"Display/Hide Crown surface and volume by 3D Convexhull",0);
  crownBar->addAction(crownSurfaceBy3DHullT);
  crownSurfaceBy3DHullT->setEnabled(false);
  connect(crownSurfaceBy3DHullT,SIGNAL(triggered()),this,SLOT(crownSurface3DHullDisplayAll()));

  crownIntersectionsT = new QAction(QPixmap(":/images/crownBar/crownIntersection.png"),"Display/Hide Crowns intersections",0);
  crownBar->addAction(crownIntersectionsT);
  crownIntersectionsT->setEnabled(false);
  connect(crownIntersectionsT,SIGNAL(triggered()),this,SLOT(intersectionsShowAll()));

  crownIntersectionTableT = new QAction(QPixmap(":/images/crownBar/intersectionTable.png"),"Display intersections table",0);
  crownBar->addAction(crownIntersectionTableT);
  crownIntersectionTableT->setEnabled(false);
  connect(crownIntersectionTableT,SIGNAL(triggered()),this,SLOT(showCrownIntersectionsTable()));
}
void MainWindow::createTreeView()
{
  treeWidget->resizeColumnToContents(1);
}
void MainWindow::undo()
{
  if(!undopoint.empty())
  {
     // z posledniho prvku ve ktoru undopoint vzit cislo
    int num = undopoint.at(undopoint.size()-1);

    // num = pocet bodu od konce v m_cloudu1
    //vytvorit novy cloud s temito body
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    for(int y = m_cloud1->get_Cloud()->points.size() - num; y< m_cloud1->get_Cloud()->points.size(); y++)
    {
      pcl::PointXYZI bod;
      bod = m_cloud1->get_Cloud()->points.at(y);
      cloud->points.push_back(bod);
    }

    // kopirovat cloud k m_cloud
   // int s = m_cloud->get_Cloud()->points.size();
    *m_cloud->get_Cloud() += *cloud;

    // smazat poslednich "x" bodu z m_cloud1
    *m_cloud1->get_Cloud()->points.erase(m_cloud1->get_Cloud()->points.end() - num, m_cloud1->get_Cloud()->points.end());
    // smazat posledni zaznam v undopoint
    undopoint.pop_back();
    //zobrazit
    //m_vis->removeAllPointClouds();
    dispCloud(*m_cloud,220,220,0);
    cloud.reset();
  }
  else
  {
    QMessageBox::information(this,("empty"), ("no more points to put back"));

  return;
  }
}
//PROGRESSBAR
void MainWindow::showProgressBar100percent()
{
    createPBar();
    showPBarValue( 33);
    showPBarValue( 66);
    showPBarValue( 100);
    removePbar();
}
void MainWindow::showProgressBarAt(QProgressBar *pBar,int a)
{
    pBar->setValue(a);
    pBar->update();
    if(a == 100)
    {
      statusBar()->removeWidget(pBar);
    }
}
void MainWindow::showProgressBarInfinity()
{
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    statusBar()->addWidget(pBar);
    pBar->setMaximum(0);
    pBar->setMinimum(0);
    pBar->update();
   // statusBar()->removeWidget(pBar);
}

void MainWindow::createPBar()
{
    pBar = new QProgressBar(statusBar());
    pBar->setMaximumSize(200,16);
    pBar->setAlignment(Qt::AlignRight);
    statusBar()->addPermanentWidget(pBar);
    pBar->setValue(0);
    //return pBar;
}
void MainWindow::showPBarValue( int i)
{
  pBar->setValue(i);
  pBar->update();
}
void MainWindow:: removePbar()
{
  statusBar()->removeWidget(pBar);
}

//QVTKWIDGET
void MainWindow::AreaEvent(const pcl::visualization::AreaPickingEvent& event, void* )
{
    std::vector<std::string> clouds = event.getCloudNames();
    if(clouds.size()<1)
        return;
    
    auto name = (m_cloud->get_name().toStdString());
    auto indices = event.getPointsIndices(name);

    std::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);
    undopoint.push_back(indices.size());
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud (m_cloud->get_Cloud());
    extract.setIndices (indicesptr);
    extract.setNegative (false); // false = vse co je vybrano
    extract.filter (*cloud2);

    extract.setNegative (true); // true = vse co neni vybrano
    extract.filter (*cloud1);

    *m_cloud1->get_Cloud() += *cloud2;
    *m_cloud->get_Cloud() = *cloud1;
    cloud1.reset();
    cloud2.reset();
    indicesptr.reset();

  m_vis->removeAllPointClouds();
  
  for(int i=0; i< clouds.size(); i++)
  {
      std::string nameClouds = clouds.at(i);
    if(name == nameClouds)
      dispCloud(*m_cloud,220,220,0);
    else{
        QString qstr = QString::fromStdString(nameClouds);
        dispCloud(Proj->get_Cloud(qstr) );
    }
  }
  m_vis->getRenderWindow()->SetCurrentCursor( VTK_CURSOR_DEFAULT );
  return;
}
void MainWindow::AreaEvent2(const pcl::visualization::AreaPickingEvent& event, void* )
{
  pcl::PointIndices::Ptr inl (new pcl::PointIndices ());
  pcl::PointIndices::Ptr in (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI>);

  if (event.getPointsIndices (inl->indices))
  {
    for(int a = 0; a < inl->indices.size();a++)
    {
      if (inl->indices.at(a) < m_cloud->get_Cloud()->points.size())
          in->indices.push_back(inl->indices.at(a));
    }
    if(m_cloud->get_Cloud()->points.size() > 1)
    {
    //undopoint.push_back(inl->indices.size());
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud (m_cloud->get_Cloud());
      extract.setIndices (in);
      extract.setNegative (true); // true = vse co neni vybrano
      extract.filter (*cloud1);
    // vse co je vybrano patri  znicit
      extract.setNegative (false); //false = vse co je vybrano
      extract.filter (*cloud2);
    // vse co neni vybrano ulozit do m_cloud1
      m_cloud->set_Cloud(cloud1);
      m_vis->removeAllPointClouds();
      dispCloud(*m_cloud1,0,220,0); //reference
      dispCloud(*m_cloud,220,220,0); //original
    }
    return;
  }
  else
  {
    return;
  }
}
void MainWindow::pointEvent(const pcl::visualization::PointPickingEvent& event, void* )
{
  int idx = event.getPointIndex ();
  if (idx == -1)
    return;

  std::string name = event.getCloudName ();
  QString a = QString("Name of selected cloud: %1   points: %2").arg(QString::fromStdString(name)).arg(Proj->get_Cloud(QString::fromStdString(name)).get_Cloud()->points.size());
  statusBar()->showMessage(a);
  m_vis->setPointCloudSelected (true, name);
  return;
}
void MainWindow:: coordianteAxes()
{
  vtkSmartPointer<vtkRenderWindow> renderWindow = m_vis->getRenderWindow();
// axes coordinate system
  vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
  axes->SetTotalLength(1,1,1);
  axes->SetNormalizedShaftLength (1, 1, 1);
  vtkOrientationMarkerWidget* widget = vtkOrientationMarkerWidget::New();
  //widget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
  widget->SetOrientationMarker( axes );
  widget->SetInteractor( renderWindow->GetInteractor() );

  if(m_axes!=true)
  {
    widget->SetEnabled( 1 );
    m_axes=false;
  }
  else
  {
    widget->SetEnabled( 0 );
    m_axes=true;
  }
  widget->SetViewport( -0.0, -0.0, 0.15, 0.15 );
  widget->InteractiveOff();
}
//DISPLAY CLOUD
void MainWindow::dispCloud(Cloud cloud, QString field)
{
  //QColor col = cloud.get_color();
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud.get_Cloud(), field.toUtf8().constData());

  if(!m_vis->updatePointCloud<pcl::PointXYZI>(cloud.get_Cloud(), intensity_distribution, cloud.get_name().toUtf8().constData()))
    m_vis->addPointCloud<pcl::PointXYZI>(cloud.get_Cloud(), intensity_distribution, cloud.get_name().toUtf8().constData());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.get_Psize(), cloud.get_name().toUtf8().constData());


    m_vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::PCL_VISUALIZER_LUT_JET, cloud.get_name().toUtf8().constData());

    m_vis->updatePointCloud<pcl::PointXYZI>(cloud.get_Cloud(), intensity_distribution, cloud.get_name().toUtf8().constData());

    //    PCL_VISUALIZER_LUT_JET
    //    PCL_VISUALIZER_LUT_JET_INVERSE
    //    PCL_VISUALIZER_LUT_HSV
    //    PCL_VISUALIZER_LUT_HSV_INVERSE
    //    PCL_VISUALIZER_LUT_GREY
    //    PCL_VISUALIZER_LUT_BLUE2RED
    //    PCL_VISUALIZER_LUT_RANGE_AUTO
 // m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, true, cloud.get_name().toUtf8().constData());
  qvtkwidget->update();
}
void MainWindow::dispCloud(Cloud cloud)
{
  QColor col = cloud.get_color();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud.get_Cloud(),col.red(),col.green(),col.blue());
  if(!m_vis->updatePointCloud<pcl::PointXYZI>(cloud.get_Cloud(), color, cloud.get_name().toUtf8().constData()))
    m_vis->addPointCloud<pcl::PointXYZI>(cloud.get_Cloud(), color, cloud.get_name().toUtf8().constData());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.get_Psize(), cloud.get_name().toUtf8().constData());
 // m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, true, cloud.get_name().toUtf8().constData());
  qvtkwidget->update();
}

void MainWindow::dispCloud(Cloud cloud, int red, int green, int blue)
{
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color(cloud.get_Cloud(),red,green,blue);

  if(!m_vis->updatePointCloud<pcl::PointXYZI>(cloud.get_Cloud(),color, cloud.get_name().toUtf8().constData()))
    m_vis->addPointCloud<pcl::PointXYZI>(cloud.get_Cloud(),color, cloud.get_name().toUtf8().constData());
  m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.get_Psize(), cloud.get_name().toUtf8().constData());
 // m_vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, true, cloud.get_name().toUtf8().constData());
  qvtkwidget->update();
}
void MainWindow::removeCloud(QString name)
{
  m_vis->removePointCloud(name.toUtf8().constData());
  qvtkwidget->update();
}
//NAmes
QString MainWindow::get_path()
{
  return Proj->get_Path();
}
QStringList MainWindow::get_allNames()
{
  QStringList names;
  //basecloud
  for(int i = 0; i< Proj->get_sizebaseCV(); i++)
  {
    names << Proj->get_baseCloud(i).get_name();
  }
  //teren
  for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
  {
    names << Proj->get_TerrainCloud(i).get_name();
  }
  //vege
  for(int i = 0; i< Proj->get_sizevegeCV(); i++)
  {
    names << Proj->get_VegeCloud(i).get_name();
  }
  //tree
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }
  //ost
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
names.sort();
return names;
}
QStringList MainWindow::get_treeNames()
{
  QStringList names;
    //tree
  for(int i = 0; i< Proj->get_sizeTreeCV(); i++)
  {
    names << Proj->get_TreeCloud(i).get_name();
  }

names.sort();
return names;
}
QStringList MainWindow::get_featureNames()
{
  QStringList names;
    //tree
  for(int i = 0; i< Proj->getFeatureSize(); i++)
  {
      names << Proj->getFeature(i).get_name();
  }

names.sort();
return names;
}
QStringList MainWindow::get_terrainNames()
{
  QStringList names;
    //teren
  for(int i = 0; i< Proj->get_sizeTerainCV(); i++)
  {
    names << Proj->get_TerrainCloud(i).get_name();
  }
names.sort();
return names;
}
QStringList MainWindow::get_vegetationNames()
{
  QStringList names;
  //vege
  for(int i = 0; i< Proj->get_sizevegeCV(); i++)
  {
    names << Proj->get_VegeCloud(i).get_name();
  }
  names.sort();
  return names;
}
QStringList MainWindow::get_ostNames()
{
  QStringList names;
  //ost
  for(int i = 0; i< Proj->get_sizeostCV(); i++)
  {
    names << Proj->get_ostCloud(i).get_name();
  }
names.sort();
return names;
}
QStringList MainWindow::get_baseNames()
{
  QStringList names;
  //basecloud
  for(int i = 0; i< Proj->get_sizebaseCV(); i++)
  {
    names << Proj->get_baseCloud(i).get_name();
  }
names.sort();
return names;
}
QString MainWindow::name_Exists()
{
  QString newName = QInputDialog::getText(this, tr("Name of new Cloud file"),tr("Please enter name for new cloud (without spaces)"));
  if(newName.isEmpty())
  {
    QMessageBox::warning(this,tr("Error"),tr("You forgot to fill name of new tree cloud"));
    name_Exists();
  }
  else
  {
    QString path = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(newName);
    QFile file(path);
    if(file.exists())
    {
      //do you wish to rewrite existing file?
      QMessageBox::StandardButton rewrite = QMessageBox::question(this,tr("Overwrite file?"),tr("File with given name exist. Do you wish to overwrite file?"),QMessageBox::Yes|QMessageBox::No);
      if(rewrite == QMessageBox::Yes)
      {
        return newName;
      }
      else
      {
        return name_Exists();
      }
    }
    else
    {
      return newName;
    }
  }
}
//TREEWIDGET
void MainWindow::addTreeItem(QString name, bool visible)
{
  QTreeWidgetItem * item = new QTreeWidgetItem();
  item->setText(1,name);
  item->setFlags(item->flags() | Qt::ItemIsUserCheckable|Qt::ItemIsSelectable);
  if(visible ==true)
    item->setCheckState(0,Qt::Checked);
  else
    item->setCheckState(0,Qt::Unchecked);
  treeWidget->addTopLevelItem(item);
  treeWidget->resizeColumnToContents(1);
}
void MainWindow::colorCloud(QString name)
{

  QColor color = QColorDialog::getColor(Qt::white,this);
  if(color.isValid())
  {
    Proj->set_color(name, color);
    dispCloud(Proj->get_Cloud(name));
  }
}
void MainWindow::colorCloudField(QString name)
{
  QStringList l;
  l<<"x"<<"y"<<"z"<<"intensity";
  bool ok;
  QString field = QInputDialog::getItem(this,tr("select field"),tr("select field to color cloud:"),l,0,false,&ok);
  if (ok == false)
    return;
  dispCloud(Proj->get_Cloud(name),field);

}
void MainWindow::PointSize(QString name)
{
  int p = QInputDialog::getInt(this,("point size"),("please enter value in range 1 - 64 defining size of point"),1,1,64);

  Proj->set_PointSize(name, p);
  dispCloud(Proj->get_Cloud(name));
}
void MainWindow::deleteCloud(QString name)
{
  removeCloud(name);
  Proj->delete_Cloud(name);
}
void MainWindow::dispCloud(QString name)
{
  Cloud *c = new Cloud(Proj->get_Cloud(name));
  dispCloud(*c);
  delete c;
}

void MainWindow::displayHideEditCloud()
{
  if(m_editCloud == true)
  {
    removeCloud(m_cloud->get_name());
    m_editCloud = false;
  }
  else
  {
    dispCloud(*m_cloud,220,220,0);
    m_editCloud = true;
  }
}

void MainWindow::saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud)
{
  QString name;
  bool owrt = false;
  while(name.isEmpty() || (!name.isEmpty() && owrt==false))
  {
    name = QInputDialog::getText(this, tr("Name of new tree File"),tr("e.g. tree_id"));

    QString name2 =QString("%1.pcd").arg(name);
    QString path = QString("%1%2%3").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name2);
    QFile file(path);
    if(file.exists())
    {
      //do you wish to rewrite existing file?
      QMessageBox::StandardButton rewrite = QMessageBox::question(this,tr("Overwrite file?"),tr("File with given name exist. Do you wish to overwrite file?"),QMessageBox::Yes|QMessageBox::No);
      if(rewrite == QMessageBox::Yes)
      { owrt = true;}
      else{owrt = false;}
    }
    else {owrt = true;}
  }
  Cloud *c = new Cloud(tree_cloud,name);
  saveCloud(c, "strom");
  delete c;
}
void MainWindow::saveTreeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr tree_cloud, QString name, bool overwrt = false)
{
  if(overwrt== true)
  {
    Proj->save_Cloud(name,tree_cloud);
    //openTreeFile(pathT);
  }
  else
  {
    Cloud *c = new Cloud(tree_cloud,name);
    Proj->set_OstCloud(*c);
    QString pathT = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name);
    QFile file(pathT);

    if(file.exists())
    {
    //do you wish to rewrite existing file?
      QMessageBox::StandardButton rewrite = QMessageBox::question(this,tr("overwrite file?"),tr("File with given name exist. Do you wish to overwrite file?"),QMessageBox::Yes|QMessageBox::No);
      if(rewrite == QMessageBox::Yes)
      {
        Proj->save_newCloud("strom",name,tree_cloud);
        openCloudFile(pathT, "strom");
      }
      else
      {
        saveTreeCloud(tree_cloud);
      }
    }
    else
    {
      Proj->save_newCloud("strom",name,tree_cloud);
      openCloudFile(pathT, "strom");
    }
    delete c;
  }

}
void MainWindow::saveCloud (Cloud*s_cloud, QString type)
{
  if(s_cloud->get_Cloud()->points.size() > 1)
  {
    Proj->save_newCloud(type,s_cloud->get_name(),s_cloud->get_Cloud());
    QString path = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(s_cloud->get_name());
    if(type == "vege" )
      openCloudFile(path, "vege" );
    if(type == "cloud" )
      openCloudFile(path, "cloud");
    if(type == "strom" )
      openCloudFile(path, "strom");
    if(type == "teren" )
      openCloudFile(path, "teren" );
    if(type == "ost" )
      openCloudFile(path, "ost");
  }
}
void MainWindow::saveVegeCloud(Cloud *s_cloud, bool overwrt = true)
{
  if(s_cloud->get_Cloud()->points.size() > 1)
  {
    QString path = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(s_cloud->get_name());
    QFile file(path);
    if(overwrt == true && file.exists())
    {
      //delete exist file
      QFile::remove(path);
      // write file
      Proj->save_Cloud(s_cloud->get_name(),s_cloud->get_Cloud());
      // assing new cloud to the old one
      QString realname = QString("%1.pcd").arg(s_cloud->get_name());
      Proj->set_VegeCloud(realname, s_cloud->get_Cloud());
    }
    if(overwrt == true && !file.exists())
    {
      Proj->save_newCloud("vege",s_cloud->get_name(),s_cloud->get_Cloud());
      QString path = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(s_cloud->get_name());
      openCloudFile(path,"vege") ;
    }
    if(overwrt == false && !file.exists())
    {
      Proj->save_newCloud("vege",s_cloud->get_name(),s_cloud->get_Cloud());
      QString path = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(s_cloud->get_name());
      openCloudFile(path,"vege") ;
    }
    if(overwrt == false && file.exists())
    {
      QString name = QInputDialog::getText(this, tr("File exist"),tr("File exist in the project.\nPlease enter new name for the file."));
      Proj->save_newCloud("vege",name,s_cloud->get_Cloud());
      QString path = QString("%1%2%3.pcd").arg(Proj->get_Path()).arg(QDir::separator ()).arg(name);
      openCloudFile(path,"vege") ;
    }
  }
}

void MainWindow::saveVegetation(Cloud *c)
{
  saveCloud(c, "vege");
  emit savedVege();
}
void MainWindow::saveTerrain (Cloud *c)
{
  saveCloud(c, "teren");
  emit savedTerrain();
}
void MainWindow::saveTree(Cloud *c)
{
  saveCloud(c, "strom");
  emit savedTree();
}
void MainWindow::saveRest(Cloud *c)
{
  saveCloud(c, "ost");
  emit savedRest();
}
void MainWindow::saveFeature(Features *c){
    std::mt19937 gen(rd());
      std::uniform_int_distribution<> dis(0, 255);

      auto r= dis(gen);
      auto g= dis(gen);
      auto b= dis(gen);

    QColor col = QColor(r, g, b);
    c->set_color(col);
    Proj->setFeature(*c);
    addTreeItem(c->get_name(), true);
    dispCloud(c->get_name());
    emit savedFeature();
}
