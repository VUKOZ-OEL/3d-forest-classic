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
#ifndef GUI_H_INCLUDED
#define GUI_H_INCLUDED

#include <QtGui/QtGui>
#include <QtWidgets/QDialog>
#include <QtWidgets/QWizard>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QListWidgetItem>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>

#include <pcl/visualization/pcl_visualizer.h>
#include "cloud.h"

//INPUTDIALOG
//! Introduction dialog of methods.
/*! Dialog based class for providing introduction dialog about used methods with options.
    Dialog consist from description of method and optionally one or more input cloud selection and one or more output cloud names. */
class InputDialog :public QDialog
{
  Q_OBJECT
    
signals:
    void inputCloud_1( QString);
    void inputCloud_2( QString);
    void inputINT(int);

public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent  parent widget */
  InputDialog(QWidget *parent = 0 );
    //!  set size of dialog
    /*!  sets dialog size  \param w dialog width \param h dialog height */
  void DialogSize(int w, int h );
    //!  set  dialog layout
    /*!  sets default dialog layout of widgets */
  void DialogLayout();
    //!  set input cloud name
    /*!  set input cloud name from list of names.
        \param label label of edit line \param li list of cloud names */
  void set_inputCloud1(QString label, QStringList li);
  void set_inputList(QString label,QStringList li);
    //!  set second input cloud name
    /*!  set second input cloud name from list of names.
        \param label label of edit line \param li list of cloud names */
  void set_inputCloud2(QString label, QStringList li);
    //!  set third input cloud name
    /*!  set third input cloud name from list of names.
        \param label description of line \param li list of cloud names */
  void set_inputCloud3(QString label, QStringList li);
  //!  set output cloud name
    /*!  set output cloud name. On line is example of possible name
        \param label label of edit line \param example possible name of new cloud */
  void set_inputCheckBox(QString label);
    void set_inputCheckBox2(QString label);
    //!  set output cloud name
    /*!  set output cloud name. On line is example of possible name
        \param label label of edit line \param example possible name of new cloud */
  void set_outputCloud1(QString label, QString example);
    //!  set second output cloud name
    /*!  set second output cloud name. On line is example of possible name
        \param label label of edit line \param example possible name of new cloud */
  void set_outputCloud2(QString label, QString example);

  void set_outputPath(QString label, QString example, QString type);
    //!  Set input int value
    /*!  Set integer value
        \param label label of edit line \param example possible name of new cloud */
  void set_inputInt(QString label, QString example);
  //!  Set input int value
    /*!  Set integer value
        \param label label of edit line \param example possible name of new cloud */
  void set_inputInt2(QString label, QString example);
  void set_inputInt3(QString label, QString example);
  void set_inputInt4(QString label, QString example);
    void set_inputInt5(QString label, QString example);
    void set_inputInt6(QString label, QString example);
    void set_inputInt7(QString label, QString example);
    void set_inputInt8(QString label, QString example);
    void set_inputInt9(QString label, QString example);
    void set_inputInt10(QString label, QString example);
    //!  Set method description
    /*!  Set method overview, its inputs, example or description.
        \param text label of the method */
  void set_description(QString text);
    //!  set type of output cloud
    /*!  set type of cloud to saved as.
        \param label description of line \param li list of cloud names */
  void set_outputType(QString label, QStringList li);
  void set_outputType2(QString label, QStringList li);
    //!  Set method title
    /*!  Settitle of dialog.
        \param title label of the method */
  void set_title(QString title);
    //!  Stretch widgets
    /*!  Stretch displayed widgets to be in its place */
  void set_stretch();
    //!  Set path.
    /*!  set path of project */
  void set_path(QString path_m);

    //!  get name of input cloud.
    /*!  \return name of input cloud */
  QString get_inputCloud1();

    //!  get name of second input cloud.
    /*!  \return name of second input cloud */
  QString get_inputCloud2();

    //!  get name of third input cloud.
    /*!  \return name of third input cloud */
  QString get_inputCloud3();
    //!  get name of output cloud.
    /*!  \return name of output cloud */
  QString get_outputCloud1();
    //!  get name of  second output cloud.
    /*!  \return name of second output cloud */
  QString get_outputCloud2();

  QString get_outputPath();
  QList<QString> get_inputList();
    //!  get type of output cloud.
    /*!  \return name QString of cloud type*/
  QString get_outputType();
  QString get_outputType2();
    //!  get int value .
    /*!  \return integer value entered by user */
  int get_intValue();

  //!  get int value .
    /*!  \return integer value entered by user */
  int get_intValue2();
  //!  get int value .
    /*!  \return integer value entered by user */
  int get_intValue3();
  int get_intValue4();
    int get_intValue5();
    int get_intValue6();
    int get_intValue7();
    int get_intValue8();
    int get_intValue9();
    int get_intValue10();
    //!  get type of output cloud.
    /*!  \return true if checkbox is seleted*/
    void setSboxDouble(QString label, float dValue=1.5, float minV=0, float maxV=10);
    float getSboxValue();
  bool get_CheckBox();
    bool get_CheckBox2();
   void set_outputDir(QString label, QString example);
  QString get_outputDir();

private slots:
    //!  when presed button ok .
    /*!  set variables when user press OK button */
  void ok();
  //!  Validate user input .
    /*!  Controls if all inputs are in right format \param text input changed text */
  void validate(QString);
  //!  Validate user input as a integer .
    /*!  Controls if user enters only digit and no other symbols.
     \param text input integer value */
  void validateInt(QString );
    //!  Validate user input on output .
    /*!  Controls if user enters only symbols without spaces.
     \param text input integer valule */
  void validateOutput1(QString);
  //!  Validate user input on second output line.
    /*!  Controls if user enters only symbols without spaces.
     \param text input integer valule */
  void validateOutput2(QString);
  void saveNewFile();
  void saveIntoDir();


public slots:
  void getINT();
  void getinputCloud1();
  void getinputCloud2();

private:
  QString input_cloud1;           /**< Input cloud name */
  QString input_cloud2;           /**< Second input cloud name */
  QString input_cloud3;           /**< Third input cloud name */
  QString output_cloud1;          /**< Output cloud name */
  QString output_cloud2;          /**< Second output cloud name */
  QString output_type;            /**< Second output cloud name */
    QString output_type2;            /**< Second output cloud name */
  QString output_path;            /**< output path file */
  QString output_dir;            /**< output path file */
  QList<QListWidgetItem*> inputList;

  float float_value1;             /**< input float value */
  float float_value2;             /**< input float value 2 */
  int int_value1;                 /**< input integer value */
  int int_value2;                 /**< input second integer value */
  int int_value3;                 /**< input second integer value */
  int int_value4;                 /**< input second integer value */
    int int_value5;                 /**< input second integer value */
    int int_value6;                 /**< input second integer value */
    int int_value7;                 /**< input second integer value */
    int int_value8;                 /**< input second integer value */
    int int_value9;                 /**< input second integer value */
    int int_value10;                 /**< input second integer value */
  QListWidget * listWidget;
  bool isList;



  QString path;                   /**< Project path */
  QComboBox * inputCloud1;        /**< List of input cloud names */
  bool isIC1;                     /**< bool value if input cloud is in proper format */
  QComboBox * inputCloud2;        /**< List of second input cloud names */
  bool isIC2;                     /**< bool value if second input cloud is in proper format */
  QComboBox * inputCloud3;        /**< List of third input cloud names */
  bool isIC3;                     /**< bool value if third input cloud is in proper format */
  QLineEdit *outputCloud1;        /**< output cloud name */
  bool output1Bool;               /**< bool value if output cloud name is empty */
  bool isOC1;                     /**< bool value if output cloud name is in proper format */
  QLineEdit *outputCloud2;        /**< output cloud name */
  bool isOC2;                     /**< bool value if output cloud name is in proper format */
  bool output2Bool;               /**< bool value if output cloud name is empty */
  QLineEdit *outputPath;        /**< output cloud name */
  QLineEdit *outputDir;        /**< output cloud name */
  bool isDir;
  QPushButton *directoryButton;   /**< Button for selecting directory for project */
  bool isPath;                     /**< bool value if output cloud name is in proper format */
  bool outputPathBool;               /**< bool value if output cloud name is empty */
  QString m_type;                 /**<File type */
  QLineEdit *intInput;            /**< input integer value */
  bool intInputBool;              /**< bool value if output cloud name is empty */
  bool isII1;                     /**< bool value if output cloud name is in proper format  */
  QLineEdit *intInput2;            /**< input integer value */
  bool intInputBool2;              /**< bool value if output cloud name is empty */
  bool isII2;                     /**< bool value if output cloud name is in proper format  */
  QLineEdit *intInput3;            /**< input integer value */
  bool intInputBool3;              /**< bool value if output cloud name is empty */
  bool isII3;                     /**< bool value if output cloud name is in proper format  */
  QLineEdit *intInput4;            /**< input integer value */
  bool intInputBool4;              /**< bool value if output cloud name is empty */
  bool isII4;                     /**< bool value if output cloud name is in proper format  */
    QLineEdit *intInput5;            /**< input integer value */
    bool intInputBool5;              /**< bool value if output cloud name is empty */
    bool isII5;                     /**< bool value if output cloud name is in proper format  */
    QLineEdit *intInput6;            /**< input integer value */
    bool intInputBool6;              /**< bool value if output cloud name is empty */
    bool isII6;                     /**< bool value if output cloud name is in proper format  */
    QLineEdit *intInput7;            /**< input integer value */
    bool intInputBool7;              /**< bool value if output cloud name is empty */
    bool isII7;                     /**< bool value if output cloud name is in proper format  */
    QLineEdit *intInput8;            /**< input integer value */
    bool intInputBool8;              /**< bool value if output cloud name is empty */
    bool isII8;                     /**< bool value if output cloud name is in proper format  */
    QLineEdit *intInput9;            /**< input integer value */
    bool intInputBool9;              /**< bool value if output cloud name is empty */
    bool isII9;                     /**< bool value if output cloud name is in proper format  */
    QLineEdit *intInput10;            /**< input integer value */
    bool intInputBool10;              /**< bool value if output cloud name is empty */
    bool isII10;                     /**< bool value if output cloud name is in proper format  */
    QComboBox * outputType;          /**< List of output cloud types*/
    bool isType;
    QComboBox * outputType2;          /**< List of output cloud types*/
    bool isType2;
    bool isICHB;
    bool CHB;
    QCheckBox * CHBox;
    
    bool isICHB2;
    bool CHB2;
    QCheckBox * CHBox2;
    
    QDoubleSpinBox *sbox;
    bool isSBOX;
    float sboxvalue;

  QDialogButtonBox *buttonBox;    /**< default buttons */
  QHBoxLayout *buttontLayout;     /**< default buttons layout */
  QVBoxLayout *InputLayout;       /**< default layout of input lines */
  QHBoxLayout *inputareaLayout;   /**< default layout of input area */
  QVBoxLayout *mainLayout;        /**< default layout */
  QHBoxLayout *outputPathLayout;  /**< default layout for path selection */
};
//EXPORT ATTRIBUTES
//! Dialog for exporting tree atributes.
/*! Dialog based class for providing export dialog for selected trees and attributes.
    Dialog consist from selection of trees, parameter selection and separator of comlun,. Dialog saves result into desired text file */
class ExportAttr : public QDialog
{
  Q_OBJECT

public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent  parent widget */
  ExportAttr(QWidget *parent = 0 );
  //!  Default contructor
    /*!  Constructor of dialog  \param parent  parent widget \param nameList tree list of names */
  ExportAttr( QStringList nameList, QWidget *parent = 0 );
    //!  Set size of dialog
    /*!  sets dialog size  \param w dialog width \param h dialog height */
  void DialogSize(int w, int h );
    //!  Set  dialog layout
    /*!  sets default dialog layout of widgets */
  void DialogLayout();

    //!  Set output file name.
    /*!  set output cloud name. On line is example of possible name
        \param label label of edit line \param example possible name of new cloud */
  void set_outputCloud1(QString label, QString example);
    //!  Set dialog description .
    /*!  Displayed description of function.
     \param text explanation */
  void set_description(QString text);
    //!  Set tree for attributes export.
    /*!  Sets list of all tree names into combobox for selection.
     \param li list of all trees*/
  void set_trees(QStringList li);
  void set_list(QStringList li);
  QList<QString> get_inputList();
    //!  Get tree name.
    /*!  return selected tree name.
     \return name of selected tree*/
  QString get_treeName();
    //!  Get separator.
    /*!  return separator of fields in resulting file.
     \return QString of separator*/
  QString get_separator();
    //!  Get output file.
    /*!  return the path to the new file containng results.
     \return QString file path*/
  QString get_outputFile();
    //!  True if DBH_HT is selected.
    /*! \return bool */
  bool get_DBH_HT();
    //!  True if DBH_LSR is selected.
    /*! \return bool */
  bool get_DBH_LSR();
    //!  True if Position is selected.
    /*! \return bool */
  bool get_Position();
    //!  True if Height is selected.
    /*! \return bool */
  bool get_Height();
    //!  True if Length is selected.
    /*! \return bool */
  bool get_Length();
    //!  True if Points is selected.
    /*! \return bool */
  bool get_Points();
    //!  True if compute area of convex planar projection is selected.
    /*! \return bool */
  bool get_areaconvex();
  //!  True if compute area of concave planar projection is selected.
    /*! \return bool */
  bool get_areaconcave();
    bool get_volume();
    bool get_sortiments();


private slots:
    //!  Called when directory button pressed
    /*!  Choose existing directory  for proejct creation*/
  void setExistingDirectory();
    //!  when pressed button ok .
    /*!  set variables when user press OK button */
  void ok();
    //!  Validate user input .
    /*!  Controls if all inputs are in right format \param text input changed text */
  void validate(QString);
    //!  Slot if other separator is selected.
    /*!  Controls field other separator is selected. \param checked if other separator field is checked */
  void other_Separator(bool checked);

  void all_attributes(int checked);

  void all_attr(int checked);

private:
  QListWidget * listWidget;
  QList<QListWidgetItem*> inputList;
  QLabel *treeLabel;              /**< Description of tree selection combobox*/
  QComboBox * inputTrees;         /**< List of input tree names */
  QLabel *fileLabel;              /**< Desription of QlineEdit for output file */
  QLineEdit *outputFile;          /**< Output file name */
  QPushButton *directoryButton;   /**< Button for selecting directory for project */
  QDialogButtonBox *buttonBox;    /**< Default buttons */
  QHBoxLayout *buttontLayout;     /**< horizontal layout */
  QVBoxLayout *InputLayout;       /**< Default layout of input lines */
  QHBoxLayout *inputareaLayout;   /**< Default layout of input area */
  QVBoxLayout *mainLayout;        /**< Default layout */
  QVBoxLayout *treeLayout;        /**< Default layout */
  QGridLayout *fileLayout;        /**< Default layout of check boxesn and attributes*/
  QRadioButton *radio1;           /**< separator button - semicolon*/
  QRadioButton *radio2;           /**< separator button - space*/
  QRadioButton *radio3;           /**< separator button - tabulator*/
  QRadioButton *radio4;           /**< separator button - other*/
  QLineEdit *sep;                 /**< optional separator */
  QCheckBox *CHB_DBH_HT;          /**< checkbox for DBH HT*/
  QCheckBox *CHB_DBH_LSR;         /**< checkbox for DBH LSR*/
  QCheckBox *CHB_height;          /**< checkbox for Height*/
  QCheckBox *CHB_length;          /**< checkbox for Length*/
  QCheckBox *CHB_position;        /**< checkbox for Position*/
  QCheckBox *CHB_areavex;         /**< checkbox for area of convex planar projection*/
  QCheckBox *CHB_areacave;        /**< checkbox for area of concave planar projection*/
  QCheckBox *CHB_points;          /**< checkbox for point number*/
    QCheckBox *CHB_volume;        /**< checkbox for Position*/
    QCheckBox *CHB_sortiments;
  QCheckBox *CHB_all;             /**< checkbox for all attributes*/

  bool DBH_HT;                    /**< true if checkbox for DBH HT is selected*/
  bool DBH_LSR;                   /**< true if checkbox for DBH LSR is selected*/
  bool Position;                  /**< true if checkbox for Position is selected*/
  bool Height;                    /**< true if checkbox for Height is selected*/
  bool Length;                    /**< true if checkbox for Lengthis selected*/
  bool Points;                    /**< true if checkbox for point number is selected*/
  bool Areaconvex;                /**< true if checkbox for area of convex planar projection is selected*/
  bool Areaconcave;               /**< true if checkbox for area of concave planar projection is selected*/
    bool qsmVolume;
    bool sortiments;

    //!  Group of tree attributes checkboxes.
    /*!  \return group of attributes checkboxes in defined layout*/
  QGroupBox *attributesGroup();
    //!  Group of separator checkboxes.
    /*!  \return group of separator widgets in defined layout*/
  QGroupBox *separatorGroup();
  QString m_separator;            /**< QString containg selected separator*/

};
//PROEJCT WIZARD
//!  Project manager for creating and importing projects.
/*!  Wizard based dialog for creating new projects or importing old ones from another path to new place. */
class ProjImport :public QWizard
{
  Q_OBJECT

public:
    //! An enum of pages.
    /*! enumerated pages of project wizard.  */
  enum  { Page_Intro,       /*!< Introduction page */
          Page_newProject,  /*!< New project  page */
          Page_transform,   /*!< Transformation matrix page */
          Page_import,      /*!< Import page */
          Page_final        /*!< Final page */
        };
    //!  Default contructor
    /*!  Constructor of dialog  \param parent parent widget */
  ProjImport(QWidget *parent = 0);
};
//!  Introduction page of project wizard.
/*!  Introduction page  for decision if user want to create new project or import old one. */
class IntroPage : public QWizardPage
{
     Q_OBJECT

public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent parent widget */
  IntroPage(QWidget *parent = 0);
    //!  called when ok button is pressed
    /*!  decide what user want and select next page of wizard */
  int nextId() const;

private:
  QLabel *topLabel;                    /**< Main label of page */
  QRadioButton *newProjectButton;      /**< Check button for creating new project */
  QRadioButton *importProjectButton;   /**< Check button for importing old project  */
};
//!  New project page of project wizard.
/*!  Page for definition of new project name, and path to the project */
class NewProjectPage : public QWizardPage
{
     Q_OBJECT

public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent parent widget */
  NewProjectPage(QWidget *parent = 0);
    //!  called when ok button is pressed
    /*!  decide what user want and select next page of wizard */
  int nextId() const;

private:
  QLabel *topLabel;               /**< Main label of page */
  QLabel *nameLabel;              /**< Label of line for entering new project name */
  QLineEdit *nameEdit;            /**< Line for entering new project name */
  QLabel *pathLabel;              /**< Label of line for entering new project path */
  QLineEdit *pathEdit;            /**< Line for entering new project path */
  QPushButton *directoryButton;   /**< Button for selecting directory for project */
  QString directory;              /**< selected directory path */
private slots:
    //!  Called when directory button pressed
    /*!  Choose existing directory  for proejct creation*/
  void setExistingDirectory();
    //!  Check new project name
    /*!  Check if entered name is in proper format \param name new project name */
  void nameCheck(QString name);
    //!  Check new project path
    /*!  Check if in selected directory do not exist filder with the same name as project name \param name new project name */
  void pathCheck(QString name);
};
//!  Transform page of project wizard.
/*!  Page for definition of new projection or selection of old one */
class TransformPage : public QWizardPage
{
     Q_OBJECT

public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent parent widget */
  TransformPage(QWidget *parent = 0);
    //!  called when ok button is pressed
    /*!  decide what user want and select next page of wizard */
  int nextId() const;

private:
  QLabel *topLabel;             /**< Main label of page */
  QRadioButton *selectButton;   /**< Button for selecting if use old one */
  QLabel *nameLabel;            /**< Label of line with projections */
  QComboBox *transformList;     /**< Line with projections */

  QRadioButton *newButton;      /**< Button for selecting if use new projection  */
  QLabel *projLabel;            /**< Label of line with new projection name */
  QLineEdit *projEdit;          /**< Line with new projection name */
  QLabel *projXLabel;           /**< Label of line with new projection  X value */
  QLineEdit *projXEdit;         /**< Line with new projection X value */
  QLabel *projYLabel;           /**< Label of line with new projection  Y value */
  QLineEdit *projYEdit;         /**< Line with new projection Y value */
  QLabel *projZLabel;           /**< Label of line with new projection  Z value */
  QLineEdit *projZEdit;         /**< Line with new projection Z value */


  QStringList names;            /**< List of old projection names */
  QStringList x;                /**< List of old projection X value */
  QStringList y;                /**< List of old projection Y value */
  QStringList z;                /**< List of old projection Z value */
private slots:
    //!  Open file with old projections and load them into lists
    /*!  Open Projections file and load saved proejction onto list of coordinates */
  void openProjects();
    //!  Change active lines
    /*!  Change active lines to line of selecting projection */
  void selectProj(bool check);
    //!  Change active lines
    /*!  Change active lines to lines for definition of new projection */
  void newProj(bool check);
    //!  Change lines examples to selected projection
    /*!  Change lines of x, y, z examples */
  void selected(int index);
    //!  Check new projection name
    /*!  Change check if new name of proejction do not contains spaces or is not the same as used. */
  void nameCheck(QString);
    //!  Check X value
    /*!  Check entered X value to be number only */
  void numberXCheck(QString);
    //!  Check Y value
    /*!  Check entered Y value to be number only */
  void numberYCheck(QString);
    //!  Check Z value
    /*!  Check entered Z value to be number only */
  void numberZCheck(QString);
};
//!  Import page of project wizard.
/*!  Page for selecting old projection and move it to the new path */
class ImportPage : public QWizardPage
{
     Q_OBJECT
public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent parent widget */
  ImportPage(QWidget *parent = 0);
    //!  called when ok button is pressed
    /*!  decide what user want and select next page of wizard */
  int nextId() const;

private:
  QLabel *topLabel;               /**< Main label of page */
  QLabel *oldLabel;               /**< Label of line with old path to project */
  QLineEdit *oldpathEdit ;        /**< Line with old path to project */
  QPushButton *oldButton;         /**< Button for selecting pro file of old project */
  QLabel *projLabel;              /**< Label of line with new project name */
  QLineEdit *projEdit;            /**< Line with new project name */
  QLabel *newLabel;               /**< Label of line with new project path */
  QLineEdit *newpathEdit ;        /**< Line with new project path */
  QPushButton *newButton;         /**< Button for selecting directory for saving new project */
  QCheckBox  *removeCheckBox;     /**< Check box for deleting old project */

private slots:
    //!  Set new directory
    /*!  Set new project directory path */
  void setNewDirectory();
    //!  Set old projection file
    /*!  Set old projectionfile and path to it */
  void setOldFile();
    //! Check new projection name
    /*!  Check new projection name to be in proper format \param name new project name */
  void nameCheck(QString name);
    //! Check new projection path
    /*!  Check if in selected directory do not exist filder with the same name as project name \param name new project name */
  void pathCheck(QString name);

};
//!  Final page of project wizard.
/*!  Page for final conclusion of project wizard */
class FinalPage : public QWizardPage
{
     Q_OBJECT

public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent parent widget */
  FinalPage(QWidget *parent = 0);
    //!  called when ok button is pressed
    /*!  decide what user want and select next page of wizard */
  int nextId() const;

private:
  QLabel *topLabel;  /**< Main label of page */
};

//TREE WIDGET
//!  Tree widget for list of clouds in project.
/*!  Tree widget with list of clouds presented in project. Throu context menu and check boxes user can control single clouds to display, delete or color.  */
class MyTree :public QTreeWidget
{
  Q_OBJECT
public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent parent widget */
  MyTree(QWidget *parent = 0);
    //!  Remove all items
    /*!  Remove all items from tree */
  void cleanAll();
    //!  Remove item
    /*!  Remove selected item from tree \param name name of the item */
  void itemdelete(QString name);
    //!  Remove all items from m_vis
    /*!  Remove all items from m_vis */
  void allItemOFF();
    //!  Display item
    /*!  Display given item \param name name of the item */
  void itemON(QString);

private slots:
    //!  Show context menu
    /*!  Display context menu on if on given point exist tree item \param pos position of mouse */
  void showContextMenu(const QPoint &pos);
    //!  If check box is changed
    /*!  If check box changed state, this method emit signal fr change display \param item name of the item */
  void onItemChange(QTreeWidgetItem *item,int i);
    //!  Delete selected item
    /*!  Delete selected item from cloud and from project \param name name of the item */
  void onDeleteItem(QString name);
    //!  color selected item
    /*!  Display selected cloud in selected color. \param name name of the item */
  void onColor(QString name);
    //!  color selected item
    /*!  Display selected cloud in selected color according to values of point field. \param name name of the item */
   void onColorField(QString name);
    //!  Change point size of cloud
    /*!  Change point size of cloud. \param name name of the item */
  void onPsize(QString name);
    //!  All items check on
    /*!  All items check on and emit signal to display all clouds. */
  void allON();
    //!  All items check off
    /*!  All items check off and emit signal to remove all clouds. */
  void allOFF();


signals:
    //!  Emited  when checkbox changed state to on
  void checkedON(QString);
    //!  Emited  when checkbox changed state to off
  void checkedOFF(QString);
    //!  Emited  when selected delete of item
  void deleteItem(QString);
    //!  Emited  when selected color menu
  void colorItem(QString);
    //!  Emited  when selected color field
  void colorItemField(QString);
    //!  Emited  when selected pointsize change
  void psize(QString);

private:
  QString name; /**< name of the cloud */
};

//THREAD




//VISUALIZER
//!  Visualizer widget for displaying clouds, lines, points, etc.
/*!  Central widget for displaing clouds. and other stuff.  */
class Visualizer : public pcl::visualization::PCLVisualizer
{
  vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget_;
public:
    //!  Default contructor
    /*!  Constructor of VTKWidget  */
  Visualizer();
  void coordinateMark();
};

//class Vis
//{
//  vtkSmartPointer<vtkRenderWindow> m_renderWindow;
//  vtkSmartPointer<vtkRenderer> m_renderer;
//public:
//    //!  Default contructor
//    /*!  Constructor of dialog   */
//  Vis();
//  vtkSmartPointer<vtkRenderWindow> get_renderWindow();
//  bool addCloud(Cloud cl);
//  void addCylinder();
//};

//EXPORT ATTRIBUTES
//! Dialog for exporting crown atributes.
/*! Dialog based class for providing export dialog for selected crown and attributes.
    Dialog consist from selection of tree,crown parameter selection and separator of comlun,. Dialog saves result into desired text file */
class ExportCrownAttr : public QDialog
{
  Q_OBJECT

public:
    //!  Default contructor
    /*!  Constructor of dialog  \param parent  parent widget */
  ExportCrownAttr(QWidget *parent = 0 );
  //!  Default contructor
    /*!  Constructor of dialog  \param parent  parent widget \param nameList tree list of names */
  ExportCrownAttr( QStringList nameList, QWidget *parent = 0 );
    //!  Set size of dialog
    /*!  sets dialog size  \param w dialog width \param h dialog height */
  void DialogSize(int w, int h );
    //!  Set  dialog layout
    /*!  sets default dialog layout of widgets */
  void DialogLayout();
    //!  Set output file name.
    /*!  set output cloud name. On line is example of possible name
        \param label label of edit line \param example possible name of new cloud */
  void set_outputCloud1(QString label, QString example);
    //!  Set dialog description .
    /*!  Displayed description of function.
     \param text explanation */
  void set_description(QString text);
    //!  Set tree for attributes export.
    /*!  Sets list of all tree names into combobox for selection.
     \param li list of all trees*/
  void set_trees(QStringList li);
    //!  Get tree name.
    /*!  return selected tree name.
     \return name of selected tree*/
  QString get_treeName();
    //!  Get separator.
    /*!  return separator of fields in resulting file.
     \return QString of separator*/
  QString get_separator();
    //!  Get output file.
    /*!  return the path to the new file containng results.
     \return QString file path*/
  QString get_outputFile();
    //!  True if DBH_HT is selected.
    /*! \return bool */
  bool getPoints();
    //!  True if DBH_LSR is selected.
    /*! \return bool */
  bool getHeight();
    //!  True if Position is selected.
    /*! \return bool */
  bool getBottomHeight();
    //!  True if Height is selected.
    /*! \return bool */
  bool getTotalHeight();
    //!  True if Length is selected.
    /*! \return bool */
  bool getPositionDeviance();
    //!  True if Points is selected.
    /*! \return bool */
  bool getPositionXYZ();
    //!  True if compute area of convex planar projection is selected.
    /*! \return bool */
  bool getLength();
  //!  True if compute area of concave planar projection is selected.
    /*! \return bool */
  bool getWidth();
  bool getVolVoxels();
  bool getVolSections();
  bool getSurface();
  bool getVol3DCH();
  bool getSurf3DCH();
  bool getSectionHeight();
  bool getThresholdDistance();
  void set_list(QStringList li);
  QList<QString> get_inputList();

private slots:
    //!  Called when directory button pressed
    /*!  Choose existing directory  for proejct creation*/
  void setExistingDirectory();
    //!  when pressed button ok .
    /*!  set variables when user press OK button */
  void ok();
    //!  Validate user input .
    /*!  Controls if all inputs are in right format \param text input changed text */
  void validate(QString);
    //!  Slot if other separator is selected.
    /*!  Controls field other separator is selected. \param checked if other separator field is checked */
  void other_Separator(bool checked);

  void all_attributes(int checked);

  void all_attr(int checked);

private:
  QListWidget * listWidget;
  QList<QListWidgetItem*> inputList;
  QLabel *treeLabel;              /**< Description of tree selection combobox*/
  QComboBox * inputTrees;         /**< List of input tree names */
  QLabel *fileLabel;              /**< Desription of QlineEdit for output file */
  QLineEdit *outputFile;          /**< Output file name */
  QPushButton *directoryButton;   /**< Button for selecting directory for project */
  QDialogButtonBox *buttonBox;    /**< Default buttons */
  QHBoxLayout *buttontLayout;     /**< horizontal layout */
  QVBoxLayout *InputLayout;       /**< Default layout of input lines */
  QHBoxLayout *inputareaLayout;   /**< Default layout of input area */
  QVBoxLayout *mainLayout;        /**< Default layout */
  QVBoxLayout *treeLayout;        /**< Default layout */
  QGridLayout *fileLayout;        /**< Default layout of check boxesn and attributes*/
  QRadioButton *radio1;           /**< separator button - semicolon*/
  QRadioButton *radio2;           /**< separator button - space*/
  QRadioButton *radio3;           /**< separator button - tabulator*/
  QRadioButton *radio4;           /**< separator button - other*/
  QLineEdit *sep;                 /**< optional separator */
  QCheckBox *CHB_height;          /**< checkbox for DBH HT*/
  QCheckBox *CHB_bottomHeight;         /**< checkbox for DBH LSR*/
  QCheckBox *CHB_totalHeight;          /**< checkbox for Height*/
  QCheckBox *CHB_length;          /**< checkbox for Length*/
  QCheckBox *CHB_width;        /**< checkbox for Position*/
  QCheckBox *CHB_positionDev;         /**< checkbox for area of convex planar projection*/
  QCheckBox *CHB_positionXYZ;        /**< checkbox for area of concave planar projection*/
  QCheckBox *CHB_points;          /**< checkbox for point number*/
  QCheckBox *CHB_volVoxels;
  QCheckBox *CHB_volSections;
  QCheckBox *CHB_vol3DCH;
  QCheckBox *CHB_surface;
  QCheckBox *CHB_surface3DCH;
  QCheckBox *CHB_sectionHeight;
  QCheckBox *CHB_thresholdDist;
  QCheckBox *CHB_all;             /**< checkbox for all attributes*/

  bool points;                    /**< true if checkbox for DBH HT is selected*/
  bool height;                   /**< true if checkbox for DBH LSR is selected*/
  bool bottomHeight;
  bool totalHeight;
  bool positionDev;                  /**< true if checkbox for Position is selected*/
  bool positionXYZ;
  bool width;                    /**< true if checkbox for Height is selected*/
  bool length;                    /**< true if checkbox for Lengthis selected*/
  bool volVoxels;                    /**< true if checkbox for point number is selected*/
  bool volSections;                /**< true if checkbox for area of convex planar projection is selected*/
  bool vol3DCH;               /**< true if checkbox for area of concave planar projection is selected*/
  bool surface;
  bool surf3DCH;
  bool thresholdDist;
  bool sectionHeight;


    //!  Group of tree attributes checkboxes.
    /*!  \return group of attributes checkboxes in defined layout*/
  QGroupBox *attributesGroup();
    //!  Group of separator checkboxes.
    /*!  \return group of separator widgets in defined layout*/
  QGroupBox *separatorGroup();
  QString m_separator;            /**< QString containg selected separator*/

};

class ExportQSMDialog : public QDialog
{
    Q_OBJECT
    
public:
    
    ExportQSMDialog(QWidget *parent = 0 );
    ExportQSMDialog( QStringList nameList, QWidget *parent = 0 );
    ~ExportQSMDialog();
    
public slots:
    
    
    //sets
    void setDescription(QString text);
    void setList(QStringList li);
    void setExistingDirectory();
    void setPrefix(QString label, QString example);
    //gets
    QList<QString> getInputList();
    QString getSeparator();
    bool getTrees();
    bool getBranches();
    bool getSortiments();
    bool getProfile();
    QString getPath();
    QString getPrefix();
    
    
    void ok();
    
    void otherSeparator(bool checked);
    void allFiles(int checked);
    void allAttr(int checked);
    void allAttributes(int checked);
    
    
private slots:
    
    void DialogSize(int w, int h );
    void DialogLayout();
    void validate(QString);
    QGroupBox *attributesGroup();
    QGroupBox *separatorGroup();
    
private:
    
    QListWidget * listWidget;
    QList<QListWidgetItem*> inputList;
    QLabel *treeLabel;              /**< Description of tree selection combobox*/
    QComboBox * inputTrees;         /**< List of input tree names */
    QLineEdit *prefixLine;
    
    QLabel *fileLabel;              /**< Desription of QlineEdit for output file */
    QLineEdit *outputFile;          /**< Output file name */
    QPushButton *directoryButton;   /**< Button for selecting directory for project */
    QDialogButtonBox *buttonBox;    /**< Default buttons */
    QHBoxLayout *buttontLayout;     /**< horizontal layout */
    QVBoxLayout *InputLayout;       /**< Default layout of input lines */
    QHBoxLayout *inputareaLayout;   /**< Default layout of input area */
    QVBoxLayout *mainLayout;        /**< Default layout */
    QVBoxLayout *treeLayout;        /**< Default layout */
    QGridLayout *fileLayout;        /**< Default layout of check boxesn and attributes*/
    
    QRadioButton *radio1;           /**< separator button - semicolon*/
    QRadioButton *radio2;           /**< separator button - space*/
    QRadioButton *radio3;           /**< separator button - tabulator*/
    QRadioButton *radio4;           /**< separator button - other*/
    QLineEdit *sep;                 /**< optional separator */
    
    QString m_separator;            /**< QString containg selected separator*/
    QString m_prefix;               /**< QString containg selected file prefix*/
    
    QCheckBox *CHB_sortiments;
    QCheckBox *CHB_profile;
    QCheckBox *CHB_trees;
    QCheckBox *CHB_branches;
    QCheckBox *CHB_all;
    
    bool m_sortiments=false;        // if file with segment profile should be created
    bool m_profile=false;           // true if file with segment profiles should be created
    bool m_trees=false;             // if file with tree volume
    bool m_branches=false;          // if file with stem and branches
};

class ExportFeaturesDialog : public QDialog
{
    Q_OBJECT
    
public:
    
    ExportFeaturesDialog(QWidget *parent = 0 );
    ExportFeaturesDialog( QStringList nameList, QWidget *parent = 0 );
    ~ExportFeaturesDialog();
    
public slots:
    
    //sets
    void setDescription(QString text);
    void setList(QStringList li);
    void setExistingDirectory();
    void setPrefix(QString label, QString example);
    //gets
    QList<QString> getInputList();
    QString getSeparator();
    bool getFeatureFile();
    bool getConcavePolygonFile();
    bool getConvexPolygonFile();
    QString getPath();
    QString getPrefix();

    void ok();
    
    void otherSeparator(bool checked);
    void allFiles(int checked);
    void allAttr(int checked);
    void allAttributes(int checked);
    
    
private slots:
    
    void DialogSize(int w, int h );
    void DialogLayout();
    void validate(QString);
    QGroupBox *attributesGroup();
    QGroupBox *separatorGroup();
    
private:
    
    QListWidget * listWidget;
    QList<QListWidgetItem*> inputList;
    QLabel *treeLabel;              /**< Description of tree selection combobox*/
    QComboBox * inputTrees;         /**< List of input tree names */
    QLineEdit *prefixLine;
    
    QLabel *fileLabel;              /**< Desription of QlineEdit for output file */
    QLineEdit *outputFile;          /**< Output file name */
    QPushButton *directoryButton;   /**< Button for selecting directory for project */
    QDialogButtonBox *buttonBox;    /**< Default buttons */
    QHBoxLayout *buttontLayout;     /**< horizontal layout */
    QVBoxLayout *InputLayout;       /**< Default layout of input lines */
    QHBoxLayout *inputareaLayout;   /**< Default layout of input area */
    QVBoxLayout *mainLayout;        /**< Default layout */
    QVBoxLayout *treeLayout;        /**< Default layout */
    QGridLayout *fileLayout;        /**< Default layout of check boxesn and attributes*/
    
    QRadioButton *radio1;           /**< separator button - semicolon*/
    QRadioButton *radio2;           /**< separator button - space*/
    QRadioButton *radio3;           /**< separator button - tabulator*/
    QRadioButton *radio4;           /**< separator button - other*/
    QLineEdit *sep;                 /**< optional separator */
    
    QString m_separator;            /**< QString containg selected separator*/
    QString m_prefix;               /**< QString containg selected file prefix*/
    QString m_path;               /**< QString containg selected file prefix*/
    
    QCheckBox *CHB_attributeFile;
    QCheckBox *CHB_convexPolygon;
    QCheckBox *CHB_concavePolygon;
    QCheckBox *CHB_all;
    
    bool m_convexPolygonFile=false;        // if file with segment profile should be created
    bool m_concavePoylgonFile=false;           // true if file with segment profiles should be created
    bool m_featuresAttributesFile=false;             // if file with tree volume
};

#endif // GUI_H_INCLUDED
