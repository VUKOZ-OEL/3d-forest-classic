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
#include "gui.h"
#include "cloud.h"
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMenu>
#include <QtCore/QString>

////INPUTDIALOG
InputDialog::InputDialog( QWidget *parent)
: QDialog(parent)
{
  DialogSize(500,300);

  isIC1 = false;
  isIC2 = false;
  isIC3 = false;
  isOC1 = false;
  isOC2 = false;
  isII1 = false;
  isII2 = false;
    isII3 = false;
    isII4 = false;
    isII5 = false;
    isII6 = false;
    isII7 = false;
    isII8 = false;
    isII9 = false;
    isII10 = false;
    isType = false;
    isType2 = false;
        isPath = false;
    isICHB = false;
    isICHB2 = false;
  isList =false;
  isDir = false;
    isSBOX = false;
//buttons
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
  connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

  DialogLayout();
}
void InputDialog::ok()
{
  if(isIC1 == true)
    input_cloud1 = inputCloud1->currentText();
  if(isIC2 == true)
    input_cloud2 = inputCloud2->currentText();
  if(isIC3 == true)
    input_cloud3 = inputCloud3->currentText();
  if(isOC1 == true)
    output_cloud1 = outputCloud1->text();
  if(isOC2 == true)
    output_cloud2 = outputCloud2->text();
  if(isPath == true)
    output_path = outputPath->text();
  if(isII1 == true)
    int_value1 = intInput->text().toInt();
  if(isII2 == true)
    int_value2 = intInput2->text().toInt();
  if(isII3 == true)
    int_value3 = intInput3->text().toInt();
  if(isII4 == true)
    int_value4 = intInput4->text().toInt();
if(isII5 == true)
    int_value5 = intInput5->text().toInt();
if(isII6 == true)
    int_value6 = intInput6->text().toInt();
if(isII7 == true)
    int_value7 = intInput7->text().toInt();
if(isII8 == true)
    int_value8 = intInput8->text().toInt();
if(isII9 == true)
    int_value9 = intInput9->text().toInt();
if(isII10 == true)
    int_value10 = intInput10->text().toInt();
  if(isType == true)
    output_type = outputType->currentText();
    if(isType2 == true)
    output_type2 = outputType2->currentText();
  if(isICHB == true)
  {
    if(CHBox->isChecked())
      CHB = true;
    else
      CHB = false;
  }
    if(isICHB2 == true)
    {
      if(CHBox2->isChecked())
        CHB2 = true;
      else
        CHB2 = false;
    }
  if(isList ==true)
  {
    inputList =  listWidget->selectedItems();
  }
  if(isDir == true)
  {
    output_dir = outputDir->text();
  }
    if(isSBOX == true)
        sboxvalue = sbox->value();
}
void InputDialog::DialogSize(int w = 500, int h = 300)
{
  resize(w, h);
}
void InputDialog::DialogLayout()
{
  // buttons
  buttontLayout = new QHBoxLayout();
  buttontLayout->setGeometry(QRect(0,0,400,50));
  buttontLayout->setAlignment(Qt::AlignRight);
  buttontLayout->setSpacing(10);
  buttontLayout->addWidget(buttonBox );
  //buttontLayout->addWidget(okButton);
  //buttontLayout->addWidget(cancelButton);

  //inputlayout of cloudnames and labels
  InputLayout = new QVBoxLayout();

// input of cloud and main label
  inputareaLayout = new QHBoxLayout();
  inputareaLayout->addLayout(InputLayout);

//mainlayout
  mainLayout = new QVBoxLayout();
  mainLayout->addLayout(inputareaLayout);
  mainLayout->addLayout(buttontLayout);
  setLayout(mainLayout);

}
void InputDialog::set_inputCloud1(QString label, QStringList li)
{
//input cloud
  QLabel *label1 = new QLabel();
  label1->setText(label);
  inputCloud1= new QComboBox();
  inputCloud1->setMinimumWidth(300);
  inputCloud1->insertItems(0,li);
//layout
  InputLayout->addWidget(label1);
  InputLayout->addWidget(inputCloud1);
  isIC1 =true;
}
void InputDialog::set_inputList(QString label, QStringList li)
{
//input cloud
  QLabel *labelList = new QLabel();
  labelList->setText(label);
  listWidget= new QListWidget(this);
  listWidget->insertItems(0,li);
  listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
  listWidget->setSortingEnabled(true);
  listWidget->sortItems (Qt::AscendingOrder );
//layout
  InputLayout->addWidget(labelList);
  InputLayout->addWidget(listWidget);
  isList =true;
}
void InputDialog::set_inputCloud2(QString label, QStringList li)
{
//input cloud
  QLabel *label2 = new QLabel();
  label2->setText(label);

  inputCloud2 = new QComboBox();
  inputCloud2->insertItems(0,li);
  label2->setBuddy(inputCloud2);
//layout
  InputLayout->addWidget(label2);
  InputLayout->addWidget(inputCloud2);
  isIC2 =true;
}
void InputDialog::set_inputCloud3(QString label, QStringList li)
{
//input cloud
  QLabel *labeli3 = new QLabel();
  labeli3->setText(label);

  inputCloud3 = new QComboBox();
  inputCloud3->insertItems(0,li);
  labeli3->setBuddy(inputCloud3);
//layout
  InputLayout->addWidget(labeli3);
  InputLayout->addWidget(inputCloud3);
  isIC3 =true;
}
void InputDialog::set_inputCheckBox(QString label)
{
  QLabel *labelbox = new QLabel();
  labelbox->setText(label);
  CHBox = new QCheckBox();
  if(CHBox->checkState() == false)
    CHBox->setChecked(true);
  CHB = true;
  labelbox->setBuddy(CHBox);
//layout
  InputLayout->addWidget(labelbox);
  InputLayout->addWidget(CHBox);
  isICHB =true;
}
void InputDialog::set_inputCheckBox2(QString label)
{
  QLabel *labelbox2 = new QLabel();
  labelbox2->setText(label);
  CHBox2 = new QCheckBox();
  if(CHBox2->checkState() == false)
    CHBox2->setChecked(true);
  CHB2 = true;
  labelbox2->setBuddy(CHBox2);
//layout
  InputLayout->addWidget(labelbox2);
  InputLayout->addWidget(CHBox2);
  isICHB2 =true;
}
void InputDialog::set_description(QString text)
{
  // text about function
  QLabel *label = new QLabel();
  label->setText(text);
  //label->setMaximumSize(180,300);
  label->setFixedWidth(180);
  label->setWordWrap(true);
  label->setMargin(10);
  label->setAlignment(Qt::AlignJustify | Qt::AlignTop);

  QScrollArea* scr = new QScrollArea;
  scr->setWidget(label);
  scr->setFixedSize(200,300);
  scr->setFrameShape(QFrame::NoFrame);
  scr->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);


  //layout
  inputareaLayout->addWidget(scr);

}

void InputDialog::set_title(QString title)
{
  setWindowTitle ( title );
}
void InputDialog::set_outputCloud1(QString label, QString example)
{
  //output cloud
  QLabel *labelOC1 = new QLabel();
  labelOC1 ->setText(label);
  outputCloud1 = new QLineEdit;

  output1Bool=true;
  labelOC1 ->setBuddy(outputCloud1);
  connect(outputCloud1,SIGNAL(textChanged(QString)),this,SLOT(validateOutput1(QString)));
  connect(outputCloud1,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  outputCloud1->setText(example);
//layout
  InputLayout->addWidget(labelOC1);
  InputLayout->addWidget(outputCloud1);
  isOC1 =true;
}
void InputDialog::set_outputCloud2(QString label, QString example)
{
  //output cloud
  QLabel *labelOC2 = new QLabel();
  labelOC2->setText(label);
  outputCloud2 = new QLineEdit;

  output2Bool=true;
  labelOC2 ->setBuddy(outputCloud2);
  connect(outputCloud2,SIGNAL(textChanged(QString)),this,SLOT(validateOutput2(QString)));
  connect(outputCloud2,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  outputCloud2->setText(example);
//layout
  InputLayout->addWidget(labelOC2 );
  InputLayout->addWidget(outputCloud2);
  isOC2 =true;
}
void InputDialog::set_inputInt(QString label, QString x)
{
  QLabel *labelint = new QLabel();
  labelint->setText(label);
  intInput = new QLineEdit;
  intInput->setText(x);
  intInputBool=true;
  intInput->setCursorPosition(0);
  labelint->setBuddy(intInput);
  //resolution
  InputLayout->addWidget(labelint);
  InputLayout->addWidget(intInput);

//ACTION
  connect(intInput,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII1 = true;
}
void InputDialog::set_inputInt2(QString label, QString x)
{
  QLabel *labelint2 = new QLabel();
  labelint2->setText(label);
  intInput2 = new QLineEdit;
  intInput2->setText(x);
  intInputBool2=true;
  intInput2->setCursorPosition(0);
  labelint2->setBuddy(intInput2);
  //resolution
  InputLayout->addWidget(labelint2);
  InputLayout->addWidget(intInput2);

//ACTION
  connect(intInput2,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput2,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII2 = true;
}
void InputDialog::set_inputInt3(QString label, QString x)
{
  QLabel *labelint3 = new QLabel();
  labelint3->setText(label);
  intInput3 = new QLineEdit;
  intInput3->setText(x);
  intInputBool3=true;
  intInput3->setCursorPosition(0);
  labelint3->setBuddy(intInput3);
  //resolution
  InputLayout->addWidget(labelint3);
  InputLayout->addWidget(intInput3);

//ACTION
  connect(intInput3,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput3,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII3 = true;
}
 void InputDialog::set_inputInt4(QString label, QString x)
{
  QLabel *labelint4 = new QLabel();
  labelint4->setText(label);
  intInput4 = new QLineEdit;
  intInput4->setText(x);
  intInputBool4=true;
  intInput4->setCursorPosition(0);
  labelint4->setBuddy(intInput4);
  //resolution
  InputLayout->addWidget(labelint4);
  InputLayout->addWidget(intInput4);

//ACTION
  connect(intInput4,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput4,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII4 = true;
}
void InputDialog::set_inputInt5(QString label, QString x)
{
  QLabel *labelint5 = new QLabel();
  labelint5->setText(label);
  intInput5 = new QLineEdit;
  intInput5->setText(x);
  intInputBool5=true;
  intInput5->setCursorPosition(0);
  labelint5->setBuddy(intInput5);
  //resolution
  InputLayout->addWidget(labelint5);
  InputLayout->addWidget(intInput5);

//ACTION
  connect(intInput5,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput5,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII5 = true;
}
void InputDialog::set_inputInt6(QString label, QString x)
{
  QLabel *labelint6 = new QLabel();
  labelint6->setText(label);
  intInput6 = new QLineEdit;
  intInput6->setText(x);
  intInputBool6=true;
  intInput6->setCursorPosition(0);
  labelint6->setBuddy(intInput6);
  //resolution
  InputLayout->addWidget(labelint6);
  InputLayout->addWidget(intInput6);

//ACTION
  connect(intInput6,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput6,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII6 = true;
}
void InputDialog::set_inputInt7(QString label, QString x)
{
  QLabel *labelint7 = new QLabel();
  labelint7->setText(label);
  intInput7 = new QLineEdit;
  intInput7->setText(x);
  intInputBool7=true;
  intInput7->setCursorPosition(0);
  labelint7->setBuddy(intInput7);
  //resolution
  InputLayout->addWidget(labelint7);
  InputLayout->addWidget(intInput7);

//ACTION
  connect(intInput7,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput7,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII7 = true;
}
void InputDialog::set_inputInt8(QString label, QString x)
{
  QLabel *labelint8 = new QLabel();
  labelint8->setText(label);
  intInput8 = new QLineEdit;
  intInput8->setText(x);
  intInputBool8=true;
  intInput8->setCursorPosition(0);
  labelint8->setBuddy(intInput8);
  //resolution
  InputLayout->addWidget(labelint8);
  InputLayout->addWidget(intInput8);

//ACTION
  connect(intInput8,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput8,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII8 = true;
}
void InputDialog::set_inputInt9(QString label, QString x)
{
  QLabel *labelint9 = new QLabel();
  labelint9->setText(label);
  intInput9 = new QLineEdit;
  intInput9->setText(x);
  intInputBool9=true;
  intInput9->setCursorPosition(0);
  labelint9->setBuddy(intInput9);
  //resolution
  InputLayout->addWidget(labelint9);
  InputLayout->addWidget(intInput9);

//ACTION
  connect(intInput9,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput9,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII9 = true;
}
void InputDialog::set_inputInt10(QString label, QString x)
{
  QLabel *labelint10 = new QLabel();
  labelint10->setText(label);
  intInput10 = new QLineEdit;
  intInput10->setText(x);
  intInputBool10=true;
  intInput10->setCursorPosition(0);
  labelint10->setBuddy(intInput10);
  //resolution
  InputLayout->addWidget(labelint10);
  InputLayout->addWidget(intInput10);

//ACTION
  connect(intInput10,SIGNAL(textChanged(QString)),this,SLOT(validateInt(QString)));
  connect(intInput10,SIGNAL(textChanged(QString)),this,SLOT(validate(QString)));
  isII10 = true;
}


void InputDialog::set_outputType(QString label, QStringList li)
{
  //input cloud
  QLabel *labelType = new QLabel();
  labelType->setText(label);

  outputType = new QComboBox();
  outputType->insertItems(0,li);
  labelType->setBuddy(outputType);
  outputType->setMaximumWidth(150);
//layout
  InputLayout->addWidget(labelType);
  InputLayout->addWidget(outputType);
  isType = true;
}
void InputDialog::set_outputType2(QString label, QStringList li)
{
  //input cloud
  QLabel *labelType2 = new QLabel();
  labelType2->setText(label);

  outputType2 = new QComboBox();
  outputType2->insertItems(0,li);
  labelType2->setBuddy(outputType2);
  outputType2->setMaximumWidth(150);
//layout
  InputLayout->addWidget(labelType2);
  InputLayout->addWidget(outputType2);
  isType2 = true;
}
void InputDialog::set_outputPath(QString label, QString example,QString type)
{
  //input cloud
  QLabel *labelPath = new QLabel();
  labelPath->setText(label);

  outputPath = new QLineEdit();
  labelPath->setBuddy(outputPath);
  outputPath->setMaximumWidth(300);
  outputPath->setText(example);

  directoryButton = new QPushButton(tr("Browse"));
  directoryButton->setBaseSize(40,16);
  m_type = type;
  connect(directoryButton, SIGNAL(clicked()), this, SLOT(saveNewFile()));
//layout
  InputLayout->addWidget(labelPath);
  outputPathLayout = new QHBoxLayout();
  InputLayout->addLayout(outputPathLayout);
  outputPathLayout->addWidget(outputPath);
  outputPathLayout->addWidget(directoryButton);
  isPath = true;
}
void InputDialog::set_outputDir(QString label, QString example)
{
  //input cloud
  QLabel *labelDir = new QLabel();
  labelDir->setText(label);

  outputDir = new QLineEdit();
  labelDir->setBuddy(outputDir);
  outputDir->setMaximumWidth(300);
  outputDir->setText(example);

  directoryButton = new QPushButton(tr("Browse"));
  directoryButton->setBaseSize(40,16);
  connect(directoryButton, SIGNAL(clicked()), this, SLOT(saveIntoDir()));
//layout
  InputLayout->addWidget(labelDir);
  outputPathLayout = new QHBoxLayout();
  InputLayout->addLayout(outputPathLayout);
  outputPathLayout->addWidget(outputDir);
  outputPathLayout->addWidget(directoryButton);
  isDir = true;
}
void InputDialog::saveIntoDir()
{
  QString fileName = QFileDialog::getExistingDirectory(this, tr("Select directory "),"",  QFileDialog::ShowDirsOnly| QFileDialog::DontResolveSymlinks);
  outputDir->setText(fileName);
}
void InputDialog::saveNewFile()
{
  QString fileName;
  fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "", m_type);
  outputPath->setText(fileName);
}
void InputDialog::set_stretch()
{
 InputLayout->addStretch();
}
 void InputDialog::set_path(QString path_m)
 {
   path= path_m;
     //std::cout<< "cesta: " << path.toStdString()<<"\n";
 }
void InputDialog::validateInt(QString text)
{
  if(isII1 == true)
  {
      if (text.contains(QRegExp("^-?\\d+")))
      {
          intInputBool= true;
      }
      else
      {
          buttonBox->setEnabled(false);
          intInputBool= false;
      }
  }
   
  if(isII2 == true)
  {
      if (text.contains(QRegExp("^\\d+")))
      {
          intInputBool2 = true;
      }
      else
      {
          buttonBox->setEnabled(false);
          intInputBool2 = false;
      }
  }
   
    if(isII3 == true)
    {
        if (text.contains(QRegExp("^\\d+")))
        {
            intInputBool3 = true;
        }
        else
        {
            buttonBox->setEnabled(false);
            intInputBool3 = false;
        }
    }
    
    if(isII4 == true)
    {
        if (text.contains(QRegExp("^\\d+")))
        {
            intInputBool4 = true;
        }
        else
        {
            buttonBox->setEnabled(false);
            intInputBool4 = false;
        }
    }
    
}
void InputDialog::validateOutput1(QString text)
{
  if (text.contains(QRegExp("^\\S+$")))
  {
    QString fullName = QString("%1%2%3.pcd").arg(path).arg(QDir::separator ()).arg(text);
    QFile file(fullName);
    if(file.exists())
    {
      outputCloud1->setStyleSheet("QLineEdit{background: red;}");
      buttonBox->setEnabled(false);
    }
    else
    {
      outputCloud1->setStyleSheet("QLineEdit{background: white;}");
      output1Bool= true;
    }
  }
  else
  {
    buttonBox->setEnabled(false);
    output1Bool= false;
  }
}
void InputDialog::validateOutput2(QString text)
{
  if (text.contains(QRegExp("^\\S+$")))
  {
    QString fullName = QString("%1%2%3.pcd").arg(path).arg(QDir::separator ()).arg(text);
    QFile file(fullName);
    if(file.exists())
    {
      outputCloud2->setStyleSheet("QLineEdit{background: red;}");
      buttonBox->setEnabled(false);
    }
    else
    {
      outputCloud2->setStyleSheet("QLineEdit{background: white;}");
      output2Bool= true;
    }
  }
  else
  {
    buttonBox->setEnabled(false);
    output2Bool= false;
  }
}
void InputDialog::validate(QString text)
{
  if(isOC1 == true && isOC2 == true && isII1 == true)
  {
    if (intInputBool == true  && output1Bool == true && output2Bool==true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == true && isOC2 == true && isII1 == false)
  {
    if (output1Bool == true && output2Bool==true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == true && isOC2 == false && isII1 == true)
  {
    if (output1Bool == true && intInputBool == true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == false && isOC2 == true && isII1 == true)
  {
    if (output2Bool == true && intInputBool == true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == false && isOC2 == false && isII1 == true)
  {
    if (intInputBool == true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == true && isOC2 == false && isII1 == false)
  {
    if (output1Bool == true )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
  else if(isOC1 == false && isOC2 == true && isII1 == false)
  {
    if (output2Bool == true  )
    {
      buttonBox->setEnabled(true);
    }
    else { buttonBox->setEnabled(false);}
  }
//  else if(isOC1 == false && isOC2 == false && isII1 == false && isIC1 == true)
//  {
//    if (input1Bool == true  )
//    {
//      buttonBox->setEnabled(true);
//    }
//    else { buttonBox->setEnabled(false);}
//  }
}

QList<QString> InputDialog:: get_inputList()
{
  QList<QString> result;
  for(int i=0; i < inputList.size(); i++)
    result.push_back( inputList.at(i)->text() );

  return result;
}
QString InputDialog::get_inputCloud1()
{
  return input_cloud1;
}
void InputDialog::getinputCloud1()
{
  emit inputCloud_1(input_cloud1);
}
QString InputDialog::get_inputCloud2()
{
  return input_cloud2;
}
void InputDialog::getinputCloud2()
{
  emit inputCloud_2(input_cloud2);
}
QString InputDialog::get_inputCloud3()
{
  return input_cloud3;
}
QString InputDialog::get_outputCloud1()
{
  return output_cloud1;
}
QString InputDialog::get_outputCloud2()
{
  return output_cloud2;
}
QString InputDialog::get_outputPath()
{
  return output_path;
}
int InputDialog::get_intValue()
{
  return int_value1;
}
void InputDialog::getINT()
{
  emit inputINT(int_value1);
}
int InputDialog::get_intValue2()
{
  return int_value2;
}
int InputDialog::get_intValue3()
{
  return int_value3;
}
int InputDialog::get_intValue4()
{
  return int_value4;
}
int InputDialog::get_intValue5()
{
  return int_value5;
}
int InputDialog::get_intValue6()
{
  return int_value6;
}
int InputDialog::get_intValue7()
{
  return int_value7;
}
int InputDialog::get_intValue8()
{
  return int_value8;
}
int InputDialog::get_intValue9()
{
  return int_value9;
}
int InputDialog::get_intValue10()
{
  return int_value10;
}

float InputDialog::getSboxValue()
{
    return sboxvalue;
}
void InputDialog::setSboxDouble(QString label, float dValue, float minV, float maxV)
{
    //input cloud
    QLabel *labelsbox = new QLabel();
    labelsbox->setText(label);
    sbox = new QDoubleSpinBox();
    sbox->setRange(minV, maxV);
    sbox->setDecimals(3);
    sbox->setValue(dValue);
    
    labelsbox->setBuddy(sbox);
    //layout
    InputLayout->addWidget(labelsbox);
    InputLayout->addWidget(sbox);
    
    isSBOX = true;
}
QString InputDialog::get_outputType()
{
  return output_type;
}
QString InputDialog::get_outputType2()
{
  return output_type2;
}
bool InputDialog::get_CheckBox()
{
  return CHB;
}
bool InputDialog::get_CheckBox2()
{
  return CHB2;
}
QString InputDialog::get_outputDir()
{
  return output_dir;
}



//ExportAttr dialog
ExportAttr::ExportAttr(QWidget *parent)
: QDialog(parent)
{
    std::cout<<"ontructor\n";
  DialogSize(500,300);


//buttons
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  treeLabel = new QLabel();
  treeLabel->setText("Select tree name:");
  //inputTrees = new QComboBox();
  listWidget= new QListWidget(this);

  fileLabel = new QLabel();
  fileLabel->setText("Enter name of the file you want to save results:");
  outputFile = new QLineEdit;
  outputFile->setText("C:\\tree_atributes.txt");
  outputFile->setReadOnly(true);
  directoryButton = new QPushButton(tr("Browse"));

  connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
  connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
  connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));

std::cout<<"layout\n";
  DialogLayout();

  DBH_HT = false;
  DBH_LSR = false;
  Position = false;
  Height = false;
  Length = false;
  Points = false;
  Areaconvex = false;
  Areaconcave = false;
    qsmVolume = false;
    sortiments = false;
    std::cout<<"ontructor hotovo\n";
}
void ExportAttr::DialogSize(int w = 500, int h = 300)
{
  resize(w, h);
}
void ExportAttr::DialogLayout()
{
  // buttons
    std::cout<<"buttons\n";
  buttontLayout = new QHBoxLayout();
  buttontLayout->setGeometry(QRect(0,0,400,50));
  buttontLayout->setAlignment(Qt::AlignRight);
  buttontLayout->setSpacing(10);
  buttontLayout->addWidget(buttonBox );
  // tree selection
    std::cout<<"tree selection\n";
  treeLayout = new QVBoxLayout();
  treeLayout->addWidget(treeLabel);
  treeLayout->addWidget(listWidget);
  treeLayout->setSpacing(5);
  //file selection
    std::cout<<"file selection\n";
  fileLayout = new QGridLayout();
  fileLayout->addWidget(fileLabel,1,0,1,2);
  fileLayout->addWidget(outputFile,2,1);
  fileLayout->addWidget(directoryButton,2,2);
  fileLayout->setSpacing(5);

  //inputlayout of cloudnames and labels
    std::cout<<"inputlayout\n";
  InputLayout = new QVBoxLayout();
  InputLayout->addLayout(treeLayout);
  InputLayout->addLayout(fileLayout);
  InputLayout->addWidget(separatorGroup());
  InputLayout->addWidget(attributesGroup());
  InputLayout->setSpacing(10);
// input of cloud and main label
  inputareaLayout = new QHBoxLayout();
  inputareaLayout->addLayout(InputLayout);

//mainlayout
  mainLayout = new QVBoxLayout();
  mainLayout->addLayout(inputareaLayout);
  mainLayout->addLayout(buttontLayout);
  setLayout(mainLayout);

}
void ExportAttr::validate( QString name)
{

}
QString ExportAttr::get_separator()
{
  return m_separator;
}
QString ExportAttr::get_outputFile()
{
  return outputFile->text();
}
bool ExportAttr::get_DBH_HT()
{
  if(DBH_HT == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_DBH_LSR()
{
  if(DBH_LSR == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_Position()
{
  if(Position == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_Height()
{
  if(Height == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_Length()
{
  if(Length == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_Points()
{
  if(Points == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_areaconvex()
{
  if(Areaconvex == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_areaconcave()
{
  if(Areaconcave == true)
    return true;
  else
    return false;
}
bool ExportAttr::get_volume()
{
    if(qsmVolume == true)
        return true;
    else
        return false;
}
bool ExportAttr::get_sortiments()
{
    if(sortiments == true)
        return true;
    else
        return false;
}
QString ExportAttr::get_treeName()
{
  return inputTrees->currentText();
}
void ExportAttr::ok()
{
  inputList =  listWidget->selectedItems();
  //separator
  if(radio1->isChecked())
    m_separator = (";");
  if(radio2->isChecked())
    m_separator = (" ");
  if(radio3->isChecked())
    m_separator = ("\t");
  if(radio4->isChecked())
    m_separator = sep->text();

    //attributes
  if(CHB_DBH_HT->isChecked())
    DBH_HT = true;
  if(CHB_DBH_LSR->isChecked())
    DBH_LSR = true;
  if(CHB_position->isChecked())
    Position = true;
  if(CHB_height->isChecked())
    Height = true;
  if(CHB_length->isChecked())
    Length = true;
  if(CHB_points->isChecked())
    Points = true;
  if(CHB_areacave->isChecked())
    Areaconcave = true;
  if(CHB_areavex->isChecked())
    Areaconvex = true;
    if(CHB_volume->isChecked())
        qsmVolume = true;
    if(CHB_sortiments->isChecked())
        sortiments = true;
}
QGroupBox *ExportAttr::attributesGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Tree attributes"));
  groupBox->setFlat(true);

  CHB_DBH_HT = new QCheckBox(tr("DBH computed by \nRandomized Hough Transform"));
  connect(CHB_DBH_HT, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_DBH_LSR = new QCheckBox(tr("DBH computed by \nLeast Square Regression"));
  connect(CHB_DBH_LSR, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_height = new QCheckBox(tr("Tree height"));
  connect(CHB_height, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_length = new QCheckBox(tr("Tree length"));
  connect(CHB_length, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_position = new QCheckBox(tr("X Y Z coordinate \nof tree position"));
  connect(CHB_position, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_areavex = new QCheckBox(tr("Area of convex hull"));
  connect(CHB_areavex, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_areacave = new QCheckBox(tr("Area of concave hull"));
  connect(CHB_areacave, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_points = new QCheckBox(tr("Point number"));
  connect(CHB_points, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
    CHB_volume = new QCheckBox(tr("Tree volume"));
    connect(CHB_volume, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
    CHB_sortiments = new QCheckBox(tr("Tree sortiments"));
    connect(CHB_sortiments, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_all = new QCheckBox(tr("All attributes"));
  connect(CHB_all, SIGNAL(stateChanged(int)), this, SLOT(all_attributes(int)));

  QGridLayout *box = new QGridLayout();
  box->addWidget(CHB_DBH_HT,1,1);
  box->addWidget(CHB_DBH_LSR,1,2);
  box->addWidget(CHB_height,1,3);
  box->addWidget(CHB_length ,2,1);
  box->addWidget(CHB_position,2,2);
  box->addWidget(CHB_areavex,2,3);
  box->addWidget(CHB_areacave ,3,1);
  box->addWidget(CHB_points,3,2);
  box->addWidget(CHB_volume,3,3);
    box->addWidget(CHB_sortiments,4,1);
    box->addWidget(CHB_all,4,2);
  groupBox->setLayout(box);
  return groupBox;
}
QGroupBox *ExportAttr::separatorGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Data separator"));
  groupBox->setFlat(true);

  radio1 = new QRadioButton(tr("Semicolon"));
  radio2 = new QRadioButton(tr("Space"));
  radio2->setChecked(true);
  radio3 = new QRadioButton(tr("Tabulator"));
  radio4 = new QRadioButton(tr("Other:"));
  sep = new QLineEdit(this);
  sep->setFixedWidth(60);
  sep->setReadOnly(true);
  sep->setStyleSheet("QLineEdit{background: lightGrey;}");

  connect(radio4, SIGNAL(toggled(bool)), this, SLOT(other_Separator(bool)));
  QHBoxLayout *hbox = new QHBoxLayout;
  hbox->addWidget(radio1);
  hbox->addWidget(radio2);
  hbox->addWidget(radio3);
  hbox->addWidget(radio4);
  hbox->addWidget(sep);
  hbox->setSpacing(10);
  hbox->addStretch(1);
  groupBox->setLayout(hbox);
  return groupBox;
}
void ExportAttr::set_description(QString text)
{
  // text about function
  QLabel *label = new QLabel();
  label->setText(text);
  //label->setMaximumSize(180,300);
  label->setMinimumWidth(160);
  label->setWordWrap(true);
  label->setMargin(10);
  label->setAlignment(Qt::AlignJustify | Qt::AlignTop);
  //layout
  inputareaLayout->addWidget(label);

}
void ExportAttr::set_list(QStringList li)
{
  listWidget->insertItems(0,li);
  listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
  listWidget->setSortingEnabled(true);
}
QList<QString> ExportAttr:: get_inputList()
{
  QList<QString> result;
  for(int i=0; i < inputList.size(); i++)
    result.push_back( inputList.at(i)->text() );

  return result;
}
void ExportAttr::setExistingDirectory()
{
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("Text file (*.txt)"));
  outputFile->setText(fileName);
}
void ExportAttr::other_Separator(bool checked)
{
  if(checked == true)
  {
    sep->setReadOnly(false);
    sep->setStyleSheet("QLineEdit{background: white;}");
  }

  else
  {
    sep->setReadOnly(true);
    sep->setStyleSheet("QLineEdit{background: lightGrey;}");
    sep->setText("");
  }
}
void ExportAttr::all_attributes(int checked)
{
  if(checked == 2)
  {
    CHB_DBH_HT->setChecked(true);
    CHB_DBH_LSR->setChecked(true);
    CHB_height->setChecked(true);
    CHB_length->setChecked(true);
    CHB_position->setChecked(true);
    CHB_areavex->setChecked(true);
    CHB_areacave->setChecked(true);
    CHB_points->setChecked(true);
      CHB_volume->setChecked(true);
      CHB_sortiments->setChecked(true);
  }
}
void ExportAttr::all_attr(int checked)
{
  if(checked == 0)
  {
    CHB_all->setChecked(false);
  }
}

//import Project
ProjImport::ProjImport(QWidget *parent)
     : QWizard(parent)
{
    std::cout<<"ProjImport::ProjImport\n" ;
  //Q_INIT_RESOURCE(3dforest);
  setPixmap(QWizard::LogoPixmap, QPixmap(":/images/logo.png"));
  setPixmap(QWizard::BannerPixmap,QPixmap(":/images/banner.png"));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));
  setWizardStyle(QWizard::ModernStyle);
std::cout<<"style\n" ;
  setPage(Page_Intro, new IntroPage(this));
    std::cout<<"Page_Intro\n" ;
  setPage(Page_newProject, new NewProjectPage(this));
    std::cout<<"Page_newProject\n" ;
  setPage(Page_transform, new TransformPage(this));
    std::cout<<"Page_transform\n" ;
  setPage(Page_import, new ImportPage(this));
    std::cout<<"Page_import\n" ;
  setPage(Page_final, new FinalPage(this));
std::cout<<"Page_final\n" ;

  setStartId(Page_Intro);
  setWindowTitle(tr("Project manager"));
    std::cout<<"constructor hotovo\n" ;
}

IntroPage::IntroPage(QWidget *parent)
     : QWizardPage(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("Welcome in project manager"));
  setSubTitle(tr("Please specify if you want to create a new project or import existing one."));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));

  topLabel = new QLabel(tr("This wizard will guide you through importing of projects from different locations or "
                           "by creating a new project. Please select if you want to import existing project or if you want to create a new one."));
  topLabel->setWordWrap(true);

  newProjectButton = new QRadioButton(tr("&Create new project"));
  importProjectButton = new QRadioButton(tr("&Import existing project"));
  importProjectButton->setChecked(true);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  layout->addWidget(newProjectButton);
  layout->addWidget(importProjectButton);
  setLayout(layout);
 }
int IntroPage::nextId() const
 {
     if (newProjectButton->isChecked()) {
         return ProjImport::Page_newProject;
     } else {
         return ProjImport::Page_import;
     }
 }

NewProjectPage::NewProjectPage(QWidget *parent)
     : QWizardPage(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("New project"));
  setSubTitle(tr("Details of the new project"));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));

  topLabel = new QLabel(tr("Please select a name and path to the new project. "
                           "If any field is red, it means that on given location a folder with the same name already exists. "
                           "In that case please change the name of the project or path to the project folder."));
  topLabel->setWordWrap(true);

  nameLabel = new QLabel(tr("Project name (please do not use spaces in name):"));
  nameEdit = new QLineEdit;
  nameLabel->setBuddy(nameEdit);
  registerField("projectName*", nameEdit);


  pathLabel = new QLabel(tr("Path to project folder:"));
  pathEdit = new QLineEdit;
  pathEdit->setText("....");
  pathEdit->setReadOnly(true);
  pathLabel->setBuddy(pathEdit);
  registerField("projectPath*", pathEdit);

  directoryButton = new QPushButton(QPixmap(":/images/browse.png"),("Browse"));

  QHBoxLayout *layoutPath = new QHBoxLayout;
  layoutPath->addWidget(pathEdit);
  layoutPath->addWidget(directoryButton);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  layout->addWidget(nameLabel);
  layout->addWidget(nameEdit);
  layout->addWidget(pathLabel);
  layout->addLayout(layoutPath);
  setLayout(layout);

  connect(nameEdit, SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  connect(pathEdit, SIGNAL(textChanged(QString)),this,SLOT(pathCheck(QString)));
  connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));
}
void NewProjectPage::setExistingDirectory()
{
    QFileDialog::Options options = QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly;
    directory = QFileDialog::getExistingDirectory(this, tr("Select directory for project"),tr("") , options);
    pathEdit->setText(directory);
}
void NewProjectPage::nameCheck(QString name)
{
  bool used=false;
  QString path = QString("%1%2%3").arg(field("projectPath").toString()).arg(QDir::separator ()).arg(field("projectName").toString());
  QDir myDir(path);

  if(myDir.exists())
    used=true;

  if(used == false && name.contains(QRegExp("^[a-zA-Z0-9_-]*$")))
  {
    nameEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    nameEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
void NewProjectPage::pathCheck(QString name)
{
  bool used=false;
  QString path = QString("%1%2%3").arg(field("projectPath").toString()).arg(QDir::separator ()).arg(field("projectName").toString());
  QDir myDir(path);

  if(myDir.exists())
    used=true;

  if(used == false)
  {
    nameEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    nameEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
int NewProjectPage::nextId() const
{
  // create dir in path;
  QString pathDir = QString("%1%2%3").arg(field("projectPath").toString()).arg(QDir::separator ()).arg(field("projectName").toString());
  QDir myDir(pathDir);

  if(!myDir.exists())
    myDir.mkpath(".");

  return ProjImport::Page_transform;
}

TransformPage::TransformPage(QWidget *parent)
     : QWizardPage(parent)
{
  openProjects();
    std::cout<<"openProjects()\n" ;
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("New project"));
  setSubTitle(tr("Details of the new project - Transformation matrix"));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/transform.png"));
std::cout<<"style\n" ;
  topLabel = new QLabel(tr("Please choose transformation matrix or create new. More info about transformation can be found in user guide."));
  topLabel->setWordWrap(true);

  selectButton = new QRadioButton(tr("Choose transform matrix"));
  newButton = new QRadioButton(tr("Create new transform matrix"));
  selectButton->setChecked(true);

std::cout<<"buttons\n" ;
  nameLabel = new QLabel(tr("Choose existing transformation:"));
  transformList = new QComboBox;
  transformList->insertItems(0,names);
  nameLabel->setBuddy(transformList);


  projLabel = new QLabel(tr("Name of new transformation matrix (without spaces):"));
  projEdit = new QLineEdit;
  projLabel->setBuddy(projEdit);
  projEdit->setEnabled(false);
  registerField("projName*", projEdit);
  projEdit->setText(names.at(0));

  projXLabel = new QLabel(tr("Enter value for X coordinate:"));
  projXEdit = new QLineEdit;
  projXLabel->setBuddy(projXEdit);
  projXEdit->setEnabled(false);
  registerField("projX*", projXEdit);
  projXEdit->setText(x.at(0));

  projYLabel = new QLabel(tr("Enter value for Y coordinate:"));
  projYEdit = new QLineEdit;
  projYLabel->setBuddy(projYEdit);
  projYEdit->setEnabled(false);
  registerField("projY*", projYEdit);
  projYEdit->setText(y.at(0));

  projZLabel = new QLabel(tr("Enter value for Z coordinate:"));
  projZEdit = new QLineEdit;
  projZLabel->setBuddy(projZEdit);
  projZEdit->setEnabled(false);
  registerField("projZ*", projZEdit);
  projZEdit->setText(z.at(0));


  QGridLayout *layoutgrid = new QGridLayout;
  layoutgrid->setColumnMinimumWidth(1,20);
  layoutgrid->setColumnMinimumWidth(2,80);
  layoutgrid->addWidget(selectButton,1,1,1,2);
  layoutgrid->addWidget(nameLabel,2,2);
  layoutgrid->addWidget(transformList,3,2);

  layoutgrid->addWidget(newButton,4,1,1,2);
  layoutgrid->addWidget(projLabel,5,2);
  layoutgrid->addWidget(projEdit,6,2);
  layoutgrid->addWidget(projXLabel,7,2);
  layoutgrid->addWidget(projXEdit,8,2);
  layoutgrid->addWidget(projYLabel,9,2);
  layoutgrid->addWidget(projYEdit,10,2);
  layoutgrid->addWidget(projZLabel,11,2);
  layoutgrid->addWidget(projZEdit,12,2);


  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  layout->addLayout(layoutgrid);
  setLayout(layout);

  connect(selectButton, SIGNAL(toggled(bool)), this, SLOT(selectProj(bool)));
  connect(newButton, SIGNAL(toggled(bool)), this, SLOT(newProj(bool)));
  connect(transformList, SIGNAL(currentIndexChanged (int)), this, SLOT(selected(int)));
  connect(projXEdit,SIGNAL(textChanged(QString)),this,SLOT(numberXCheck(QString)));
  connect(projYEdit,SIGNAL(textChanged(QString)),this,SLOT(numberYCheck(QString)));
  connect(projZEdit,SIGNAL(textChanged(QString)),this,SLOT(numberZCheck(QString)));
}

void TransformPage::openProjects()
{
    std::cout<<"TransformPage::openProjects()\n" ;
  QFile fileproj ("projects.txt");
  names << "select projection matrix";
  x << "";
  y << "";
  z << "";
    std::cout<<"if(!fileproj.exists())\n" ;
    if(!fileproj.exists())
        return;
    std::cout<<"fileproj.open()\n" ;
    fileproj.open(QIODevice::ReadOnly | QIODevice::Text);
    QTextStream in(&fileproj);
    std::cout<<"QTextStream\n" ;
  while(!in.atEnd())
  {
    QString line = in.readLine();
    QStringList coords = line.split(" ");
      if(coords.size() == 4)
      {
          names << coords.at(0);
          x << coords.at(1);
          y << coords.at(2);
          z << coords.at(3);
      }
  }
  fileproj.close();
    std::cout<<"fileproj.close();\n" ;
}
int TransformPage::nextId() const
{
  if(newButton->isChecked())
  {
    // pridat zapis do projects
    QFile fileproj ("projects.txt");
    fileproj.open(QIODevice::Append | QIODevice::Text);
    QTextStream out(&fileproj);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(0);
    out <<"\n"<< field("projName").toString() <<" "<< field("projX").toString() << " " << field("projY").toString()<< " " << field("projZ").toString() ;
    fileproj.close();
  }

  // create proj file
  QString pathDir = QString("%1%2%3").arg(field("projectPath").toString()).arg(QDir::separator ()).arg(field("projectName").toString());
  QString fileN =QString("%1%2%3.3df").arg(pathDir).arg(QDir::separator ()).arg(field("projectName").toString());
  QFile file (fileN);
  file.open(QIODevice::WriteOnly);
  // write header of proj file
  QTextStream out(&file);
  out.setRealNumberNotation(QTextStream::FixedNotation);
  out.setRealNumberPrecision(0);
  out << field("projectName").toString() <<" "<< field("projX").toString() << " " << field("projY").toString()<< " " << field("projZ").toString()<< " "<<  pathDir <<"\n" ;
  file.close();

  return ProjImport::Page_final;
}
void TransformPage::selectProj(bool check)
{
  projEdit->setEnabled(false);
  projXEdit->setEnabled(false);
  projYEdit->setEnabled(false);
  projZEdit->setEnabled(false);
  transformList->setEnabled(true);
  projEdit->setStyleSheet("QLineEdit{background: white;}");
  disconnect(projEdit,SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  connect(transformList, SIGNAL(currentIndexChanged (int)), this, SLOT(selected(int)));
}
void TransformPage::newProj(bool check)
{
  disconnect(transformList, SIGNAL(currentIndexChanged (int)), this, SLOT(selected(int)));
  connect(projEdit,SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  transformList->setCurrentIndex (0);
  transformList->setEnabled(false);
  projEdit->setEnabled(true);
  projXEdit->setEnabled(true);
  projYEdit->setEnabled(true);
  projZEdit->setEnabled(true);
  projEdit->setText("");
  wizard()->button(QWizard::NextButton)->setEnabled(false);
  projEdit->setStyleSheet("QLineEdit{background: red;}");
}
void TransformPage::selected(int index)
{
  disconnect(projEdit,SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  connect(transformList, SIGNAL(currentIndexChanged (int)), this, SLOT(selected(int)));
  projEdit->setText(names.at(index));
  projXEdit->setText(x.at(index));
  projYEdit->setText(y.at(index));
  projZEdit->setText(z.at(index));

}
void TransformPage::nameCheck(QString name)
{
  bool used = false;
  for(int i = 0; i < names.size(); i++)
  {
    if(name == names.at(i))
      used = true;
  }

  if(used == true || name.contains(QRegExp("^\\s+$")))
  {
    projEdit->setStyleSheet("QLineEdit{background: red;}");
  wizard()->button(QWizard::NextButton)->setEnabled(false);
  }

  else
  {
    projEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
}
void TransformPage::numberXCheck(QString name)
{
  if (name.contains(QRegExp("^\\-?\\d+$")))
  {
   projXEdit->setStyleSheet("QLineEdit{background: white;}");
   wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projXEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
void TransformPage::numberYCheck(QString name)
{
  if (name.contains(QRegExp("^\\-?\\d+$")))
  {
    projYEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projYEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
void TransformPage::numberZCheck(QString name)
{
  if (name.contains(QRegExp("^\\-?\\d+$")))
  {
    projZEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projZEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
ImportPage::ImportPage(QWidget *parent)
     : QWizardPage(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("Import project"));
  setSubTitle(tr("Setup of project"));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));

  topLabel = new QLabel(tr("Please choose project file that you want to import into a new location. "
                           "All clouds of the project will be moved into a new folder at the specified location."));
  topLabel->setWordWrap(true);

  oldLabel = new QLabel(tr("Path to old project file:"));
  oldpathEdit = new QLineEdit;
  oldpathEdit->setText("");
  oldpathEdit->setReadOnly(true);
  oldLabel->setBuddy(oldpathEdit);
  registerField("oldprojectPath*", oldpathEdit);

  oldButton = new QPushButton(QPixmap(":/images/browse.png"),("Browse"));

  projLabel = new QLabel(tr("Name of new project"));
  projEdit = new QLineEdit;
  projLabel->setBuddy(projEdit);
  registerField("newprojectname*", projEdit);

  newLabel = new QLabel(tr("Path to new project folder:"));
  newpathEdit = new QLineEdit;
  newpathEdit->setText("C:\\");
  newpathEdit->setReadOnly(true);
  newLabel->setBuddy(newpathEdit);
  registerField("newprojectPath*", newpathEdit);

  newButton = new QPushButton(QPixmap(":/images/browse.png"),("Browse"));
  removeCheckBox = new QCheckBox ( tr("Remove old project?"),this);

  QGridLayout *layoutgrid = new QGridLayout;
  layoutgrid->setColumnMinimumWidth(1,20);
  layoutgrid->setColumnMinimumWidth(2,80);
  layoutgrid->addWidget(oldLabel,1,2);
  layoutgrid->addWidget(oldpathEdit,2,2);
  layoutgrid->addWidget(oldButton,2,3);
  layoutgrid->addWidget(projLabel,3,2);
  layoutgrid->addWidget(projEdit,4,2);
  layoutgrid->addWidget(newLabel,5,2);
  layoutgrid->addWidget(newpathEdit,6,2);
  layoutgrid->addWidget(newButton,6,3);
  layoutgrid->addWidget(removeCheckBox,7,2);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  layout->addLayout(layoutgrid);
  setLayout(layout);

  connect(newButton, SIGNAL(clicked()), this, SLOT(setNewDirectory()));
  connect(oldButton, SIGNAL(clicked()), this, SLOT(setOldFile()));
  connect(projEdit, SIGNAL(textChanged(QString)),this,SLOT(nameCheck(QString)));
  connect(newpathEdit, SIGNAL(textChanged(QString)),this,SLOT(pathCheck(QString)));
}
void ImportPage::setOldFile()
{
  QString file = QFileDialog::getOpenFileName(this,tr("open file"),"",tr("files (*.3df)"));
  oldpathEdit->setText(file);
}
void ImportPage::setNewDirectory()
{
  QFileDialog::Options options = QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly;
  QString directory = QFileDialog::getExistingDirectory(this, tr("Select directory for project"),tr("") , options);
  newpathEdit->setText(directory);
}
void ImportPage::nameCheck(QString name)
{
  bool used=false;
  QString path = QString("%1%2%3").arg(field("newprojectPath").toString()).arg(QDir::separator ()).arg(field("newprojectname").toString());
  QDir myDir(path);

  if(myDir.exists())
    used=true;

  if(used == false && name.contains(QRegExp("^[a-zA-Z0-9_-]*$")))
  {
    projEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}
void ImportPage::pathCheck(QString name)
{
  bool used=false;
  QString path = QString("%1%2%3").arg(field("newprojectPath").toString()).arg(QDir::separator ()).arg(field("newprojectname").toString());
  QDir myDir(path);

  if(myDir.exists())
    used=true;

  if(used == false)
  {
    projEdit->setStyleSheet("QLineEdit{background: white;}");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
  }
  else
  {
    projEdit->setStyleSheet("QLineEdit{background: red;}");
    wizard()->button(QWizard::NextButton)->setEnabled(false);
  }
}

int ImportPage::nextId() const
{
  // open new project file
  QString pathDir = QString("%1%2%3").arg(field("newprojectPath").toString()).arg(QDir::separator ()).arg(field("newprojectname").toString());
  QDir myDir(pathDir);
  if(!myDir.exists())
    myDir.mkpath(".");

  QString fileN =QString("%1%2%3.3df").arg(pathDir).arg(QDir::separator ()).arg(field("newprojectname").toString());
  QFile filenew (fileN);
  filenew.open(QIODevice::WriteOnly| QIODevice::Text);
  QTextStream out(&filenew);

  // open old project file
  QFile fileold (field("oldprojectPath").toString());
  fileold.open(QIODevice::ReadOnly | QIODevice::Text);
  QTextStream in(&fileold);
  QFileInfo fold (fileold);
  QString fileold_path = fold.canonicalPath();

  bool first_line = true;
  while(!in.atEnd())
  {
    QString lines = in.readLine();
    QStringList coords = lines.split(" ");
//READ FIRST LINE

    if(first_line == true)
    {
      if(coords.size() == 5)
      {
        out << field("newprojectname").toString() << " " << coords.at(1) << " " << coords.at(2)<< " " << coords.at(3)<< " " << pathDir<< "\n";
      }
      else
      {

        out << field("newprojectname").toString() << " " << coords.at(1) << " " << coords.at(2)<< " " << "0"<< " " << pathDir<< "\n";
      }

      first_line = false;
    }
//READ REST OF FILE
    else
    {
      if(coords.size() == 5) //type, path, r,g,b
      {
        QStringList apth = coords.at(1).split(QDir::separator ());
        QString new_file = QString("%1%2%3").arg(pathDir).arg(QDir::separator ()).arg(apth.at(apth.size()-1));
        QString old_file = QString("%1%2%3").arg(fileold_path).arg(QDir::separator ()).arg(apth.at(apth.size()-1));

        // zkopirovat soubor
        QFile::copy(old_file, new_file);
        //ulozit do proj.3df
        out << coords.at(0) << " " << new_file << " " << coords.at(2)<< " " << coords.at(3)<< " " << coords.at(4)<< "\n";
      }
      else if (coords.size() > 0)
      {
        QStringList apth = coords.at(1).split(QDir::separator ());
        QString new_file = QString("%1%2%3").arg(pathDir).arg(QDir::separator ()).arg(apth.at(apth.size()-1));
        QString old_file = QString("%1%2%3").arg(fileold_path).arg(QDir::separator ()).arg(apth.at(apth.size()-1));
        // zkopirovat soubor
        QFile::copy(old_file, new_file);
        //ulozit do proj.3df
        out << coords.at(0) << " " << new_file <<"\n";
      }
      else
      {
        continue;
      }
    }
  }
  filenew.close(); // new file close
  fileold.close(); // old file close


  if(removeCheckBox->checkState () == 2)
  {
    //remove old projectQFileInfo f(field("oldprojectPath").toString());
    QFileInfo f(field("oldprojectPath").toString());
    QDir dir(f.canonicalPath ());
    if (dir.exists())
    {
      Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst))
      {
       QFile::remove(info.absoluteFilePath());
      }
      dir.rmdir(f.canonicalPath());
    }
  }
  return ProjImport::Page_final;
}
FinalPage::FinalPage(QWidget *parent)
     : QWizardPage(parent)
{
  //Q_INIT_RESOURCE(3dforest);
  setTitle(tr("Welcome in project manager"));
  setSubTitle(tr("Final conclulsion."));
  setPixmap(QWizard::WatermarkPixmap, QPixmap(":/images/strom.png"));

  topLabel = new QLabel(tr("Your project is created and ready to use! Enjoy use of 3D Forest and if you have any questions feel free to ask at web site http://www.3dforest.eu"));
  topLabel->setWordWrap(true);


  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(topLabel);
  setLayout(layout);
 }
int FinalPage::nextId() const
{
  // open project file

  return -1;
}

////MYTREE
MyTree::MyTree(QWidget *parent)
  : QTreeWidget(parent)
{
    setContextMenuPolicy(Qt::CustomContextMenu);
    setColumnCount(2);
    //resizeColumnToContents(1);
    header()->resizeSection(0, 50);
    QStringList q;
    q << QString (" visible") ;
    q << QString ("     name");
    setHeaderLabels(q);
    header()->resizeSection(1, 120);
    setSortingEnabled(true);

    //SLOTS
    connect(this, SIGNAL(customContextMenuRequested(const QPoint&)),this, SLOT(showContextMenu(const QPoint&)));
    connect(this, SIGNAL(itemChanged(QTreeWidgetItem*,int)), this, SLOT(onItemChange(QTreeWidgetItem*,int)));
}
void MyTree::itemdelete(QString name)
{
  QTreeWidgetItemIterator it(this);
  while (*it)
  {
    if( (*it)->text(1) == name)
    {
      QTreeWidgetItem *currentItem = (*it);
      delete currentItem;
    }
    ++it;
  }
}
void MyTree::showContextMenu(const QPoint &pos)
{
  QMenu *menu = new QMenu;

  QModelIndex index = this->currentIndex();
  if(!index.isValid())
    return;

  QString it = this->model()->data(this->model()->index(index.row(), 1),0).toString();

  QAction *deleteACT = new QAction("Delete",menu);
  menu->addAction(deleteACT);

  QAction *colorACT = new QAction("Color",menu);
  menu->addAction(colorACT);

  QAction *colorFieldACT = new QAction("Color by field",menu);
  menu->addAction(colorFieldACT);

  QAction *PsizeACT = new QAction("Point size",menu);
  menu->addAction(PsizeACT);

  QAction *allONACT = new QAction("All Clouds ON",menu);
  menu->addAction(allONACT);
  connect(allONACT, SIGNAL(triggered()), this, SLOT(allON()));

  QAction *allOFFACT = new QAction("All Clouds OFF",menu);
  menu->addAction(allOFFACT);
  connect(allOFFACT, SIGNAL(triggered()), this, SLOT(allOFF()));



  QSignalMapper *signalMapper = new QSignalMapper(this);
  connect(deleteACT, SIGNAL(triggered()), signalMapper, SLOT(map()));
  signalMapper->setMapping(deleteACT, it);
  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(onDeleteItem(QString)));

  QSignalMapper *signalMapperC = new QSignalMapper(this);
  connect(colorACT, SIGNAL(triggered()), signalMapperC, SLOT(map()));
  signalMapperC->setMapping(colorACT, it);
  connect(signalMapperC, SIGNAL(mapped(QString)), this, SLOT(onColor(QString)));

  QSignalMapper *signalMapperCF = new QSignalMapper(this);
  connect(colorFieldACT, SIGNAL(triggered()), signalMapperCF, SLOT(map()));
  signalMapperCF->setMapping(colorFieldACT, it);
  connect(signalMapperCF, SIGNAL(mapped(QString)), this, SLOT(onColorField(QString)));

  QSignalMapper *signalMapperPsize = new QSignalMapper(this);
  connect(PsizeACT, SIGNAL(triggered()), signalMapperPsize, SLOT(map()));
  signalMapperPsize->setMapping(PsizeACT, it);
  connect(signalMapperPsize, SIGNAL(mapped(QString)), this, SLOT(onPsize(QString)));

  menu->exec(QCursor::pos());
}
void MyTree::onItemChange(QTreeWidgetItem *item,int i)
{
  name = item->text(1);

  if(!item->checkState(0))
  {
    emit checkedON(name);
  }
 if(item->checkState(0))
  {
    emit checkedOFF(name);
  }
}
void MyTree::onDeleteItem(QString name)
{
  QMessageBox *msgBox =  new QMessageBox(0);
	msgBox->setText("DELETE");
	QString a = QString("Do you want to delete cloud -- %1 -- from project?").arg(name);
	msgBox->setInformativeText(a);
	msgBox->setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox->setDefaultButton(QMessageBox::Yes);

	if(msgBox->exec() == QMessageBox::Yes)
  {
    itemdelete(name);
    emit deleteItem(name);
  }
  delete msgBox;
}
void MyTree::onColor(QString name)
{
  emit colorItem(name);
}
void MyTree::onColorField(QString name)
{
  emit colorItemField(name);
}
void MyTree::onPsize(QString name)
{
  emit psize(name);
}
void MyTree::cleanAll()
{
  QTreeWidgetItemIterator it(this);
//  int index = 0;
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
     if(!item)return;
     int x = this->indexOfTopLevelItem(item);
     if(x >= 0 && x < this->topLevelItemCount())
     {
       item = this->takeTopLevelItem(x);
       if(item)
         delete item;
     }
  }
}
void MyTree::allON()
{
  QTreeWidgetItemIterator it(this);
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
    name = item->text(1);
    item->setCheckState(0,Qt::Checked);
    emit checkedOFF(name);
    it++;
  }
}
void MyTree::allOFF()
{
  allItemOFF();
}
void MyTree::allItemOFF()
{
  QTreeWidgetItemIterator it(this);
  //int index = 0;
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
    name = item->text(1);
    item->setCheckState(0,Qt::Unchecked);
    emit checkedON(name);
    it++;
  }

}
void MyTree::itemON(QString name)
{
QTreeWidgetItemIterator it(this);
  while (*it)
  {
    QTreeWidgetItem *item = (*it);
    if (name == item->text(1))
    {
      item->setCheckState(0,Qt::Checked);
      emit checkedOFF(name);
    }
    it++;
  }
}


//ExportCrownAttr dialog
ExportCrownAttr::ExportCrownAttr(QWidget *parent)
: QDialog(parent)
{
  DialogSize(500,300);


//buttons
  buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  treeLabel = new QLabel();
  treeLabel->setText("Select tree name:");
  listWidget= new QListWidget(this);

  fileLabel = new QLabel();
  fileLabel->setText("Enter name of the file you want to save results:");
  outputFile = new QLineEdit;
  outputFile->setText("C:\\crown_atributes.txt");
  outputFile->setReadOnly(true);
  directoryButton = new QPushButton(tr("Browse"));

  connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
  connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
  connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
  connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));


  DialogLayout();

    points = false;
    height = false;
    bottomHeight = false;
    totalHeight = false;
    positionDev = false;
    positionXYZ = false;
    width = false;
    length = false;
    volVoxels = false;
    volSections = false;
    vol3DCH = false;
    surface = false;
    surf3DCH = false;
    thresholdDist = false;
    sectionHeight = false;
    
}
void ExportCrownAttr::DialogSize(int w = 500, int h = 500)
{
  resize(w, h);
}
void ExportCrownAttr::DialogLayout()
{
  // buttons
  buttontLayout = new QHBoxLayout();
  buttontLayout->setGeometry(QRect(0,0,400,50));
  buttontLayout->setAlignment(Qt::AlignRight);
  buttontLayout->setSpacing(10);
  buttontLayout->addWidget(buttonBox );
  // tree selection
  treeLayout = new QVBoxLayout();
  treeLayout->addWidget(treeLabel);
  treeLayout->addWidget(listWidget);
  treeLayout->setSpacing(5);
  //file selection
  fileLayout = new QGridLayout();
  fileLayout->addWidget(fileLabel,1,0,1,2);
  fileLayout->addWidget(outputFile,2,1);
  fileLayout->addWidget(directoryButton,2,2);
  fileLayout->setSpacing(5);

  //inputlayout of cloudnames and labels
  InputLayout = new QVBoxLayout();
  InputLayout->addLayout(treeLayout);
  InputLayout->addLayout(fileLayout);
  InputLayout->addWidget(separatorGroup());
  InputLayout->addWidget(attributesGroup());
  InputLayout->setSpacing(10);
// input of cloud and main label
  inputareaLayout = new QHBoxLayout();
  inputareaLayout->addLayout(InputLayout);

//mainlayout
  mainLayout = new QVBoxLayout();
  mainLayout->addLayout(inputareaLayout);
  mainLayout->addLayout(buttontLayout);
  setLayout(mainLayout);

}
void ExportCrownAttr::validate( QString name)
{

}
QString ExportCrownAttr::get_separator()
{
  return m_separator;
}
QString ExportCrownAttr::get_outputFile()
{
  return outputFile->text();
}
bool ExportCrownAttr::getVolVoxels()
{
  if(volVoxels == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getVolSections()
{
  if(volSections == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getVol3DCH()
{
  if(vol3DCH == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getSurface()
{
  if(surface == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getSurf3DCH()
{
  if(surf3DCH == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getPositionDeviance()
{
  if(positionDev == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getPositionXYZ()
{
  if(positionXYZ == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getHeight()
{
  if(height == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getBottomHeight()
{
  if(bottomHeight == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getTotalHeight()
{
  if(totalHeight == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getLength()
{
  if(length == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getWidth()
{
  if(width == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getPoints()
{
  if(points == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getSectionHeight()
{
  if(sectionHeight == true)
    return true;
  else
    return false;
}
bool ExportCrownAttr::getThresholdDistance()
{
  if(thresholdDist == true)
    return true;
  else
    return false;
}
QString ExportCrownAttr::get_treeName()
{
  return inputTrees->currentText();
}
void ExportCrownAttr::ok()
{
  inputList =  listWidget->selectedItems();
  //separator
  if(radio1->isChecked())
    m_separator = (";");
  if(radio2->isChecked())
    m_separator = (" ");
  if(radio3->isChecked())
    m_separator = ("\t");
  if(radio4->isChecked())
    m_separator = sep->text();

    //attributes
  if(CHB_points->isChecked())
    points = true;
  if(CHB_height->isChecked())
    height = true;
  if(CHB_bottomHeight->isChecked())
    bottomHeight = true;
  if(CHB_totalHeight->isChecked())
    totalHeight = true;
  if(CHB_length->isChecked())
    length = true;
  if(CHB_width->isChecked())
    width = true;
  if(CHB_positionDev->isChecked())
    positionDev = true;
  if(CHB_positionXYZ->isChecked())
     positionXYZ = true;
  if(CHB_volVoxels->isChecked())
     volVoxels = true;
  if(CHB_volSections->isChecked())
     volSections = true;
  if(CHB_surface->isChecked())
     surface= true;
  if(CHB_vol3DCH->isChecked())
     vol3DCH = true;
  if(CHB_surface3DCH->isChecked())
   surf3DCH = true;
  if(CHB_sectionHeight->isChecked())
    sectionHeight = true;
  if(CHB_thresholdDist->isChecked())
    thresholdDist = true;
}
QGroupBox *ExportCrownAttr::attributesGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Tree crown attributes"));
  groupBox->setFlat(true);

  CHB_height = new QCheckBox(tr("Crown height"));
  connect(CHB_height, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_bottomHeight = new QCheckBox(tr("Crown bottom height"));
  connect(CHB_bottomHeight, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_totalHeight = new QCheckBox(tr("Crown total height"));
  connect(CHB_totalHeight, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_length = new QCheckBox(tr("Crown length"));
  connect(CHB_length, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_width = new QCheckBox(tr("Crown width"));
  connect(CHB_width, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_positionDev = new QCheckBox(tr("Crown position deviance\n distance and azimuth"));
  connect(CHB_positionDev, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_positionXYZ = new QCheckBox(tr("Crown position coordinates\n X Y Z"));
  connect(CHB_positionXYZ, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_points = new QCheckBox(tr("Point number"));
  connect(CHB_points, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_volVoxels = new QCheckBox(tr("Volume by voxels"));
  connect(CHB_volVoxels, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_volSections = new QCheckBox(tr("Volume by sections"));
  connect(CHB_volSections, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_vol3DCH = new QCheckBox(tr("Volume of 3D Convexhull"));
  connect(CHB_vol3DCH, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_surface = new QCheckBox(tr("Surface"));
  connect(CHB_surface, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_surface3DCH = new QCheckBox(tr("surface of 3D Convexhull"));
  connect(CHB_surface3DCH, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_sectionHeight = new QCheckBox(tr("Section height"));
  connect(CHB_sectionHeight, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));
  CHB_thresholdDist = new QCheckBox(tr("Threshold distance"));
  connect(CHB_thresholdDist, SIGNAL(stateChanged(int)), this, SLOT(all_attr(int)));

  CHB_all = new QCheckBox(tr("All attributes"));
  connect(CHB_all, SIGNAL(stateChanged(int)), this, SLOT(all_attributes(int)));

  QGridLayout *box = new QGridLayout();
  box->addWidget(CHB_points,1,1);
  box->addWidget(CHB_height,1,2);
  box->addWidget(CHB_bottomHeight,1,3);
  box->addWidget(CHB_totalHeight ,2,1);
  box->addWidget(CHB_length,2,2);
  box->addWidget(CHB_width,2,3);
  box->addWidget(CHB_positionDev,3,1);
  box->addWidget(CHB_positionXYZ,3,2);
  box->addWidget(CHB_volVoxels,3,3);
  box->addWidget(CHB_volSections,4,1);
  box->addWidget(CHB_surface,4,2);
  box->addWidget(CHB_vol3DCH,4,3);
  box->addWidget(CHB_surface3DCH,5,1);
  box->addWidget(CHB_sectionHeight,5,2);
  box->addWidget(CHB_thresholdDist,5,3);

  box->addWidget(CHB_all,6,1);
  groupBox->setLayout(box);
  return groupBox;
}
QGroupBox *ExportCrownAttr::separatorGroup()
{
  QGroupBox *groupBox = new QGroupBox(tr("Data separator"));
  groupBox->setFlat(true);

  radio1 = new QRadioButton(tr("Semicolon"));
  radio2 = new QRadioButton(tr("Space"));
  radio2->setChecked(true);
  radio3 = new QRadioButton(tr("Tabulator"));
  radio4 = new QRadioButton(tr("Other:"));
  sep = new QLineEdit(this);
  sep->setFixedWidth(60);
  sep->setReadOnly(true);
  sep->setStyleSheet("QLineEdit{background: lightGrey;}");

  connect(radio4, SIGNAL(toggled(bool)), this, SLOT(other_Separator(bool)));
  QHBoxLayout *hbox = new QHBoxLayout;
  hbox->addWidget(radio1);
  hbox->addWidget(radio2);
  hbox->addWidget(radio3);
  hbox->addWidget(radio4);
  hbox->addWidget(sep);
  hbox->setSpacing(10);
  hbox->addStretch(1);
  groupBox->setLayout(hbox);
  return groupBox;
}
void ExportCrownAttr::set_description(QString text)
{
  // text about function
  QLabel *label = new QLabel();
  label->setText(text);
  //label->setMaximumSize(180,300);
  label->setMinimumWidth(160);
  label->setWordWrap(true);
  label->setMargin(10);
  label->setAlignment(Qt::AlignJustify | Qt::AlignTop);
  //layout
  inputareaLayout->addWidget(label);

}
void ExportCrownAttr::set_trees(QStringList li)
{
  inputTrees->insertItems(0,li);
}
void ExportCrownAttr::setExistingDirectory()
{
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("Text file (*.txt)"));
  outputFile->setText(fileName);
}
void ExportCrownAttr::other_Separator(bool checked)
{
  if(checked == true)
  {
    sep->setReadOnly(false);
    sep->setStyleSheet("QLineEdit{background: white;}");
  }

  else
  {
    sep->setReadOnly(true);
    sep->setStyleSheet("QLineEdit{background: lightGrey;}");
    sep->setText("");
  }
}
void ExportCrownAttr::all_attributes(int checked)
{
  if(checked == 2)
  {
    CHB_height->setChecked(true);
    CHB_bottomHeight->setChecked(true);
    CHB_totalHeight->setChecked(true);
    CHB_length->setChecked(true);
    CHB_width->setChecked(true);
    CHB_positionDev->setChecked(true);
    CHB_positionXYZ->setChecked(true);
    CHB_points->setChecked(true);
    CHB_volVoxels->setChecked(true);
    CHB_volSections->setChecked(true);
    CHB_vol3DCH->setChecked(true);
    CHB_surface->setChecked(true);
    CHB_surface3DCH->setChecked(true);
    CHB_sectionHeight->setChecked(true);
    CHB_thresholdDist->setChecked(true);
  }
}
void ExportCrownAttr::all_attr(int checked)
{
  if(checked == 0)
  {
    CHB_all->setChecked(false);
  }
}
void ExportCrownAttr::set_list(QStringList li)
{
  listWidget->insertItems(0,li);
  listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
  listWidget->setSortingEnabled(true);
}
QList<QString> ExportCrownAttr:: get_inputList()
{
  QList<QString> result;
  for(int i=0; i < inputList.size(); i++)
    result.push_back( inputList.at(i)->text() );

  return result;
}


ExportQSMDialog::ExportQSMDialog(QWidget *parent)
: QDialog(parent)
{
    DialogSize(500,300);
    //buttons
    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    
    treeLabel = new QLabel();
    treeLabel->setText("Select tree name:");
    listWidget= new QListWidget(this);
    
    fileLabel = new QLabel();
    fileLabel->setText("Enter name of the file you want to save results:");
    outputFile = new QLineEdit;
    outputFile->setText("C:\\QSM_atributes.txt");
    outputFile->setReadOnly(true);
    directoryButton = new QPushButton(tr("Browse"));
    
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));
    
    DialogLayout();
}
ExportQSMDialog::ExportQSMDialog( QStringList nameList, QWidget *parent )
: QDialog(parent)
{
    treeLabel = new QLabel();
    treeLabel->setText("Select tree name:");
    listWidget= new QListWidget(this);
    
    fileLabel = new QLabel();
    fileLabel->setText("Enter name of the file you want to save results:");
    outputFile = new QLineEdit;
    outputFile->setText("C:\\crown_atributes.txt");
    outputFile->setReadOnly(true);
    directoryButton = new QPushButton(tr("Browse"));
    
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));
    
    
    DialogLayout();
}
ExportQSMDialog::~ExportQSMDialog()
{
    
}
void ExportQSMDialog::DialogSize(int w, int h )
{
    resize(w, h);
}
void ExportQSMDialog::DialogLayout()
{
    // buttons
    buttontLayout = new QHBoxLayout();
    buttontLayout->setGeometry(QRect(0,0,400,50));
    buttontLayout->setAlignment(Qt::AlignRight);
    buttontLayout->setSpacing(10);
    buttontLayout->addWidget(buttonBox );
    // tree selection
    treeLayout = new QVBoxLayout();
    treeLayout->addWidget(treeLabel);
    treeLayout->addWidget(listWidget);
    treeLayout->setSpacing(5);
    
    //file selection
    fileLayout = new QGridLayout();
    fileLayout->addWidget(fileLabel,1,0,1,2);
    fileLayout->addWidget(outputFile,2,1);
    fileLayout->addWidget(directoryButton,2,2);
    fileLayout->setSpacing(5);
    
    //inputlayout of cloudnames and labels
    InputLayout = new QVBoxLayout();
    InputLayout->addLayout(treeLayout);
    InputLayout->addLayout(fileLayout);
    InputLayout->addWidget(separatorGroup());
    InputLayout->addWidget(attributesGroup());
    InputLayout->setSpacing(10);
    // input of cloud and main label
    inputareaLayout = new QHBoxLayout();
    inputareaLayout->addLayout(InputLayout);
    
    //mainlayout
    mainLayout = new QVBoxLayout();
    mainLayout->addLayout(inputareaLayout);
    mainLayout->addLayout(buttontLayout);
    setLayout(mainLayout);
}
QString ExportQSMDialog::getSeparator()
{
    return m_separator;
}
QGroupBox *ExportQSMDialog::separatorGroup()
{
    QGroupBox *groupBox = new QGroupBox(tr("Data separator"));
    groupBox->setFlat(true);
    
    radio1 = new QRadioButton(tr("Semicolon"));
    radio2 = new QRadioButton(tr("Space"));
    radio2->setChecked(true);
    radio3 = new QRadioButton(tr("Tabulator"));
    radio4 = new QRadioButton(tr("Other:"));
    sep = new QLineEdit(this);
    sep->setFixedWidth(60);
    sep->setReadOnly(true);
    sep->setStyleSheet("QLineEdit{background: lightGrey;}");
    
    connect(radio4, SIGNAL(toggled(bool)), this, SLOT(otherSeparator(bool)));
    QHBoxLayout *hbox = new QHBoxLayout;
    hbox->addWidget(radio1);
    hbox->addWidget(radio2);
    hbox->addWidget(radio3);
    hbox->addWidget(radio4);
    hbox->addWidget(sep);
    hbox->setSpacing(10);
    hbox->addStretch(1);
    groupBox->setLayout(hbox);
    return groupBox;
}
QGroupBox *ExportQSMDialog::attributesGroup()
{
    QGroupBox *groupBox = new QGroupBox(tr("Tree Files"));
    groupBox->setFlat(true);
    
    CHB_trees = new QCheckBox(tr("Tree information"));
    connect(CHB_trees, SIGNAL(stateChanged(int)), this, SLOT(allAttr(int)));
    CHB_profile = new QCheckBox(tr("Tree profile information"));
    connect(CHB_profile, SIGNAL(stateChanged(int)), this, SLOT(allAttr(int)));
    CHB_sortiments = new QCheckBox(tr("Tree sortiments"));
    connect(CHB_sortiments, SIGNAL(stateChanged(int)), this, SLOT(allAttr(int)));
    CHB_branches = new QCheckBox(tr("Tree branches information"));
    connect(CHB_branches, SIGNAL(stateChanged(int)), this, SLOT(allAttr(int)));
    CHB_all = new QCheckBox(tr("All attributes"));
    connect(CHB_all, SIGNAL(stateChanged(int)), this, SLOT(allAttributes(int)));
    
    QGridLayout *box = new QGridLayout();
    box->addWidget(CHB_all,1,1);
    box->addWidget(CHB_trees,1,2);
    box->addWidget(CHB_profile,1,3);
    box->addWidget(CHB_branches ,2,1);
    box->addWidget(CHB_sortiments,2,2);

    groupBox->setLayout(box);
    return groupBox;
}
void ExportQSMDialog::setDescription(QString text)
{
    // text about function
    QLabel *label = new QLabel();
    label->setText(text);
    //label->setMaximumSize(180,300);
    label->setMinimumWidth(160);
    label->setWordWrap(true);
    label->setMargin(10);
    label->setAlignment(Qt::AlignJustify | Qt::AlignTop);
    //layout
    inputareaLayout->addWidget(label);
    
}
void ExportQSMDialog::setList(QStringList li)
{
    listWidget->insertItems(0,li);
    listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
    listWidget->setSortingEnabled(true);
}

void ExportQSMDialog::setExistingDirectory()
{
    QString fileName =QFileDialog::getExistingDirectory(this, tr("Open Directory"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    outputFile->setText(fileName);
}
void ExportQSMDialog::setPrefix(QString label, QString example)
{
    //output cloud
    QLabel *labelOC1 = new QLabel();
    labelOC1 ->setText(label);
    prefixLine = new QLineEdit;

    labelOC1 ->setBuddy(prefixLine);
    prefixLine->setText(example);
    //layout
    InputLayout->addWidget(labelOC1);
    InputLayout->addWidget(prefixLine);
}
void ExportQSMDialog::ok()
{
    // selected trees
    inputList =  listWidget->selectedItems();
    //separator
    if(radio1->isChecked())
        m_separator = (";");
    if(radio2->isChecked())
        m_separator = (" ");
    if(radio3->isChecked())
        m_separator = ("\t");
    if(radio4->isChecked())
        m_separator = sep->text();
    //attributes
    if(CHB_sortiments->isChecked())
        m_sortiments = true;
    if(CHB_profile->isChecked())
        m_profile = true;
    if(CHB_trees->isChecked())
        m_trees = true;
    if(CHB_branches->isChecked())
        m_branches = true;
    // prefix
    m_prefix = prefixLine->text();
    
}
void ExportQSMDialog::validate(QString)
{
    // validate path
}
void ExportQSMDialog::otherSeparator(bool checked)
{
    if(checked == true)
    {
        sep->setReadOnly(false);
        sep->setStyleSheet("QLineEdit{background: white;}");
    }
    
    else
    {
        sep->setReadOnly(true);
        sep->setStyleSheet("QLineEdit{background: lightGrey;}");
        sep->setText("");
    }
}
void ExportQSMDialog::allFiles(int checked)
{
    
}
void ExportQSMDialog::allAttr(int checked)
{
    if(checked == 0)
    {
        CHB_all->setChecked(false);
    }
}
void ExportQSMDialog::allAttributes(int checked)
{
    if(checked == 2)
    {
        CHB_trees->setChecked(true);
        CHB_profile->setChecked(true);
        CHB_branches->setChecked(true);
        CHB_sortiments->setChecked(true);
    }
}

bool ExportQSMDialog::getTrees()
{
    return m_trees;
}
bool ExportQSMDialog::getBranches()
{
    return m_branches;
}
bool ExportQSMDialog::getSortiments()
{
    return m_sortiments;
}
bool ExportQSMDialog::getProfile()
{
    return m_profile;
}
QString ExportQSMDialog::getPath()
{
    return outputFile->text();
}
QList<QString> ExportQSMDialog::getInputList()
{
    QList<QString> result;
    for(int i=0; i < inputList.size(); i++)
        result.push_back( inputList.at(i)->text() );
    
    return result;
}
QString ExportQSMDialog::getPrefix()
{
    return m_prefix;
}


ExportFeaturesDialog::ExportFeaturesDialog(QWidget *parent)
: QDialog(parent)
{
    DialogSize(500,300);
    //buttons
    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    
    fileLabel = new QLabel();
    fileLabel->setText("Enter directory you want to save results:");
    outputFile = new QLineEdit;
    outputFile->setText("Browse...");
    outputFile->setReadOnly(true);
    directoryButton = new QPushButton(tr("Browse"));
    
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));
    
    
    DialogLayout();
    
}
ExportFeaturesDialog::ExportFeaturesDialog( QStringList nameList, QWidget *parent )
: QDialog(parent)
{
    // neměl by se spouštět
    fileLabel = new QLabel();
    fileLabel->setText("Enter name of the file you want to save results:");
    outputFile = new QLineEdit;
    outputFile->setText("Browse...");
    outputFile->setReadOnly(true);
    directoryButton = new QPushButton(tr("Browse"));
    
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(ok()));
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    connect(directoryButton, SIGNAL(clicked()), this, SLOT(setExistingDirectory()));
    
    
    DialogLayout();
}
ExportFeaturesDialog::~ExportFeaturesDialog()
{
    
}
void ExportFeaturesDialog::DialogSize(int w, int h )
{
    resize(w, h);
}
void ExportFeaturesDialog::DialogLayout()
{
    // buttons
    buttontLayout = new QHBoxLayout();
    buttontLayout->setGeometry(QRect(0,0,400,50));
    buttontLayout->setAlignment(Qt::AlignRight);
    buttontLayout->setSpacing(10);
    buttontLayout->addWidget(buttonBox );
    // prefix
    QLabel *labelOC1 = new QLabel();
    labelOC1 ->setText("set prefix of the created files: ");
    prefixLine = new QLineEdit;
    labelOC1 ->setBuddy(prefixLine);
    prefixLine->setText("features_");
    treeLayout = new QVBoxLayout();
    treeLayout->addWidget(labelOC1);
    treeLayout->addWidget(prefixLine);
    
    //file selection
    fileLayout = new QGridLayout();
    fileLayout->addWidget(fileLabel,1,0,1,2);
    fileLayout->addWidget(outputFile,2,1);
    fileLayout->addWidget(directoryButton,2,2);
    fileLayout->setSpacing(5);
    
    //inputlayout of cloudnames and labels
    InputLayout = new QVBoxLayout();
    InputLayout->addLayout(treeLayout);
    InputLayout->addLayout(fileLayout);
    InputLayout->addWidget(separatorGroup());
    InputLayout->addWidget(attributesGroup());
    InputLayout->setSpacing(10);
    // input of cloud and main label
    inputareaLayout = new QHBoxLayout();
    inputareaLayout->addLayout(InputLayout);
    
    //mainlayout
    mainLayout = new QVBoxLayout();
    mainLayout->addLayout(inputareaLayout);
    mainLayout->addLayout(buttontLayout);
    setLayout(mainLayout);
}
QString ExportFeaturesDialog::getSeparator()
{
    return m_separator;
}
QGroupBox *ExportFeaturesDialog::separatorGroup()
{
    QGroupBox *groupBox = new QGroupBox(tr("Data separator"));
    groupBox->setFlat(true);
    
    radio1 = new QRadioButton(tr("Semicolon"));
    radio2 = new QRadioButton(tr("Space"));
    radio2->setChecked(true);
    radio3 = new QRadioButton(tr("Tabulator"));
    radio4 = new QRadioButton(tr("Other:"));
    sep = new QLineEdit(this);
    sep->setFixedWidth(60);
    sep->setReadOnly(true);
    sep->setStyleSheet("QLineEdit{background: lightGrey;}");
    
    connect(radio4, SIGNAL(toggled(bool)), this, SLOT(otherSeparator(bool)));
    QHBoxLayout *hbox = new QHBoxLayout;
    hbox->addWidget(radio1);
    hbox->addWidget(radio2);
    hbox->addWidget(radio3);
    hbox->addWidget(radio4);
    hbox->addWidget(sep);
    hbox->setSpacing(10);
    hbox->addStretch(1);
    groupBox->setLayout(hbox);
    return groupBox;
}
QGroupBox *ExportFeaturesDialog::attributesGroup()
{
    QGroupBox *groupBox = new QGroupBox(tr("Feature attributes"));
    groupBox->setFlat(true);
    
    CHB_attributeFile = new QCheckBox(tr("Atribute file"));
    connect(CHB_attributeFile, SIGNAL(stateChanged(int)), this, SLOT(allAttr(int)));
    CHB_convexPolygon = new QCheckBox(tr("Convex polygon file"));
    connect(CHB_convexPolygon, SIGNAL(stateChanged(int)), this, SLOT(allAttr(int)));
    CHB_concavePolygon = new QCheckBox(tr("Concave polygon file"));
    connect(CHB_concavePolygon, SIGNAL(stateChanged(int)), this, SLOT(allAttr(int)));
    CHB_all = new QCheckBox(tr("All files"));
    connect(CHB_all, SIGNAL(stateChanged(int)), this, SLOT(allAttributes(int)));
    
    QGridLayout *box = new QGridLayout();
    box->addWidget(CHB_all,1,1);
    box->addWidget(CHB_attributeFile ,2,1);
    box->addWidget(CHB_convexPolygon,2,2);
    box->addWidget(CHB_concavePolygon,2,3);


    groupBox->setLayout(box);
    return groupBox;
}
void ExportFeaturesDialog::setDescription(QString text)
{
    // text about function
    QLabel *label = new QLabel();
    label->setText(text);
    //label->setMaximumSize(180,300);
    label->setMinimumWidth(160);
    label->setWordWrap(true);
    label->setMargin(10);
    label->setAlignment(Qt::AlignJustify | Qt::AlignTop);
    //layout
    inputareaLayout->addWidget(label);
    
}
void ExportFeaturesDialog::setList(QStringList li)
{
    listWidget->insertItems(0,li);
    listWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);
    listWidget->setSortingEnabled(true);
}

void ExportFeaturesDialog::setExistingDirectory()
{
    QString fileName =QFileDialog::getExistingDirectory(this, tr("Open Directory"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    outputFile->setText(fileName);
}
void ExportFeaturesDialog::setPrefix(QString label, QString example)
{
    //output cloud
    
    //layout
    
}
void ExportFeaturesDialog::ok()
{

    //separator
    if(radio1->isChecked())
        m_separator = (";");
    if(radio2->isChecked())
        m_separator = (" ");
    if(radio3->isChecked())
        m_separator = ("\t");
    if(radio4->isChecked())
        m_separator = sep->text();
    //attributes
    if(CHB_concavePolygon->isChecked())
        m_concavePoylgonFile= true;
    if(CHB_convexPolygon->isChecked())
        m_convexPolygonFile= true;
    if(CHB_attributeFile->isChecked())
        m_featuresAttributesFile = true;
    // prefix
    m_prefix = prefixLine->text();
    //path
    m_path = outputFile->text();
}
void ExportFeaturesDialog::validate(QString)
{
    // validate path
}
void ExportFeaturesDialog::otherSeparator(bool checked)
{
    if(checked == true)
    {
        sep->setReadOnly(false);
        sep->setStyleSheet("QLineEdit{background: white;}");
    }
    
    else
    {
        sep->setReadOnly(true);
        sep->setStyleSheet("QLineEdit{background: lightGrey;}");
        sep->setText("");
    }
}
void ExportFeaturesDialog::allFiles(int checked)
{
    
}
void ExportFeaturesDialog::allAttr(int checked)
{
    if(checked == 0)
    {
        CHB_all->setChecked(false);
    }
}
void ExportFeaturesDialog::allAttributes(int checked)
{
    if(checked == 2)
    {
        CHB_attributeFile->setChecked(true);
        CHB_convexPolygon->setChecked(true);
        CHB_concavePolygon->setChecked(true);
    }
}

bool ExportFeaturesDialog::getFeatureFile()
{
    return m_featuresAttributesFile;
}
bool ExportFeaturesDialog::getConcavePolygonFile()
{
    return m_concavePoylgonFile;
}
bool ExportFeaturesDialog::getConvexPolygonFile()
{
    return m_convexPolygonFile;
}

QString ExportFeaturesDialog::getPath()
{
    //return outputFile->text();
    return m_path;
}
QList<QString> ExportFeaturesDialog::getInputList()
{
    QList<QString> result;
    for(int i=0; i < inputList.size(); i++)
        result.push_back( inputList.at(i)->text() );
    
    return result;
}
QString ExportFeaturesDialog::getPrefix()
{
    return m_prefix;
}
//// THREAD



////VISUALIZER
Visualizer::Visualizer()
{
  PCLVisualizer ("3D Viewer", false);





}
void Visualizer::coordinateMark()
{

// axes coordinate system




//
//  if ( !axes_widget_ )
//    {
//    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New ();
//    axes->SetTotalLength(2.1,2.1,2.1);
//    axes->SetCylinderRadius(0.5);
//
//    axes_widget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New ();
//    axes_widget_->SetOutlineColor (0.93, 0.57, 0.13);
//    axes_widget_->SetOrientationMarker (axes);
//    axes_widget_->SetInteractor (interactor_);
//    axes_widget_->SetViewport (0., 0., 0.15, 0.15);
//    axes_widget_->SetEnabled (true);
//    axes_widget_->InteractiveOff ();
//  }
//  else
//  {
//    axes_widget_->SetEnabled (true);
//    pcl::console::print_warn (stderr, "Orientation Widget Axes already exists, just enabling it");
//  }
}
//Vis::Vis()
//{
//   m_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
//
//  // Setup renderer
//  m_renderer = vtkSmartPointer<vtkRenderer>::New();
//  m_renderer->SetBackground(0.1, 0.2, 0.4);
//  m_renderWindow->AddRenderer(m_renderer);
//
//}
//vtkSmartPointer<vtkRenderWindow> Vis::get_renderWindow()
//{
//  return m_renderWindow;
//}
//bool Vis::addCloud(Cloud cl)
//{
//  vtkSmartPointer<vtkPoints> points =
//  vtkSmartPointer<vtkPoints>::New();
//
//  for(int i = 0; cl.get_Cloud()->points.size() > i; i++)
//  {
//    points->InsertNextPoint(cl.get_Cloud()->points.at(i).x, cl.get_Cloud()->points.at(i).y, cl.get_Cloud()->points.at(i).z);
//  }
//  vtkSmartPointer<vtkPolyData> polydata =
//    vtkSmartPointer<vtkPolyData>::New();
//
//  polydata->SetPoints(points);
//
//  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
//    vtkSmartPointer<vtkVertexGlyphFilter>::New();
//#if VTK_MAJOR_VERSION <= 5
//  glyphFilter->SetInputConnection(polydata->GetProducerPort());
//#else
//  glyphFilter->SetInputData(polydata);
//#endif
//  glyphFilter->Update();
//
//  // Visualize
//  // Create a mapper and actor
//  vtkSmartPointer<vtkPolyDataMapper> mapper =
//    vtkSmartPointer<vtkPolyDataMapper>::New();
//  mapper->SetInputConnection(glyphFilter->GetOutputPort());
//
//  vtkSmartPointer<vtkActor> actor =
//    vtkSmartPointer<vtkActor>::New();
//  actor->SetMapper(mapper);
//
//
//  vtkSmartPointer<vtkWin32RenderWindowInteractor> renderWindowInteractor =
//    vtkSmartPointer<vtkWin32RenderWindowInteractor>::New();
//  renderWindowInteractor->SetRenderWindow(m_renderWindow);
//
//  vtkSmartPointer<vtkInteractorStyleTrackballActor> style =
//    vtkSmartPointer<vtkInteractorStyleTrackballActor>::New();
//
//  renderWindowInteractor->SetInteractorStyle( style );
//
//  renderWindowInteractor->Start();
//
//
//  m_renderer->AddActor(actor);
//  m_renderer->SetBackground(0.1, 0.2, 0.4);
//  m_renderWindow->Render();
//
//}
//void Vis::addCylinder()
//{
//  vtkCylinderSource *cylinder = vtkCylinderSource::New();
//  cylinder->SetResolution(8);
//
//  // The mapper is responsible for pushing the geometry into the graphics
//  // library. It may also do color mapping, if scalars or other attributes
//  // are defined.
//  //
//  vtkPolyDataMapper *cylinderMapper = vtkPolyDataMapper::New();
//  cylinderMapper->SetInputConnection(cylinder->GetOutputPort());
//
//  // The actor is a grouping mechanism: besides the geometry (mapper), it
//  // also has a property, transformation matrix, and/or texture map.
//  // Here we set its color and rotate it -22.5 degrees.
//  vtkActor *cylinderActor = vtkActor::New();
//  cylinderActor->SetMapper(cylinderMapper);
//  cylinderActor->GetProperty()->SetColor(1.0000, 0.3882, 0.2784);
//  cylinderActor->RotateX(30.0);
//  cylinderActor->RotateY(-45.0);
//
//  m_renderer->AddActor(cylinderActor);
//  m_renderer->SetBackground(0.1, 0.2, 0.4);
//  m_renderWindow->Render();
//
//
//  // We'll zoom in a little by accessing the camera and invoking a "Zoom"
//  // method on it.
//  m_renderer->ResetCamera();
//}
