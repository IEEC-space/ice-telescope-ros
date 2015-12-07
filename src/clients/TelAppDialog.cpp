#include <QAction>
#include <QDialogButtonBox>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QSignalMapper>
#include <QCheckBox>

#include <ctime>
#include <iostream>
#include <stdlib.h>

#include "ice_telescope/TelAppDialog.h"
#include "ice_telescope/baader.h"
#include "ice_telescope/sbig.h"
#include "ice_telescope/meade.h"

#define OPEN_DOME 0
#define CLOSE_DOME 1
#define STATUS_DOME 2

namespace telsys 
{

TelAppDialog::TelAppDialog()
{
  createMenu();
  createLogDisplay();
  createHorizontalGroupBoxDome();
  createGridGroupBoxCCD();
  createGridGroupBoxTelescope();

  QGridLayout *mainLayout = new QGridLayout;
  mainLayout->setMenuBar(menuBar);
  mainLayout->setColumnMinimumWidth(1, 700);
  mainLayout->setColumnStretch(0, 10);
  mainLayout->setColumnStretch(1, 30);
  mainLayout->setRowStretch(0, 15);
  mainLayout->setRowStretch(1, 30);
  mainLayout->setRowStretch(2, 80);
  mainLayout->addWidget(logDisplayBox, 0, 1, 3, 1);
  mainLayout->addWidget(horizontalGroupBoxDome, 0, 0);
  mainLayout->addWidget(gridGroupBoxCCD, 1, 0);
  mainLayout->addWidget(gridGroupBoxTelescope, 2, 0);
  setLayout(mainLayout);

  setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowCloseButtonHint | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);
  setWindowTitle(tr("ICE Telescope"));

  domeClient = nh_.serviceClient<ice_telescope::baader>("baader_action");
  ccdClient = nh_.serviceClient<ice_telescope::sbig>("sbig_action");
  telescopeClient = nh_.serviceClient<ice_telescope::meade>("meade_action");
}

TelAppDialog::~TelAppDialog()
{
  
}

void TelAppDialog::createMenu()
{
  menuBar = new QMenuBar;

  fileMenu = new QMenu(tr("&File"), this);
  exitAction = fileMenu->addAction(tr("E&xit"));
  menuBar->addMenu(fileMenu);

  connect(exitAction, SIGNAL(triggered()), this, SLOT(accept()));
}

void TelAppDialog::createLogDisplay()
{
  logDisplayBox = new QGroupBox(tr("Log"));
  QHBoxLayout *layout = new QHBoxLayout;

  logTextDisplay = new QTextEdit();
  logTextDisplay->setReadOnly(true);

  layout->addWidget(logTextDisplay);
  logDisplayBox->setLayout(layout);
}

void TelAppDialog::createHorizontalGroupBoxDome()
{
  int domeNumButtons = 3;
  horizontalGroupBoxDome = new QGroupBox(tr("Dome control"));
  QHBoxLayout *layout = new QHBoxLayout;
  QPushButton *buttonsDome[domeNumButtons];

  buttonsDome[OPEN_DOME] = new QPushButton(tr("OPEN"));
  buttonsDome[CLOSE_DOME] = new QPushButton(tr("CLOSE"));
  buttonsDome[STATUS_DOME] = new QPushButton(tr("STATUS"));

  for (int i = 0; i < domeNumButtons; ++i) {
    layout->addWidget(buttonsDome[i]);
  }

  horizontalGroupBoxDome->setLayout(layout);

  QSignalMapper *signalMapper = new QSignalMapper(this);
  signalMapper->setMapping(buttonsDome[OPEN_DOME], QString("open"));
  signalMapper->setMapping(buttonsDome[CLOSE_DOME], QString("close"));
  signalMapper->setMapping(buttonsDome[STATUS_DOME], QString("status"));

  connect(buttonsDome[OPEN_DOME], SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonsDome[CLOSE_DOME], SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonsDome[STATUS_DOME], SIGNAL(clicked()), signalMapper, SLOT(map()));

  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(actionDome(QString)));
}

void TelAppDialog::createGridGroupBoxCCD()
{
  gridGroupBoxCCD = new QGroupBox(tr("CCD control"));
  QGridLayout *layout = new QGridLayout;

  QPushButton *buttonCCDCapture = new QPushButton(tr("CAPTURE"));
  QPushButton *buttonCCDSetTemp = new QPushButton(tr("SET TEMP."));
  QPushButton *buttonCCDGetTemp = new QPushButton(tr("GET TEMP."));

  textEditCCDCapture = new QLineEdit();
  textEditCCDCapture->setPlaceholderText(tr("Exposure in seconds"));
  textEditCCDSetTemp = new QLineEdit();
  textEditCCDSetTemp->setPlaceholderText(tr("Temperature in \260C"));
  checkBoxCCDSetTemp = new QCheckBox(tr("Enabled"));
  checkBoxCCDSetTemp->setChecked(true);

  layout->addWidget(buttonCCDCapture, 0, 0);
  layout->addWidget(textEditCCDCapture, 0, 1, 1, 2);
  layout->addWidget(buttonCCDSetTemp, 1, 0);
  layout->addWidget(textEditCCDSetTemp, 1, 1);
  layout->addWidget(checkBoxCCDSetTemp, 1, 2);
  layout->addWidget(buttonCCDGetTemp, 2, 0);

  gridGroupBoxCCD->setLayout(layout);

  QSignalMapper *signalMapper = new QSignalMapper(this);
  signalMapper->setMapping(buttonCCDCapture, QString("capture"));
  signalMapper->setMapping(buttonCCDSetTemp, QString("settemp"));
  signalMapper->setMapping(buttonCCDGetTemp, QString("gettemp"));

  connect(buttonCCDCapture, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonCCDSetTemp, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonCCDGetTemp, SIGNAL(clicked()), signalMapper, SLOT(map()));

  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(actionCCD(QString)));
}

void TelAppDialog::createGridGroupBoxTelescope()
{
  gridGroupBoxTelescope = new QGroupBox(tr("Telescope control"));
  QGridLayout *layout = new QGridLayout;

  QPushButton *buttonTelGoTo = new QPushButton(tr("GOTO"));
  QPushButton *buttonTelMessier = new QPushButton(tr("MESSIER"));
  QPushButton *buttonTelStar = new QPushButton(tr("STAR"));
  QPushButton *buttonTelDeepSky = new QPushButton(tr("DEEPSKY"));
  QPushButton *buttonTelFocus = new QPushButton(tr("FOCUS"));
  QPushButton *buttonTelSetLatLon = new QPushButton(tr("SET LAT./LONG."));
  QPushButton *buttonTelGPS = new QPushButton(tr("GPS"));
  QPushButton *buttonTelGetObjRD = new QPushButton(tr("GET OBJ. RA/DEC"));
  QPushButton *buttonTelGetTelRD = new QPushButton(tr("GET TEL. RA/DEC"));
  QPushButton *buttonTelGetDateT = new QPushButton(tr("GET DATE/TIME"));
  QPushButton *buttonTelSetDateT = new QPushButton(tr("SET DATE/TIME"));
  QPushButton *buttonTelGetLatLon = new QPushButton(tr("GET LAT./LONG."));

  textTelRA = new QLineEdit();
  textTelRA->setPlaceholderText(tr("Right Ascension"));
  textTelDEC = new QLineEdit();
  textTelDEC->setPlaceholderText(tr("Declination"));
  textTelMessierNum = new QLineEdit();
  textTelMessierNum->setPlaceholderText(tr("Object catalog number"));
  textTelStarNum = new QLineEdit();
  textTelStarNum->setPlaceholderText(tr("Object catalog number"));
  textTelDeepSkyNum = new QLineEdit();
  textTelDeepSkyNum->setPlaceholderText(tr("Object catalog number"));
  checkBoxTelFocus = new QCheckBox(tr("Checked: In -- Unchecked: Out"));
  textTelLat = new QLineEdit();
  textTelLat->setPlaceholderText(tr("Site latitude"));
  textTelLong = new QLineEdit();
  textTelLong->setPlaceholderText(tr("Site longitude"));

  layout->addWidget(buttonTelGoTo, 0, 0);
  layout->addWidget(textTelRA, 0, 1);
  layout->addWidget(textTelDEC, 0, 2);
  layout->addWidget(buttonTelMessier, 1, 0);
  layout->addWidget(textTelMessierNum, 1, 1, 1, 2);
  layout->addWidget(buttonTelStar, 2, 0);
  layout->addWidget(textTelStarNum, 2, 1, 1, 2);
  layout->addWidget(buttonTelDeepSky, 3, 0);
  layout->addWidget(textTelDeepSkyNum, 3, 1, 1, 2);
  layout->addWidget(buttonTelFocus, 4, 0);
  layout->addWidget(checkBoxTelFocus, 4, 1, 1, 2);
  layout->addWidget(buttonTelSetLatLon, 5, 0);
  layout->addWidget(textTelLat, 5, 1);
  layout->addWidget(textTelLong, 5, 2);
  layout->addWidget(buttonTelGPS, 6, 0);
  layout->addWidget(buttonTelGetObjRD, 6, 1);
  layout->addWidget(buttonTelGetTelRD, 6, 2);
  layout->addWidget(buttonTelGetDateT, 7, 0);
  layout->addWidget(buttonTelSetDateT, 7, 1);
  layout->addWidget(buttonTelGetLatLon, 7, 2);

  gridGroupBoxTelescope->setLayout(layout);

  QSignalMapper *signalMapper = new QSignalMapper(this);
  signalMapper->setMapping(buttonTelGoTo, QString("goto"));
  signalMapper->setMapping(buttonTelMessier, QString("messier"));
  signalMapper->setMapping(buttonTelStar, QString("star"));
  signalMapper->setMapping(buttonTelDeepSky, QString("deepsky"));
  signalMapper->setMapping(buttonTelFocus, QString("focus"));
  signalMapper->setMapping(buttonTelSetLatLon, QString("setlatlon"));
  signalMapper->setMapping(buttonTelGPS, QString("gps"));
  signalMapper->setMapping(buttonTelGetObjRD, QString("getobjradec"));
  signalMapper->setMapping(buttonTelGetTelRD, QString("gettelradec"));
  signalMapper->setMapping(buttonTelGetDateT, QString("getdatetime"));
  signalMapper->setMapping(buttonTelSetDateT, QString("setdatetime"));
  signalMapper->setMapping(buttonTelGetLatLon, QString("getlatlon"));

  connect(buttonTelGoTo, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelMessier, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelStar, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelDeepSky, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelFocus, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelSetLatLon, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelGPS, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelGetObjRD, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelGetTelRD, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelGetDateT, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelSetDateT, SIGNAL(clicked()), signalMapper, SLOT(map()));
  connect(buttonTelGetLatLon, SIGNAL(clicked()), signalMapper, SLOT(map()));

  connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(actionTelescope(QString)));
}

void TelAppDialog::actionDome(QString action)
{
  ice_telescope::baader srv;
  srv.request.baader_action = action.toStdString();

  time_t now = time(0);
  std::string dt(ctime(&now));
  std::stringstream s;

  if(domeClient.call(srv))
  {
    s << "[" << dt.substr(0, dt.length()-1) << "]: " << srv.response.baader_response.c_str() << std::endl;

    if (srv.response.baader_error)
    {
      ROS_ERROR(srv.response.baader_response.c_str());
    }
    else
    {
      ROS_INFO(srv.response.baader_response.c_str());
    }
  }
  else
  {
    s << "[" << dt.substr(0, dt.length()-1) << "]: Failed to call dome service" << std::endl;
    
    ROS_ERROR("Failed to call dome service");
  }

  logTextDisplay->moveCursor(QTextCursor::Start);
  logTextDisplay->insertPlainText(s.str().c_str());
}

void TelAppDialog::actionCCD(QString action)
{
  ice_telescope::sbig srv;
  time_t now = time(0);
  std::string dt(ctime(&now));
  std::stringstream s;

  srv.request.sbig_action = action.toStdString();

  if(srv.request.sbig_action == "capture")
  {
    srv.request.file_path = "/home/pi/fits/";
    srv.request.fits_file = true;
    srv.request.img_count = 1;
    srv.request.lf_img = true;
    srv.request.readout_mode = 0;
    srv.request.top = 0;
    srv.request.left = 0;
    srv.request.width = 0;
    srv.request.height = 0;
    srv.request.fast_readout = true;
    srv.request.dual_readout_channel = true;

    if(isOnlyDouble(textEditCCDCapture->text().toStdString().c_str()))
    {
      srv.request.exp_time = textEditCCDCapture->text().toDouble();
    }
    else
    {
      s << "[" << dt.substr(0, dt.length()-1) << "]: Incorrect exposure time value" << std::endl;
      logTextDisplay->moveCursor(QTextCursor::Start);
      logTextDisplay->insertPlainText(s.str().c_str()); 
      return; 
    }
  }
  else if(srv.request.sbig_action == "settemp")
  {
    if(isOnlyDouble(textEditCCDSetTemp->text().toStdString().c_str()))
    {
      srv.request.temperature = textEditCCDSetTemp->text().toDouble();
    }
    else
    {
      s << "[" << dt.substr(0, dt.length()-1) << "]: Incorrect temperature value" << std::endl;
      logTextDisplay->moveCursor(QTextCursor::Start);
      logTextDisplay->insertPlainText(s.str().c_str()); 
      return; 
    }
    srv.request.temp_enable = checkBoxCCDSetTemp->isChecked();
  }

  if(ccdClient.call(srv))
  {
    s << "[" << dt.substr(0, dt.length()-1) << "]: " << srv.response.sbig_response.c_str() << std::endl;

    if(srv.response.sbig_error)
    {
      ROS_ERROR(srv.response.sbig_response.c_str());
    }
    else
    {
      ROS_INFO(srv.response.sbig_response.c_str());
    }
  }
  else
  {
    s << "[" << dt.substr(0, dt.length()-1) << "]: Failed to call ccd service" << std::endl;

    ROS_ERROR("Failed to call ccd service");
  }

  logTextDisplay->moveCursor(QTextCursor::Start);
  logTextDisplay->insertPlainText(s.str().c_str());
}

void TelAppDialog::actionTelescope(QString action)
{
  ice_telescope::meade srv;
  time_t now = time(0);
  std::string dt(ctime(&now));
  std::stringstream s;

  srv.request.meade_action = action.toStdString();

  if(srv.request.meade_action == "goto")
  {
    if(isOnlyDouble(textTelRA->text().toStdString().c_str()) && isOnlyDouble(textTelDEC->text().toStdString().c_str()))
    {
      srv.request.ra = textTelRA->text().toDouble();
      srv.request.dec = textTelDEC->text().toDouble();
    }
    else
    {
      s << "[" << dt.substr(0, dt.length()-1) << "]: Incorrect target values" << std::endl;
      logTextDisplay->moveCursor(QTextCursor::Start);
      logTextDisplay->insertPlainText(s.str().c_str()); 
      return; 
    }
  }
  else if(srv.request.meade_action == "messier")
  {
    if(isOnlyInt(textTelMessierNum->text().toStdString().c_str()))
    {
      srv.request.object_num = textTelMessierNum->text().toInt();
    }
    else
    {
      s << "[" << dt.substr(0, dt.length()-1) << "]: Incorrect Messier catalog number" << std::endl;
      logTextDisplay->moveCursor(QTextCursor::Start);
      logTextDisplay->insertPlainText(s.str().c_str()); 
      return; 
    }
  }
  else if(srv.request.meade_action == "star")
  {
    if(isOnlyInt(textTelStarNum->text().toStdString().c_str()))
    {
      srv.request.object_num = textTelStarNum->text().toInt();
    }
    else
    {
      s << "[" << dt.substr(0, dt.length()-1) << "]: Incorrect Star catalog number" << std::endl;
      logTextDisplay->moveCursor(QTextCursor::Start);
      logTextDisplay->insertPlainText(s.str().c_str()); 
      return; 
    }
  }
  else if(srv.request.meade_action == "deepsky")
  {
    if(isOnlyInt(textTelDeepSkyNum->text().toStdString().c_str()))
    {
      srv.request.object_num = textTelDeepSkyNum->text().toInt();
    }
    else
    {
      s << "[" << dt.substr(0, dt.length()-1) << "]: Incorrect DeepSky catalog number" << std::endl;
      logTextDisplay->moveCursor(QTextCursor::Start);
      logTextDisplay->insertPlainText(s.str().c_str()); 
      return; 
    }
  }
  else if(srv.request.meade_action == "focus")
  {
    if(checkBoxTelFocus->isChecked())
    {
      srv.request.focus_motion = "in";
    }
    else
    {
      srv.request.focus_motion = "out";
    }
  }
  else if(srv.request.meade_action == "setlatlon")
  {
    if(isOnlyDouble(textTelLat->text().toStdString().c_str()) && isOnlyDouble(textTelLong->text().toStdString().c_str()))
    {
      srv.request.lat = textTelLat->text().toDouble();
      srv.request.lon = textTelLong->text().toDouble();
    }
    else
    {
      s << "[" << dt.substr(0, dt.length()-1) << "]: Incorrect coordinate values" << std::endl;
      logTextDisplay->moveCursor(QTextCursor::Start);
      logTextDisplay->insertPlainText(s.str().c_str()); 
      return; 
    }
  }

  if(telescopeClient.call(srv))
  {
    s << "[" << dt.substr(0, dt.length()-1) << "]: " << srv.response.meade_response.c_str() << std::endl;

    if (srv.response.meade_error)
    {
      ROS_ERROR(srv.response.meade_response.c_str());
    }
    else
    {
      ROS_INFO(srv.response.meade_response.c_str());
    }
  }
  else
  {
    s << "[" << dt.substr(0, dt.length()-1) << "]: Failed to call telescope service" << std::endl;

    ROS_ERROR("Failed to call telescope service");
  }

  logTextDisplay->moveCursor(QTextCursor::Start);
  logTextDisplay->insertPlainText(s.str().c_str());
}

bool TelAppDialog::isOnlyInt(const char* str)
{
  char* endptr = 0;
  strtol(str, &endptr, 10);

  return !(*endptr != '\0' || endptr == str);
}

bool TelAppDialog::isOnlyDouble(const char* str)
{
  char* endptr = 0;
  strtod(str, &endptr);

  return !(*endptr != '\0' || endptr == str);
}

}