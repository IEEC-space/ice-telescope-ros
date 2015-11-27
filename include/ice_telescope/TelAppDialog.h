#include <QDialog>
#include <QString>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string.h>
#endif

class QAction;
class QDialogButtonBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QMenu;
class QMenuBar;
class QPushButton;
class QTextEdit;
class QCheckBox;
class QLineEdit;

namespace telsys 
{

class TelAppDialog : public QDialog
{
  Q_OBJECT
public:
  TelAppDialog();
  virtual ~TelAppDialog();

signals:
  void updateLog(QString str);

private slots:
  void actionDome(QString action);
  void actionCCD(QString action);
  void actionTelescope(QString action);
  void updateUILog(QString str);

private:
  void createMenu();
  void createLogDisplay();
  void createHorizontalGroupBoxDome();
  void createGridGroupBoxCCD();
  void createGridGroupBoxTelescope();

  bool isOnlyInt(const char* str);
  bool isOnlyDouble(const char* str);

  void retryCallback(const std_msgs::String::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::ServiceClient domeClient;
  ros::ServiceClient ccdClient;
  ros::ServiceClient telescopeClient;
  ros::Subscriber telescopeSub;
  ros::Subscriber domeSub;
  ros::AsyncSpinner spinner;

  QMenuBar *menuBar;
  QMenu *fileMenu;
  QAction *exitAction;

  QGroupBox *logDisplayBox;
  QTextEdit *logTextDisplay;

  QGroupBox *horizontalGroupBoxDome;
  QGroupBox *gridGroupBoxCCD;
  QGroupBox *gridGroupBoxTelescope;

  QLineEdit *textEditCCDCapture;
  QLineEdit *textEditCCDSetTemp;
  QCheckBox *checkBoxCCDSetTemp;

  QLineEdit *textTelRA;
  QLineEdit *textTelDEC;
  QLineEdit *textTelMessierNum;
  QLineEdit *textTelStarNum;
  QLineEdit *textTelDeepSkyNum;
  QCheckBox *checkBoxTelFocus;
  QLineEdit *textTelLat;
  QLineEdit *textTelLong;

};

}