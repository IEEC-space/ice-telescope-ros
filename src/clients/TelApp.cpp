#include <QApplication>
#include <ros/ros.h>
#include "ice_telescope/TelAppDialog.h"

class TelApp : public QApplication
{
public:
  ros::NodeHandlePtr nh_;

  TelApp(int& argc, char** argv)
    : QApplication(argc, argv)
  {
    ros::init(argc, argv, "ice_tel_client", ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle);
  }

  int exec()
  {
    telsys::TelAppDialog dialog;
    dialog.show();

    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  TelApp app(argc, argv);
  return app.exec();
}