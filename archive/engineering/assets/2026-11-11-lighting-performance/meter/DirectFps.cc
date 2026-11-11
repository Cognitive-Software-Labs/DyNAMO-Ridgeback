#include "DirectFps.hh"
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/plugin/Register.hh>
#include <chrono>
#include <cstdlib>
void DirectFps::LoadConfig(const tinyxml2::XMLElement *) {
 stream.open(std::getenv("DIRECT_FPS_OUTPUT")); stream.precision(17);
 gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
}
bool DirectFps::eventFilter(QObject *, QEvent *event) {
 if(event->type()==gz::gui::events::Render::kType) {
  stream << std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() << '\n'; stream.flush();
 }
 return false;
}
GZ_ADD_PLUGIN(DirectFps, gz::gui::Plugin)
