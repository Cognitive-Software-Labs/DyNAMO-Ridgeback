#include <gz/gui/Plugin.hh>
#include <fstream>
class DirectFps : public gz::gui::Plugin {
 Q_OBJECT
 public: void LoadConfig(const tinyxml2::XMLElement *) override;
 protected: bool eventFilter(QObject *, QEvent *) override;
 private: std::ofstream stream;
};
