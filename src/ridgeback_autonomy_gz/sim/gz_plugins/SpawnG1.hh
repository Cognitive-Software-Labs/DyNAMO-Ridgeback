#ifndef SPAWN_G1_HH
#define SPAWN_G1_HH

#include <gz/gui/Plugin.hh>

namespace spawn_g1
{
  class SpawnG1 : public gz::gui::Plugin
  {
    Q_OBJECT

  public:
    SpawnG1();
    ~SpawnG1() override = default;
    void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  public slots:
    void OnSpawn();

  private:
    std::string sdfString_;
  };
}

#endif
