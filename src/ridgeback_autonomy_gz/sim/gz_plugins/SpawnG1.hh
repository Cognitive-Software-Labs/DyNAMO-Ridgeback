#ifndef SPAWN_G1_HH
#define SPAWN_G1_HH

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>

namespace spawn_g1
{
  class SpawnG1 : public gz::gui::Plugin
  {
    Q_OBJECT

  public:
    SpawnG1();
    ~SpawnG1() override = default;
    void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  signals:
    /// \brief Placement was not started because a G1 already exists.
    void spawnRefused();

  public slots:
    void OnSpawn();

  private:
    bool G1AlreadyInWorld();

    std::string sdfString_;
    std::string modelName_{"g1"};
    gz::transport::Node node_;
  };
}

#endif
