#include "SpawnG1.hh"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <regex>
#include <vector>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/common/Filesystem.hh>

namespace spawn_g1
{

namespace
{
// Short enough not to stall the GUI noticeably on a button click.
constexpr unsigned int kSceneQueryTimeoutMs = 1000;
}

SpawnG1::SpawnG1() = default;

void SpawnG1::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Spawn G1";

  // Find model.sdf via GZ_SIM_RESOURCE_PATH
  std::string sdfPath;
  std::string modelDir;

  if (_pluginElem)
  {
    auto elem = _pluginElem->FirstChildElement("sdf_path");
    if (elem && elem->GetText())
      sdfPath = elem->GetText();
  }

  auto tryCandidate = [&](const std::string &candidate, const std::string &dir) {
    if (gz::common::isFile(candidate))
    {
      sdfPath = candidate;
      modelDir = dir;
      return true;
    }
    return false;
  };

  if (sdfPath.empty() || !gz::common::isFile(sdfPath))
  {
    const char *resourcePath = std::getenv("GZ_SIM_RESOURCE_PATH");
    if (resourcePath)
    {
      std::string paths(resourcePath);
      std::stringstream ss(paths);
      std::string path;
      while (std::getline(ss, path, ':'))
      {
        if (path.empty()) continue;
        if (tryCandidate(path + "/g1/model.sdf", path + "/g1/"))
          break;
      }
    }
  }

  if (sdfPath.empty() || !gz::common::isFile(sdfPath))
  {
    const char *pluginPath = std::getenv("GZ_GUI_PLUGIN_PATH");
    if (pluginPath)
    {
      std::string paths(pluginPath);
      std::stringstream ss(paths);
      std::string path;
      while (std::getline(ss, path, ':'))
      {
        if (path.empty()) continue;
        const std::vector<std::string> candidates = {
          path + "/../../share/ridgeback_autonomy_gz/sim/models/g1/model.sdf",
          path + "/../../../share/ridgeback_autonomy_gz/sim/models/g1/model.sdf",
        };
        for (const auto &candidate : candidates)
        {
          auto pos = candidate.rfind('/');
          const auto dir = pos == std::string::npos ? std::string() : candidate.substr(0, pos + 1);
          if (tryCandidate(candidate, dir))
            break;
        }
        if (!sdfPath.empty() && gz::common::isFile(sdfPath))
          break;
      }
    }
  }
  else
  {
    // Extract directory from explicit path
    auto pos = sdfPath.rfind('/');
    if (pos != std::string::npos)
      modelDir = sdfPath.substr(0, pos + 1);
  }

  if (!sdfPath.empty() && gz::common::isFile(sdfPath))
  {
    std::ifstream file(sdfPath);
    std::stringstream buf;
    buf << file.rdbuf();
    sdfString_ = buf.str();

    // Convert relative mesh URIs to absolute paths so the SDF string
    // works when sent via SpawnFromDescription (no file context)
    if (!modelDir.empty())
    {
      sdfString_ = std::regex_replace(sdfString_,
          std::regex("<uri>meshes/"),
          "<uri>" + modelDir + "meshes/");
    }

    std::smatch match;
    if (std::regex_search(sdfString_, match, std::regex("<model\\s+name=\"([^\"]+)\"")))
      modelName_ = match[1];

    gzmsg << "SpawnG1: Loaded SDF from " << sdfPath << std::endl;
  }
  else
  {
    gzwarn << "SpawnG1: Could not find g1/model.sdf" << std::endl;
  }
}

bool SpawnG1::G1AlreadyInWorld()
{
  const auto worlds = gz::gui::worldNames();
  if (worlds.isEmpty())
    return false;
  const std::string service = "/world/" + worlds.front().toStdString() + "/scene/info";
  gz::msgs::Scene scene;
  bool result = false;
  if (!node_.Request(service, gz::msgs::Empty(), kSceneQueryTimeoutMs, scene, result) || !result)
  {
    // Fail open: the duplicate check must not take away manual placement.
    gzwarn << "SpawnG1: " << service << " unavailable; skipping duplicate check." << std::endl;
    return false;
  }
  // The GUI spawner keeps the SDF name (renaming a second copy), and scripts
  // use prefixed names such as g1_stationary_eval; both count as present.
  for (const auto &model : scene.model())
  {
    const auto &name = model.name();
    if (name == modelName_ || name.rfind(modelName_ + "_", 0) == 0)
    {
      gzmsg << "SpawnG1: [" << name << "] already exists; remove it before placing "
            << "another G1." << std::endl;
      return true;
    }
  }
  return false;
}

void SpawnG1::OnSpawn()
{
  if (sdfString_.empty())
  {
    gzerr << "SpawnG1: No SDF loaded, cannot spawn." << std::endl;
    return;
  }

  if (this->G1AlreadyInWorld())
  {
    emit this->spawnRefused();
    return;
  }

  gz::gui::events::SpawnFromDescription event(sdfString_);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(), &event);

  gzmsg << "SpawnG1: Entering placement mode." << std::endl;
}

}

GZ_ADD_PLUGIN(spawn_g1::SpawnG1, gz::gui::Plugin)
