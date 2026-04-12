#include "SpawnG1.hh"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <regex>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Filesystem.hh>

namespace spawn_g1
{

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
        std::string candidate = path + "/g1/model.sdf";
        if (gz::common::isFile(candidate))
        {
          sdfPath = candidate;
          modelDir = path + "/g1/";
          break;
        }
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

    gzmsg << "SpawnG1: Loaded SDF from " << sdfPath << std::endl;
  }
  else
  {
    gzwarn << "SpawnG1: Could not find g1/model.sdf" << std::endl;
  }
}

void SpawnG1::OnSpawn()
{
  if (sdfString_.empty())
  {
    gzerr << "SpawnG1: No SDF loaded, cannot spawn." << std::endl;
    return;
  }

  gz::gui::events::SpawnFromDescription event(sdfString_);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(), &event);

  gzmsg << "SpawnG1: Entering placement mode." << std::endl;
}

}

GZ_ADD_PLUGIN(spawn_g1::SpawnG1, gz::gui::Plugin)
