import QtQuick 2.9
import QtQuick.Controls 2.2

ToolBar {
  id: spawnG1Toolbar
  width: 120
  height: 40

  Button {
    id: spawnBtn
    text: "Spawn G1"
    anchors.fill: parent
    font.pixelSize: 13
    font.bold: true

    background: Rectangle {
      color: spawnBtn.pressed ? "#1565C0" : spawnBtn.hovered ? "#1E88E5" : "#2196F3"
      radius: 4
    }

    contentItem: Text {
      text: spawnBtn.text
      color: "white"
      horizontalAlignment: Text.AlignHCenter
      verticalAlignment: Text.AlignVCenter
      font: spawnBtn.font
    }

    onClicked: {
      SpawnG1.OnSpawn();
    }
  }

  // A G1 is already in the world: say so briefly instead of placing another.
  Connections {
    target: SpawnG1
    onSpawnRefused: {
      spawnBtn.text = "G1 present";
      resetLabel.restart();
    }
  }

  Timer {
    id: resetLabel
    interval: 2500
    onTriggered: spawnBtn.text = "Spawn G1"
  }
}
