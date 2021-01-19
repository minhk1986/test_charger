export default {
  setRosMap(state, payload) {
    let map = state.find((m) => m.id === payload.id);
    if (map == null) return;

    map.rosMap = payload.map;
    let mapScale = map.webMap.scale;
    let rosMapInfo = map.rosMap.info;
    let rosMapData = map.rosMap.data;

    let image = new ImageData(rosMapInfo.width * mapScale, rosMapInfo.height * mapScale);

    let mapI = (rosMapInfo.height - 1) * rosMapInfo.width;
    for (var row = 0; row < rosMapInfo.height; row++, mapI -= rosMapInfo.width * 2) {
      for (var col = 0; col < rosMapInfo.width; col++, mapI++) {
        var val = rosMapData[mapI];
        if (rosMapData[mapI] === 100) {
          val = 0;
        } else if (rosMapData[mapI] === 0) {
          val = 255;
        } else {
          val = 180;
        }
        let i = (col * mapScale + (row * mapScale) * rosMapInfo.width * mapScale) * 4;
        for (var scaleRow = 0; scaleRow < mapScale; scaleRow++, i += ((rosMapInfo.width - 1) * mapScale) * 4) {
          for (var scaleCol = 0; scaleCol < mapScale; scaleCol++) {
            image.data[i++] = val;
            image.data[i++] = val;
            image.data[i++] = val;
            image.data[i++] = 255;
          }
        }
      }
    }
    map.webMap.Image = image;
    let smartPoints = [];
    for(let i = 0; i < map.smartPoints.length; i++) {
      smartPoints.push({
        id: map.smartPoints[i].id,
        x: (- rosMapInfo.origin.position.x + map.smartPoints[i].x) / rosMapInfo.resolution * mapScale,
        y: (rosMapInfo.height + (rosMapInfo.origin.position.y - map.smartPoints[i].y) / rosMapInfo.resolution) * mapScale,
      })
    }
    map.smartWebPoints = smartPoints;
  },
  setConnectionState(state, payload) {
    let map = state.find((m) => m.id === payload.id);
    if (map != null) map.rosConntectionState = payload.isConnected;
  },
}