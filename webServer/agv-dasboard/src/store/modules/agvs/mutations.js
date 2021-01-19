function ConvertMapPositionToWebPosition(mapPosition, map, scale) {
  return {
    x: (- map.origin.position.x + mapPosition.x) / map.resolution * scale,
    y: (map.height + (map.origin.position.y - mapPosition.y) / map.resolution) * scale,
    z: 0,
  }
}

export default {
  setConnectionState(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv != null) {
      agv.rosConntectionState = payload.isConnected;
    }
  },
  setSim1000Pose(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv == null) return;

    agv.sim1000Pose = payload.pose;

    // if (payload.rosMap.info.height == 0 || payload.rosMap.info.width == 0) return;

    // if (Math.abs(agv.sim1000Pose.posex - payload.pose.posex) < 10
    //   && Math.abs(agv.sim1000Pose.posey - payload.pose.posey) < 10
    //   && Math.abs(agv.sim1000Pose.poseyaw - payload.pose.poseyaw) < 1000) return;

    
    agv.webPose = {
      position: ConvertMapPositionToWebPosition({
        x: payload.pose.posex / 1000,
        y: payload.pose.posey / 1000,
      }, payload.rosMap.info, payload.mapScale),
      orientation: {
        x: 0,
        y: 0,
        z: payload.pose.poseyaw / 1000,
      },
    }
  },
  moveAgvPosition(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv != null) {
      agv.webPose.position.x += payload.movementX;
      agv.webPose.position.y += payload.movementY;
    }
  },
  changeAgvDirection(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv == null) return;
    let theta = Math.atan2(agv.webPose.position.y - payload.mouse.y, payload.mouse.x - agv.webPose.position.x);
    agv.webPose.orientation.z = theta * 180 / Math.PI;
  },
  setAgvDirection(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv == null) return;

    agv.webPose.orientation = {
      x: payload.x,
      y: payload.y,
      z: payload.z,
    };
  },
  setAgvPosition(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv == null) return;

    agv.webPose.position = {
      x: payload.x,
      y: payload.y,
      z: payload.z,
    };
  },
  setAgvStatus(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv == null) return;

    agv.statusDevices = payload.status;
    // if(agv.statusDeviceSelected.name != null && agv.statusDeviceSelected.name != '') {

    // }
  },
  setAgvTask(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv == null) return;

    agv.taskId = payload.taskId;
  },
  setRunningState(state, payload) {
    let agv = state.find((a) => a.id === payload.agvId);
    if (agv == null) return;

    agv.isRunnging = payload.isRunnging;
  },
}
