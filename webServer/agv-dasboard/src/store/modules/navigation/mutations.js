export default {
  setSelectedTaskId(state, id) {
    state.taskId = id;
  },
  showTaskMenuContext(state, position) {
    state.taskMenuContext = {
      position: {
        x: (typeof position.x === "number") ? position.x : 0,
        y: (typeof position.y === "number") ? position.y : 0,
      },
      isShow: true,
    }
  },
  hideTaskMenuContext(state) {
    state.taskMenuContext.isShow = false;
  },
  showAgvMenuContext(state, position) {
    state.agvMenuContext = {
      position: {
        x: (typeof position.x === "number") ? position.x : 0,
        y: (typeof position.y === "number") ? position.y : 0,
      },
      isShow: true,
    }
  },
  hideAgvMenuContext(state) {
    state.agvMenuContext.isShow = false;
  },
  showAgvInitPoseMenuContext(state, position) {
    state.agvInitPoseMenuContext = {
      position: {
        x: (typeof position.x === "number") ? position.x : 0,
        y: (typeof position.y === "number") ? position.y : 0,
      },
      isShow: true,
    }
  },
  hideAgvInitPoseMenuContext(state) {
    state.agvInitPoseMenuContext.isShow = false;
  },
  setSelectionType(state, type) {
    if (Number.isInteger(type)) {
      state.selectionType = type;
    }
  },
  setMapPanZoomEnable(state, enable) {
    state.mapPanZoomEnable = enable;
  },
  showActionManagement(state) {
    state.actionManagement.isShow = true;
  },
  hideActionManagement(state) {
    state.actionManagement.isShow = false;
  },
  setJoystickButtonShowSate(state, isShow) {
    state.joystickButton.isShow = isShow;
  },
  setMovementButtonShowSate(state, isShow) {
    state.movementButton.isShow = isShow;
  },
  changeWindowSize(state, payload) {
    state.viewHeight = payload.height;
    state.viewWidth = payload.width;
  },
  setNavigationState(state, payload) {
    state.navigationState = payload;
  },
  subscribePose(state) {
    state.subscribePose = true;
  },
  unsubscribePose(state) {
    state.subscribePose = false;
  },
}