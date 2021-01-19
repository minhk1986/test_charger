export default {
  setSelectedTaskId(context, id) {
    context.commit('setSelectedTaskId', id);
  },
  showTaskMenuContext(context, position) {
    context.commit("showTaskMenuContext", position);
    context.commit("setMapPanZoomEnable", false);
  },
  hideTaskMenuContext(context) {
    context.commit("hideTaskMenuContext");
    context.commit("setMapPanZoomEnable", true);
  },
  showAgvMenuContext(context, position) {
    context.commit("showAgvMenuContext", position);
    context.commit("setMapPanZoomEnable", false);
  },
  hideAgvMenuContext(context) {
    context.commit("hideAgvMenuContext");
    context.commit("setMapPanZoomEnable", true);
  },
  showAgvInitPoseMenuContext(context, position) {
    context.commit("showAgvInitPoseMenuContext", position);
    context.commit("setMapPanZoomEnable", false);
  },
  hideAgvInitPoseMenuContext(context) {
    context.commit("hideAgvInitPoseMenuContext");
    context.commit("setMapPanZoomEnable", true);
  },
  selectedNone(context) {
    context.commit("setSelectionType", -1);
  },
  selectedTaskPosition(context) {
    context.commit("setSelectionType", 1);
  },
  selectedTaskThetaEnd(context) {
    context.commit("setSelectionType", 2);
  },
  selectedRobot(context) {
    context.commit("setSelectionType", 3);
  },
  selectedRobotDirection(context) {
    context.commit("setSelectionType", 4);
  },
  enableMapPanZoom(context) {
    context.commit("setMapPanZoomEnable", true);
  },
  disableMapPanZoom(context) {
    context.commit("setMapPanZoomEnable", false);
  },
  showActionManagement(context) {
    context.commit("showActionManagement");
  },
  hideActionManagement(context) {
    context.commit("hideActionManagement");
  },
  showJoystickButton(context) {
    context.commit("setJoystickButtonShowSate", true);
  },
  hideJoystickButton(context) {
    context.commit("setJoystickButtonShowSate", false);
  },
  showMovementButton(context) {
    context.commit("setMovementButtonShowSate", true);
  },
  hideMovementButton(context) {
    context.commit("setMovementButtonShowSate", false);
  },
  changeWindowSize(context, payload) {
    context.commit("changeWindowSize", payload);
  },
  setNavigationState(context, payload) {
    context.commit("setNavigationState", payload);
  },
  subscribePose: (context) => context.commit("subscribePose"),
  unsubscribePose: (context) => context.commit("unsubscribePose"),
}