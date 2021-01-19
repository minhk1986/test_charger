export default {
  setProperty(context, payload) {
    context.commit('setProperty', payload);
  },
  insertTaskAfterToJob(context) {
    context.commit('insertTaskAfterToJob');
  },
  insertTaskBeforeToJob(context) {
    context.commit('insertTaskBeforeToJob');
  },
  removeSelectedTaskFromJob(context) {
    context.commit("removeSelectedTaskFromJob");
  },
  remoceTaskFromSelectedJob(context, payload) {
    context.commit("remoceTaskFromSelectedJob", payload);
  },
  setThetaEndOfSelectedTask(context, payload) {
    context.commit("setThetaEndOfSelectedTask", payload);
  },
  moveSelectedAgv(context, payload) {
    context.commit("moveSelectedAgv", payload);
  },
  changeSelectedAgvDirection(context, payload) {
    context.commit("changeSelectedAgvDirection", payload);
  },
  moveSelectedAgvToCenterMap(context) {
    context.commit("moveSelectedAgvToCenterMap");
  },
  setSelectedAgvPosition(context, payload) {
    context.commit("setSelectedAgvPosition", payload);
  },
  setSelectedAgvDirection(context, payload) {
    context.commit("setSelectedAgvDirection", payload);
  },
  addGoalToCar(context) {
    context.commit("addGoalToCar");
  },
  setSelectedAgvStatus(context, status) {
    context.commit("setSelectedAgvStatus", status);
  },
  updateActionsOfSelectedTask(context, payload) {
    context.commit("updateActionsOfSelectedTask", payload);
  },
}