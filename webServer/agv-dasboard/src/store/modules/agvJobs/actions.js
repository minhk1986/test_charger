export default {
  pushTaskToJob: (context, payload) => context.commit("pushTaskToJob", payload),
  moveTaskPosition: (context, payload) => context.commit("moveTaskPosition", payload),
  setThetaEndOfTask: (context, payload) => context.commit("setThetaEndOfTask", payload),
  insertTaskAfterToJob: (context, payload) => context.commit("insertTaskAfterToJob", payload),
  insertTaskBeforeToJob: (context, payload) => context.commit("insertTaskBeforeToJob", payload),
  removeTaskFromJob: (context, payload) => context.commit("removeTaskFromJob", payload),
  updateActions: (context, payload) => context.commit("updateActions", payload),
  setTasks: (context, payload) => context.commit("setTasks", payload),
}