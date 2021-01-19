export default {
  moveAgvPosition: (context, payload) => context.commit("moveAgvPosition", payload),
  changeAgvDirection: (context, payload) => context.commit("changeAgvDirection", payload),
  setAgvDirection: (context, payload) => context.commit("setAgvDirection", payload),
  setAgvPosition: (context, payload) => context.commit("setAgvPosition", payload),
  setAgvStatus: (context, payload) => context.commit("setAgvStatus", payload),
  setAgvTask: (context, payload) => context.commit("setAgvTask", payload),
  setRunningState: (context, payload) => context.commit("setRunningState", payload),
}