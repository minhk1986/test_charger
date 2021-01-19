export default {
  rosConnected(context) {
    context.commit('setRosConnectionState', true)
  },
  rosDisconnected(context) {
    context.commit('setRosConnectionState', false)
  },
}