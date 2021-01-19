export default {
  setToken(state, token) {
    state.token = token;
    localStorage.setItem("agvr-account-token", token);
  }
}
