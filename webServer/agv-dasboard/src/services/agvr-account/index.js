export default class AgvrAccount {
  #store = null;
  #token = null;
  constructor(options) {
    this.#store = options.store;
    this.#store.dispatch("account/setToken", localStorage.getItem("agvr-account-token"));
    this.#token = this.#store.state.account.token;
  }
}