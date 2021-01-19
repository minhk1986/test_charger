import { AgvrAccount } from "@/services";
export default {
  install: (Vue, options) => {
    Vue.prototype.$agvrAccount = new AgvrAccount(options);
  }
}