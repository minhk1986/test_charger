import Vue from 'vue';
import App from './App.vue';
import router from './router';
import store from './store';
import vuetify from './plugins/vuetify';
import panZoom from 'vue-panzoom';
import UUID from "vue-uuid";
import AGVR from "./plugins/agvr";
import AgvrAccount from "./plugins/agvr-account";
import Notifications from 'vue-notification';
import "./assets/css/main.css";

Vue.config.productionTip = false;

Vue.prototype.$store = store;

Vue.use(panZoom);
Vue.use(UUID);
Vue.use(Notifications);

Vue.use(AGVR, process.env.VUE_APP_AGV_MAP_ID );
Vue.use(AgvrAccount, { store });

new Vue({
    router,
    store,
    vuetify,
    render: h => h(App)
}).$mount('#app');