import Vue from 'vue'
import Router from 'vue-router'

import MainLayout from '@/layouts/MainLayout'
import LoginLayout from '@/layouts/LoginLayout'
import DashboardLayout from '@/layouts/DashboardLayout'
import LogoutLayout from '@/layouts/LogoutLayout'

import IntroduceRoutes from './introduce'
import LoginRoutes from './login'
import LogoutRoutes from './logout'
import DashboardRoutes from './dashboard'

Vue.use(Router)

const router = new Router({
  mode: 'hash',
  base: process.env.BASE_URL,
  routes: [
    {
      path: '/',
      component: MainLayout,
      children: [
        {
          path: 'introduce',
          component: DashboardLayout,
          children: IntroduceRoutes,
          meta: {
            title: 'Introduce'
          }
        },
        {
          path: 'login',
          component: LoginLayout,
          children: LoginRoutes,
          meta: {
            title: 'Login'
          }
        },
        {
          path: 'logout',
          component: LogoutLayout,
          children: LogoutRoutes,
          meta: {
            title: 'Logout'
          }
        },
        {
          path: '',
          component: DashboardLayout,
          children: DashboardRoutes,
          meta: {
            title: 'Dashboard'
          }
        }
      ]
    }
  ]
})


router.beforeEach((to, from, next) => {
  let token = Vue.prototype.$store.state.account.token;
  if (token != null && token.length > 0) {
    if (to.path === "/login") {
      next("/");
    }
    else {
      document.title = to.meta.title ? to.meta.title : 'AGV PRATI';
      next();
    }
  }
  else if (to.path === "/login") {
    next();
  }
  else {
    next("/login");
  }
})

export default router