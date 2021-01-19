<template>
  <v-app>
    <AppBar />
    <NavigationMenu />
    <v-main>
      <router-view />
    </v-main>
    <AppFooter />
    <notifications class="notifications" position="top right" group="ros" />
  </v-app>
</template>

<script>
import AppBar from "./dashboard/AppBar";
import NavigationMenu from "./dashboard/NavigationMenu";
import AppFooter from "./dashboard/AppFooter";

export default {
  name: "DASHBOARD-LAYOUT",
  components: {
    AppBar,
    NavigationMenu,
    AppFooter,
  },
};
</script>

<style scoped>
.notifications {
  padding-top: 50px;
}
</style>