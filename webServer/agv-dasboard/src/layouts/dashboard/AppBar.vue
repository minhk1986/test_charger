<template>
  <v-app-bar color="primary" dark app flat dense>
    <v-app-bar-nav-icon @click="() => toggleNavigationMenu(true)" />
    <v-toolbar-title class="noselect">AGV ROBOTICS</v-toolbar-title>
    <v-spacer />
    <v-badge
      class="mt-2"
      :color="rosConnectColor"
      :icon="rosConnectIcon"
      overlap
    >
      <v-icon large v-text="'mdi-robot-mower'" />
    </v-badge>
  </v-app-bar>
</template>

<script>
import { createNamespacedHelpers } from "vuex";

const {mapActions: dashboardMapActions, } = createNamespacedHelpers("dashboard");
const {mapState: navigationMapState, } = createNamespacedHelpers("navigation");
const { mapState: agvsMapState } = createNamespacedHelpers("agvs");

export default {
  name: "AppBar",
  data() {
    return {
      rosConnectIcon: "mdi-close",
      rosConnectColor: "red darken-1",
    };
  },
  methods: {
    ...dashboardMapActions({
      toggleNavigationMenu: "toggleNavigationMenu",
    }),
  },
  computed: {
    ...navigationMapState({
      navigationAgvId: (state) => state.agvId,
    }),
    ...agvsMapState({
      isRosConnected(state) {
        let agv = state.find(a => a.id === this.navigationAgvId);
        return agv != null ? agv.rosConntectionState :  false;
      },
    }),
  },
  mounted() {
    if (this.isRosConnected) {
      this.rosConnectIcon = "mdi-check";
      this.rosConnectColor = "green darken-1";
    } else {
      this.rosConnectIcon = "mdi-close";
      this.rosConnectColor = "red darken-1";
    }
  },
  watch: {
    isRosConnected(val) {
      if (val) {
        this.rosConnectIcon = "mdi-check";
        this.rosConnectColor = "green darken-1";
      } else {
        this.rosConnectIcon = "mdi-close";
        this.rosConnectColor = "red darken-1";
      }
    },
  },
};
</script>
