<template>
  <v-container class="fill-height pa-0 agv-navigation-controller" fluid>
    <router-view />
    <div class="topbar d-flex">
      <v-icon class="blue--text text--darken-2 pa-1" large v-text="navigationSubscribePose ?'mdi-near-me' : 'mdi-crosshairs-gps' "/>
      <AgvrPoseEditor />
      <AgvrMapPanZoomOption />
    </div>
  </v-container>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
import AgvrPoseEditor from "@/components/AgvrPoseEditor";
import AgvrMapPanZoomOption from "@/components/AgvrMapPanZoomOption";

const { mapState : navigationMapState } = createNamespacedHelpers("navigation");

export default {
  name: "Navigation",
  components: {
    AgvrMapPanZoomOption,
    AgvrPoseEditor,
  },
  computed: {
    ...navigationMapState({
      joystickButtonState: (state) => state.joystickButton.isShow,
      movementButtonState: (state) => state.movementButton.isShow,
      navigationState: (state) => state.navigationState,
      navigationSubscribePose: (state) => state.subscribePose,
    }),
  },
};
</script>

<style scoped>
.agv-navigation-controller {
  position: absolute;
  background-color: rgba(180, 180, 180, 255);
}
.agv-navigation-controller .joystick {
  position: absolute;
  bottom: 4px;
  left: 4px;
}
.agv-navigation-controller .movement {
  position: absolute;
  bottom: 4px;
  right: 4px;
}
.agv-navigation-controller .options {
  position: absolute;
  top: 4px;
  right: 4px;
}
.agv-navigation-controller .topbar {
  position: absolute;
  top: 0px;
  width: 100%;
}
</style>
