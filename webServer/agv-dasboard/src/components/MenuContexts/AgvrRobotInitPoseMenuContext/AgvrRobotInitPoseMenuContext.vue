<template>
  <v-menu
    v-model="isShow"
    :position-x="navigationAgvInitPoseMenuContext.position.x"
    :position-y="navigationAgvInitPoseMenuContext.position.y"
    absolute
    offset-y
    rounded="0"
  >
    <v-list dense>
      <v-list-item-group class="pa-0" color="primary">
        <v-list-item @click="onInitializePose">
          <v-list-item-icon>
            <v-icon v-text="'mdi-play'"></v-icon>
          </v-list-item-icon>
          <v-list-item-content>
            <v-list-item-title v-text="'Initialize pose'"></v-list-item-title>
          </v-list-item-content>
        </v-list-item>
      </v-list-item-group>
    </v-list>
  </v-menu>
</template>

<script>
import { createNamespacedHelpers } from "vuex";

const { mapState: navigationMapState, mapActions: navigationMapActions } = createNamespacedHelpers("navigation");

export default {
  name: "AgvrRobotInitPoseMenuContext",
  computed: {
    ...navigationMapState({
      navigationAgvInitPoseMenuContext: (state) => state.agvInitPoseMenuContext,
      navigationAgvId: (state) => state.agvId,
    }),
    isShow: {
      get() {
        return this.navigationAgvInitPoseMenuContext.isShow;
      },
      set(val) {
        if (val) {
          this.navigationShowAgvInitPoseMenuContext();
        } else {
          this.navigationHideAgvInitPoseMenuContext();
        }
      },
    },
  },
  methods: {
    ...navigationMapActions({
      navigationShowAgvInitPoseMenuContext: "showAgvInitPoseMenuContext",
      navigationHideAgvInitPoseMenuContext: "hideAgvInitPoseMenuContext",
      setNavigationState: "setNavigationState",
    }),
    onInitializePose() {
      this.$agvr.robots[this.navigationAgvId].initializePose();
      this.$router.push('/navigation');
    },
  },
};
</script>