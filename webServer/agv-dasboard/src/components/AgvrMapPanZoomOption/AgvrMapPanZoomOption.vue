<template>
  <v-menu rounded="0" offset-y>
    <template v-slot:activator="{ attrs, on }">
      <v-btn icon x-large v-on="on" v-bind="attrs" small>
        <v-icon large dark> mdi-dots-vertical </v-icon>
      </v-btn>
    </template>
    <v-list dense>
      <v-list-item @click="() => pushRouterTo('/navigation/init')">
        <v-list-item-icon>
          <v-icon v-text="'mdi-crosshairs-gps'"></v-icon>
        </v-list-item-icon>
        <v-list-item-content>
          <v-list-item-title>Initialize pose</v-list-item-title>
        </v-list-item-content>
      </v-list-item>
      <v-list-item @click="() => pushRouterTo('/navigation')">
        <v-list-item-icon>
          <v-icon v-text="'mdi-near-me'"></v-icon>
        </v-list-item-icon>
        <v-list-item-content>
          <v-list-item-title>Navigation</v-list-item-title>
        </v-list-item-content>
      </v-list-item>
      <v-divider />
      <v-list-item>
        <v-list-item-action>
          <v-checkbox v-model="isShowJoystickButton"></v-checkbox>
        </v-list-item-action>

        <v-list-item-content>
          <v-list-item-title>Show joystick button</v-list-item-title>
        </v-list-item-content>
      </v-list-item>
      <v-list-item>
        <v-list-item-action>
          <v-checkbox v-model="isShowMovementButton"></v-checkbox>
        </v-list-item-action>

        <v-list-item-content>
          <v-list-item-title>Show movement button</v-list-item-title>
        </v-list-item-content>
      </v-list-item>
    </v-list>
  </v-menu>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
const { mapState, mapActions } = createNamespacedHelpers("navigation");

export default {
  name: "AgvrMapPanZoomOption",
  methods: {
    ...mapActions([
      "showJoystickButton",
      "hideJoystickButton",
      "showMovementButton",
      "hideMovementButton",
      "setNavigationState",
    ]),
    pushRouterTo(path) {
      if(this.$router.currentRoute.path === path) return;
      this.$router.push(path);
    },
  },
  computed: {
    ...mapState({
      joystickButtonState: (state) => state.joystickButton.isShow,
      movementButtonState: (state) => state.movementButton.isShow,
    }),
    isShowJoystickButton: {
      get() {
        return this.joystickButtonState;
      },
      set(val) {
        if (val) {
          this.showJoystickButton();
        } else {
          this.hideJoystickButton();
        }
      },
    },
    isShowMovementButton: {
      get() {
        return this.movementButtonState;
      },
      set(val) {
        if (val) {
          this.showMovementButton();
        } else {
          this.hideMovementButton();
        }
      },
    },
  },
};
</script>