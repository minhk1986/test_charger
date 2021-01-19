<template>
  <div class="agv-movement" :style="movementStyle" @contextmenu="onContextMenu">
    <v-row class="agv-movement-row" :style="rowStyle">
      <v-col />
      <v-col class="agv-movement-forward" align="center" justify="space-around">
        <v-btn
          class="agv-movement-button"
          icon
          @mousedown="move({ x: 0, y: 1 })"
          @mouseup="move({ x: 0, y: 0 })"
          @mouseout="move({ x: 0, y: 0 })"
        >
          <v-icon>mdi-chevron-up</v-icon>
        </v-btn>
      </v-col>
      <v-col />
    </v-row>
    <v-row class="agv-movement-row" :style="rowStyle">
      <v-col class="agv-movement-left" align="center" justify="space-around">
        <v-btn
          class="agv-movement-button"
          icon
          @mousedown="move({ x: -1, y: 0 })"
          @mouseup="move({ x: 0, y: 0 })"
          @mouseout="move({ x: 0, y: 0 })"
        >
          <v-icon>mdi-chevron-left</v-icon>
        </v-btn>
      </v-col>
      <v-col />
      <v-col class="agv-movement-right" align="center" justify="center">
        <v-btn
          class="agv-movement-button"
          icon
          @mousedown="move({ x: 1, y: 0 })"
          @mouseup="move({ x: 0, y: 0 })"
          @mouseout="move({ x: 0, y: 0 })"
        >
          <v-icon>mdi-chevron-right</v-icon>
        </v-btn>
      </v-col>
    </v-row>
    <v-row class="agv-movement-row" :style="rowStyle">
      <v-col />
      <v-col
        class="agv-movement-backward"
        align="center"
        justify="space-around"
      >
        <v-btn
          class="agv-movement-button"
          icon
          @mousedown="move({ x: 0, y: -1 })"
          @mouseup="move({ x: 0, y: 0 })"
          @mouseout="move({ x: 0, y: 0 })"
        >
          <v-icon>mdi-chevron-down</v-icon>
        </v-btn>
      </v-col>
      <v-col />
    </v-row>
  </div>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
import { CmdVel } from "@/utils/ros";

const { mapState: navigationMapState } = createNamespacedHelpers("navigation");

export default {
  name: "AgvrMovementButton",
  props: {
    width: {
      type: Number,
      default: null,
    },
    height: {
      type: Number,
      default: null,
    },
  },
  computed: {
    ...navigationMapState({
      navigationAgvId: (state) => state.agvId,
    }),
    movementStyle() {
      return {
        height: this.height ? `${this.height}px` : null,
        width: this.width ? `${this.width}px` : null,
      };
    },
    rowStyle() {
      return {
        height: this.height ? `${this.height / 3}px` : null,
        width: this.width ? `${this.width}px` : null,
      };
    },
  },
  methods: {
    move(twist) {
      this.$agvr.robots[this.navigationAgvId].pushCmdvel(CmdVel.controlMetaToTwistMessage(twist));
    },
    onContextMenu(e) {
      e.preventDefault();
      e.stopPropagation();
    },
  },
};
</script>

<style scoped>
.agv-movement {
  width: 200px;
  height: 200px;
  background-color: rgba(1, 1, 1, 0.3);
  border-radius: 50%;
}

.agv-movement-row {
  width: 200px;
  height: 66.66px;
  margin-left: 0px;
}

.agv-movement-forward {
  margin-top: 10px;
  padding-bottom: 0px;
  border-top: black 1px solid;
  border-left: black 1px solid;
  border-right: black 1px solid;
  border-radius: 15px 15px 0px 0px;
}

.agv-movement-left {
  margin-right: 5px;
  margin-left: 10px;
  padding-right: 8px;
  border-top: black 1px solid;
  border-left: black 1px solid;
  border-bottom: black 1px solid;
  border-radius: 15px 0px 0px 15px;
}

.agv-movement-right {
  margin-right: 10px;
  margin-left: 5px;
  padding-left: 8px;
  border-top: black 1px solid;
  border-right: black 1px solid;
  border-bottom: black 1px solid;
  border-radius: 0px 15px 15px 0px;
}

.agv-movement-backward {
  margin-bottom: 10px;
  border-left: black 1px solid;
  border-right: black 1px solid;
  border-bottom: black 1px solid;
  border-radius: 0px 0px 15px 15px;
}

.agv-movement-button {
  height: 100%;
  width: 100%;
}
</style>
