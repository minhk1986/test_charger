<template>
  <div
    class="agv-joystick"
    :style="{
      height: `${joystickHeight}px`,
      width: `${joystickWidth}px`,
    }"
  >
    <div
      class="agv-joystick-point"
      :style="{
        top: `${joystickHandleTop}px`,
        left: `${joystickHandleLeft}px`,
        height: `${joystickHandleHeight}px`,
        width: `${joystickHandleWidth}px`,
      }"
      @mousedown="() => (joystickClicked = true)"
      @mouseup="() => (joystickClicked = false)"
      @mouseleave="() => (joystickClicked = false)"
      @mousemove="(e) => joystickMove(e.movementX, e.movementY)"
      @touchstart="joystickTouchStart"
      @touchend="joystickTouchEnd"
      @touchmove="onTouchMove"
    />
  </div>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
import { CmdVel } from "@/utils/ros";
const { mapState: navigationMapState } = createNamespacedHelpers("navigation");

export default {
  name: "AgvrJoyStickButton",
  data: () => ({
    joystickClicked: false,
    joystickHandleTop: 0,
    joystickHandleLeft: 0,
    touchClientX: -1,
    touchClientY: -1,
  }),
  props: {
    width: {
      type: Number,
      default: null,
    },
    height: {
      type: Number,
      default: null,
    },
    handleWidth: {
      type: Number,
      default: null,
    },
    handleHeight: {
      type: Number,
      default: null,
    },
  },
  computed: {
    ...navigationMapState({
      navigationAgvId: (state) => state.agvId,
    }),
    joystickStyle() {
      return {
        height: this.height ? `${this.height}px` : null,
        width: this.width ? `${this.width}px` : null,
      };
    },
    joystickWidth() {
      return this.width ? this.width : 200;
    },
    joystickHeight() {
      return this.height ? this.height : 200;
    },
    joystickHandleWidth() {
      return this.handleWidth ? this.handleWidth : 80;
    },
    joystickHandleHeight() {
      return this.handleHeight ? this.handleHeight : 80;
    },
    joystickHandleX() {
      return (this.joystickWidth - this.joystickHandleWidth) / 2;
    },
    joystickHandleY() {
      return (this.joystickHeight - this.joystickHandleHeight) / 2;
    },
  },
  mounted() {
    this.joystickResetPosition();
  },
  methods: {
    joystickMove(movementX, movementY) {
      if (this.joystickClicked) {
        let centerY =
          this.joystickHandleTop +
          movementY +
          this.joystickHandleHeight / 2 -
          this.joystickHeight / 2;
        let centerX =
          this.joystickHandleLeft +
          movementX +
          this.joystickHandleWidth / 2 -
          this.joystickWidth / 2;
        let d = Math.sqrt(Math.pow(centerX, 2) + Math.pow(centerY, 2));
        if (d <= this.joystickHandleX && d <= this.joystickHandleY) {
          this.joystickHandleTop += movementY;
          this.joystickHandleLeft += movementX;
          this.$agvr.robots[this.navigationAgvId].pushCmdvel(
            CmdVel.controlMetaToTwistMessage({
              x: centerX / this.joystickHandleX,
              y: -centerY / this.joystickHandleY,
            })
          );
        }
      }
    },
    joystickResetPosition() {
      this.joystickHandleTop = this.joystickHandleX;
      this.joystickHandleLeft = this.joystickHandleY;
      this.$agvr.robots[this.navigationAgvId].pushCmdvel(CmdVel.controlMetaToTwistMessage({ x: 0, y: 0 }));
    },
    joystickTouchStart(e) {
      let touch = e.changedTouches[0];
      this.touchClientX = touch.clientX;
      this.touchClientY = touch.clientY;
      this.joystickClicked = true;
    },
    joystickTouchEnd() {
      this.touchClientX = -1;
      this.touchClientY = -1;
      this.joystickClicked = false;
    },
    onTouchMove(e) {
      e.preventDefault();
      if (this.touchClientX > 0 && this.touchClientY > 0) {
        let touch = e.changedTouches[0];
        this.joystickMove(
          touch.clientX - this.touchClientX,
          touch.clientY - this.touchClientY
        );
        this.touchClientX = touch.clientX;
        this.touchClientY = touch.clientY;
      }
    },
  },
  watch: {
    joystickClicked(newVal, oldVal) {
      if (newVal != oldVal && !newVal) {
        this.joystickResetPosition();
      }
    },
  },
};
</script>

<style scoped>
.agv-joystick {
  background-color: rgba(1, 1, 1, 0.3);
  border-radius: 50%;
}
.agv-joystick-point {
  background-color: red;
  border-radius: 50%;
  position: relative;
}
</style>
