<template>
  <g
    @contextmenu="onContextMenu"
    @mousedown="selectedRobot"
    @touchstart="selectedRobot"
    :transform="transformData"
  >
    <!-- <defs>
      <filter id="filter-boxshadow" :x="0" :y="0" width="200%" height="200%">
        <feOffset in="SourceGraphic" dx="0" dy="0" />
        <feGaussianBlur result="blurOut" in="offOut" stdDeviation="5" />
        <feBlend in="SourceGraphic" in2="blurOut" mode="normal" />
      </filter>
    </defs> -->
    <!-- <rect :x="-agvWidth/2" :y="-agvHeight/2" :width="agvWidth*2" :height="agvHeight" /> --> -->
    <rect
      :visibility="agvRunningState"
      style="fill: #ff9800"
      :x="-3"
      :y="-3"
      rx="5"
      ry="5"
      :width="agvWidth + 6"
      :height="agvHeight + 6"
    >
      <animate
        id="animate1"
        begin="0;animate2.end"
        attributeName="opacity"
        :from="0"
        :to="1"
        dur="0.4s"
      />
      <animate
        id="animate2"
        begin="animate1.end"
        attributeName="opacity"
        :from="1"
        :to="0"
        dur="0.6s"
      />
    </rect>
    <image
      :height="agvHeight"
      :width="agvWidth"
      href="@/assets/images/agv.svg"
    />
    <image
      :x="agvWidth / 4"
      :y="agvHeight / 4"
      :height="agvHeight / 2"
      :width="agvWidth / 2"
      href="@/assets/images/logo-white.png"
    />
  </g>
</template>

<script>
import { createNamespacedHelpers } from "vuex";

const {
  mapState: navigationMapState,
  mapActions: navigationMapAction,
} = createNamespacedHelpers("navigation");

const { mapState: agvsMapState } = createNamespacedHelpers("agvs");

export default {
  name: "AgvrRobot",
  data() {
    return {
      pathData: "",
      timeoutContextMenu: null,
      agvWidth: 0,
      agvHeight: 0,
    };
  },
  props: {
    selectable: {
      type: Boolean,
      default: false,
    },
    agvId: {
      type: String,
      default: "",
    },
    agvMapResolution: {
      type: Number,
      default: 1,
    },
    agvMapScale: {
      type: Number,
      default: 1,
    },
  },
  computed: {
    ...agvsMapState({
      agvWebSize(state) {
        return state.find((a) => a.id === this.agvId).size;
      },
      agvWebPose(state) {
        let agv = state.find((a) => a.id === this.agvId);
        return agv != null
          ? agv.webPose
          : {
              position: { x: 0, y: 0, z: 0 },
              orientation: { x: 0, y: 0, z: 0 },
            };
      },
      agvRunningState(state) {
        let agv = state.find((a) => a.id === this.agvId);
        return agv?.isRunnging ?? false ? "visible" : "hidden";
      },
    }),
    ...navigationMapState({
      isContextMenuShow: (state) =>
        state.taskMenuContext.isShow ||
        state.agvMenuContext.isShow ||
        state.agvInitPoseMenuContext.isShow,
    }),
    transformData() {
      return `rotate(${Math.floor(
        90 - this.agvWebPose.orientation.z
      )},${Math.floor(this.agvWebPose.position.x)},${Math.floor(
        this.agvWebPose.position.y
      )}) translate(${this.agvWebPose.position.x - this.agvWidth / 2}, ${
        this.agvWebPose.position.y - this.agvHeight / 2
      })`;
    },
  },
  methods: {
    ...navigationMapAction({
      navigationShowAgvMenuContext: "showAgvMenuContext",
      navigationShowAgvInitPoseMenuContext: "showAgvInitPoseMenuContext",
      navigationDisableMapPanZoom: "disableMapPanZoom",
      navigationSelectedRobot: "selectedRobot",
    }),
    onContextMenu(e) {
      e.preventDefault();
      e.stopPropagation();
      this.$nextTick(() => {
        if (this.selectable) {
          this.navigationShowAgvInitPoseMenuContext({
            x: e.clientX,
            y: e.clientY,
          });
        } else {
          this.navigationShowAgvMenuContext({
            x: e.clientX,
            y: e.clientY,
          });
        }
      });
    },
    selectedRobot() {
      if (this.isContextMenuShow) return;
      this.navigationDisableMapPanZoom();
      if (this.selectable) {
        this.navigationSelectedRobot();
      }
    },
  },
  mounted() {
    this.agvWidth =
      (this.agvWebSize.width * this.agvMapScale) / this.agvMapResolution;
    this.agvHeight =
      (this.agvWebSize.height * this.agvMapScale) / this.agvMapResolution;
  },
  watch: {
    agvWebSize(val) {
      this.agvWidth = (val.width * this.agvMapScale) / this.agvMapResolution;
      this.agvHeight = (val.height * this.agvMapScale) / this.agvMapResolution;
    },
    agvMapResolution(val) {
      this.agvWidth = (this.agvWebSize.width * this.agvMapScale) / val;
      this.agvHeight = (this.agvWebSize.height * this.agvMapScale) / val;
    },
  },
};
</script>
