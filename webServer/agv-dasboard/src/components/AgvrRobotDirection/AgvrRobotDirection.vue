<template>
  <g>
    <defs>
      <marker
        id="arrowhead"
        markerWidth="3"
        markerHeight="2"
        refX="1"
        refY="1"
        orient="auto"
      >
        <polygon points="0 0, 3 1, 0 2" style="fill: limegreen" />
      </marker>
    </defs>
    <line
      :x1="agvWebPose.position.x"
      :y1="agvWebPose.position.y"
      :x2="
        agvWebPose.position.x +
        100 * Math.cos((agvWebPose.orientation.z * Math.PI) / 180)
      "
      :y2="
        agvWebPose.position.y -
        100 * Math.sin((agvWebPose.orientation.z * Math.PI) / 180)
      "
      stroke="limegreen"
      stroke-width="10"
      marker-end="url(#arrowhead)"
      @mousedown="selectedRobotDirection"
      @touchstart="selectedRobotDirection"
    />
  </g>
</template>
<script>
import { createNamespacedHelpers } from "vuex";
const { mapState: agvsMapState } = createNamespacedHelpers("agvs");
const {
  mapState: navigationMapState,
  mapActions: navigationMapAction,
} = createNamespacedHelpers("navigation");

export default {
  name: "AgvrRobotDirection",
  props: {
    agvId: {
      type: String,
      default: "",
    },
  },
  computed: {
    ...agvsMapState({
      agvWebPose(state) {
        let agv = state.find((a) => a.id === this.agvId);
        return agv != null
          ? agv.webPose
          : {
              position: { x: 0, y: 0, z: 0 },
              orientation: { x: 0, y: 0, z: 0 },
            };
      },
    }),
    ...navigationMapState({
      isContextMenuShow: (state) =>
        state.taskMenuContext.isShow ||
        state.agvMenuContext.isShow ||
        state.agvInitPoseMenuContext.isShow,
    }),
  },
  methods: {
    ...navigationMapAction({
      navigationDisableMapPanZoom: "disableMapPanZoom",
      navigationSelectedRobotDirection: "selectedRobotDirection",
    }),
    selectedRobotDirection() {
      if (this.isContextMenuShow) return;
      this.navigationDisableMapPanZoom();
      this.navigationSelectedRobotDirection();
    },
  },
};
</script>