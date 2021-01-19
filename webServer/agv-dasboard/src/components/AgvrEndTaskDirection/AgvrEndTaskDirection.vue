<template>
  <g :visibility="agvTask.hidden ? 'hidden' : 'visible'">
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
      :x1="agvTask.position.x"
      :y1="agvTask.position.y"
      :x2="
        agvTask.position.x + 50 * Math.cos((agvTask.thetaEnd * Math.PI) / 180)
      "
      :y2="
        agvTask.position.y - 50 * Math.sin((agvTask.thetaEnd * Math.PI) / 180)
      "
      stroke="limegreen"
      stroke-width="10"
      marker-end="url(#arrowhead)"
      @mousedown="onMouseDown"
      @touchstart="onMouseDown"
    />
  </g>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
const { mapState: agvJobsMapState } = createNamespacedHelpers("agvJobs");
const {
  mapState: navigationMapState,
  mapActions: navigationMapActions,
} = createNamespacedHelpers("navigation");

export default {
  name: "AgvrEndTaskDirection",
  computed: {
    ...navigationMapState({
      navigationJobId: (state) => state.jobId,
      navigationTaskId: (state) => state.taskId,
      isContextMenuShow: (state) =>
        state.taskMenuContext.isShow ||
        state.agvMenuContext.isShow ||
        state.agvInitPoseMenuContext.isShow,
    }),
    ...agvJobsMapState({
      agvTask(state) {
        let job = state.find((j) => j.id === this.navigationJobId);
        if (job == null || job.tasks.length == 0)
          return { position: { x: 0, y: 0, z: 0 }, thetaEnd: 0, hidden: true };

        let task = job.tasks.find((t) => t.id === this.navigationTaskId);
        return task != null ? task : { position: { x: 0, y: 0, z: 0 }, thetaEnd: 0, hidden: true };
      },
    }),
    
  },
  methods: {
    ...navigationMapActions({
      navigationDisableMapPanZoom: "disableMapPanZoom",
      navigationSelectedTaskThetaEnd: "selectedTaskThetaEnd",
    }),
    onMouseDown() {
      if (this.isContextMenuShow) return;
      this.navigationDisableMapPanZoom();
      this.navigationSelectedTaskThetaEnd();
    },
  },
};
</script>