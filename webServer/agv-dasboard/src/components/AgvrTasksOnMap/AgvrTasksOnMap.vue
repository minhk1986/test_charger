<template>
  <g>
    <circle
      v-for="task in agvTasks"
      :key="`way-${jobId}-${task.id}`"
      :cx="task.position.x"
      :cy="task.position.y"
      r="12"
      stroke-width="3"
      :fill="task.id === navigationTaskId ? 'teal' : 'indigo'"
      @mousedown="() => onMouseDown(task.id)"
      @touchstart="() => onMouseDown(task.id)"
      @contextmenu="onContextMenu"
    />
    <text
      v-for="(task, index) in agvTasks"
      :key="`way-index-${jobId}-${task.id}`"
      :x="task.position.x" 
      :y="task.position.y" 
      text-anchor="middle" 
      fill="white"
      stroke-width="1px" 
      font-size="0.8em"
      class="agvr-task-index-text"
      dy="0.4em" v-text="index" />
  </g>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
const { mapState: agvJobsMapState } = createNamespacedHelpers("agvJobs");
const { mapState: navigationMapState, mapActions: navigationMapActions } = createNamespacedHelpers("navigation");
export default {
  name: "AgvrTasksOnMap",
  props: {
    jobId: {
      type: String,
      default: "",
    },
  },
  computed: {
    ...navigationMapState({
      navigationTaskId: (state) => state.taskId,
      selectedIndexTask: (state) => state.selectedIndexTask,
      isContextMenuShow: (state) => state.taskMenuContext.isShow || state.agvMenuContext.isShow || state.agvInitPoseMenuContext.isShow,
    }),
    ...agvJobsMapState({
      agvTasks(state) {
        let job = state.find((j) => j.id === this.jobId);
        return job != null ? job.tasks : [];
      },
    }),
  },
  methods: {
    ...navigationMapActions({
      navigationSetSelectedTaskId: "setSelectedTaskId",
      navigationShowTaskMenuContext: "showTaskMenuContext",
      navigationDisableMapPanZoom: "disableMapPanZoom",
      navigationSelectedTaskPosition: "selectedTaskPosition",
    }),
    onMouseDown(id) {
      if (this.isContextMenuShow) return;
      this.navigationDisableMapPanZoom();
      this.navigationSelectedTaskPosition();
      this.navigationSetSelectedTaskId(id);
    },
    onContextMenu(e) {
      e.preventDefault();
      e.stopPropagation();
      this.$nextTick(() => {
        this.navigationShowTaskMenuContext({
          x: e.clientX,
          y: e.clientY,
        });
      });
    },
  },
};
</script>

<style scoped>
.agvr-task-index-text {
  pointer-events: none;
}
</style>