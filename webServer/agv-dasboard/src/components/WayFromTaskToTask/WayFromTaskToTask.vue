<template>
  <g>
    <template v-if="agvTasks.length > 0">
      <line
        v-for="index in agvTasks.length - 1"
        :key="`way-${jobId}-${agvTasks[index - 1].id}-${agvTasks[index].id}`"
        :x1="agvTasks[index - 1].position.x"
        :y1="agvTasks[index - 1].position.y"
        :x2="agvTasks[index].position.x"
        :y2="agvTasks[index].position.y"
        stroke-dasharray="10,8"
        style="stroke: rgb(0, 0, 255); stroke-width: 2"
      />
    </template>
  </g>
</template>
<script>
import { createNamespacedHelpers } from "vuex";
const { mapState: agvJobsMapState } = createNamespacedHelpers("agvJobs");
export default {
  name: "WayFromTaskToTask",
  props: {
    jobId: {
      type: String,
      default: "",
    },
  },
  computed: {
    ...agvJobsMapState({
      agvTasks(state) {
        let job = state.find((j) => j.id === this.jobId);
        return job != null ? job.tasks : [];
      },
    }),
  },
};
</script>