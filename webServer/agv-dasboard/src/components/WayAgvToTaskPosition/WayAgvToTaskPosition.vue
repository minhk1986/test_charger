<template>
  <g>
    <line
      :x1="positionX1"
      :y1="positionY1"
      :x2="positionX2"
      :y2="positionY2"
      stroke-dasharray="10,5"
      style="stroke: red; stroke-width: 5"
    />
  </g>
</template>
<script>
import { createNamespacedHelpers } from "vuex";
const { mapState: agvsMapState } = createNamespacedHelpers("agvs");
const { mapState: agvJobsMapState } = createNamespacedHelpers("agvJobs");

export default {
  name: "WayAgvToTaskPosition",
  data() {
    return {
      positionX1: 0,
      positionY1: 0,
      positionX2: 0,
      positionY2: 0,
    };
  },
  props: {
    agvId: {
      type: String,
      default: "",
    },
  },
  computed: {
    ...agvsMapState({
      jobId(state) {
        let agv = state.find((a) => a.id === this.agvId);
        return agv != null ? agv.jobId : "";
      },
      taskId(state) {
        let agv = state.find((a) => a.id === this.agvId);
        return agv != null ? agv.taskId : "";
      },
      startWayPoint(state) {
        let agv = state.find((a) => a.id === this.agvId);
        return agv != null ? agv.webPose.position : { x: 0, y: 0, z: 0 };
      },
    }),
    ...agvJobsMapState({
      endWayPoint(state) {
        let job = state.find((j) => j.id === this.jobId);
        if (job == null || job.tasks.length == 0) return null;
        let task = job.tasks.find((t) => t.id === this.taskId);
        return task != null ? task.position : job.tasks[0].position;
      },
    }),
  },
  watch: {
    startWayPoint(val) {
      let position = {
        x: Math.floor(val.x),
        y: Math.floor(val.y),
      };
      this.positionX1 = position.x;
      this.positionY1 = position.y;
      if (this.endWayPoint == null) {
        this.positionX2 = position.x;
        this.positionY2 = position.y;
      }
    },
    endWayPoint(val) {
      if (val == null) {
        this.positionX2 = Math.floor(this.startWayPoint.x);
        this.positionY2 = Math.floor(this.startWayPoint.y);
      } else {
        this.positionX2 = Math.floor(val.x);
        this.positionY2 = Math.floor(val.y);
      }
    },
  },
  mounted() {
    this.positionX1 = Math.floor(this.startWayPoint.x);
    this.positionY1 = Math.floor(this.startWayPoint.y);
    if (this.endWayPoint == null) {
      this.positionX2 = Math.floor(this.startWayPoint.x);
      this.positionY2 = Math.floor(this.startWayPoint.y);
    } else {
      this.positionX2 = Math.floor(this.endWayPoint.x);
      this.positionY2 = Math.floor(this.endWayPoint.y);
    }
  },
};
</script>