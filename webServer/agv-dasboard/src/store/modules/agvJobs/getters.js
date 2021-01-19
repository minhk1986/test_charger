import { ROS2D } from '@/utils/ros';
export default {
  getPoseStampedTask: (state) => (payload) => {
    let job = state.find((j) => j.id === payload.jobId);
    if (job == null || job.tasks.length == 0) return null;

    let task = null;
    if (payload.taskId == null) {
      task = job.tasks[0];
    }
    else {
      task = job.tasks.find((t) => t.id === payload.taskId);
    }
    if (task == null) return null;
    console.log(payload);
    return {
      orientation: ROS2D.globalThetaToQuaternion(task.thetaEnd * Math.PI / 180),
      position: {
        x: payload.mapRosInfo.origin.position.x + task.position.x * payload.mapRosInfo.resolution / payload.mapScale,
        y: payload.mapRosInfo.origin.position.y - ((task.position.y / payload.mapScale) - payload.mapRosInfo.height) * payload.mapRosInfo.resolution,
        z: 0,
      },
    };
  },
  getLastTask: (state) => (jobId) => {
    let job = state.find((j) => j.id === jobId);
    if (job == null || job.tasks.length == 0) return null;
    return job.tasks[job.tasks.length - 1];
  }
}