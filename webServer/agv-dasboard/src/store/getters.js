import { ROS2D } from '@/utils/ros';
export default {
  getPoseStampedTaskOfSelectedJob: (state) => (index) => {
    let job = state.agvJobs.find(job => job.id === state.navigation.selectedJobId);
    if (index < 0 || index >= job.tasks.length) return null;

    let map = state.agvMaps.find(map => map.id === state.navigation.selectedMapId);
    let todoTask = job.tasks[index];
    return {
      orientation: ROS2D.globalThetaToQuaternion(todoTask.thetaEnd * Math.PI / 180),
      position: {
        x: map.rosMap.info.origin.position.x + todoTask.position.x * map.rosMap.info.resolution / map.webMap.scale,
        y: map.rosMap.info.origin.position.y - ((todoTask.position.y / map.webMap.scale) - map.rosMap.info.height) * map.rosMap.info.resolution,
        z: 0,
      },
    };
  },
  getListActionOfSelectedTask: (state) => () => {
    if(state.navigation.selectedIndexTask < 0) return [];
    let tasks = state.agvJobs.find(job => job.id === state.navigation.selectedJobId).tasks;
    if(state.navigation.selectedIndexTask >= tasks.length) return [];
    return tasks[state.navigation.selectedIndexTask].actions;
  }
}