import StoreProcess from "@/utils/store-process";

export default {
  setProperty(state, payload) {
    StoreProcess.setDeepProperty(state, payload.value, payload.properties);
  },
  insertTaskBeforeToJob(state) {

    this.commit("ros/jobs/insertTaskBeforeToJob", {
      jobId: state.navigation.selectedJobId,
      indexTask: state.navigation.selectedIndexTask,
      agvPosition: state.agvs.find(agv => agv.id === state.navigation.selectedAgvId).webPose.position,
    });
  },
  insertTaskAfterToJob(state) {
    this.commit("ros/jobs/insertTaskAfterToJob", {
      jobId: state.navigation.selectedJobId,
      indexTask: state.navigation.selectedIndexTask,
    });
  },
  removeSelectedTaskFromJob(state) {
    this.commit("ros/jobs/removeTaskFromJob", {
      jobId: state.navigation.selectedJobId,
      index: state.navigation.selectedIndexTask,
    });
  },
  remoceTaskFromSelectedJob(state, payload) {
    this.commit("ros/jobs/removeTaskFromJob", {
      jobId: state.navigation.selectedJobId,
      index: payload.index,
    });
  },
  setThetaEndOfSelectedTask(state, payload) {
    this.commit("ros/jobs/setThetaEndOfTask", {
      jobId: state.navigation.selectedJobId,
      index: state.navigation.selectedIndexTask,
      mouse: payload.mouse,
    })
  },
  moveSelectedAgv(state, payload) {
    this.commit("ros/agvs/chanegAgvPosition", {
      agvId: state.navigation.selectedAgvId,
      movementX: payload.movementX,
      movementY: payload.movementY,
    })
  },
  changeSelectedAgvDirection(state, payload) {
    this.commit("ros/agvs/changeAgvDirection", {
      agvId: state.navigation.selectedAgvId,
      mouse: payload.mouse,
    })
  },
  moveSelectedAgvToCenterMap(state) {
    let webMap = state.agvMaps.find(map => map.id === state.navigation.selectedMapId).webMap;
    this.commit("ros/agvs/setAgvPosition", {
      agvId: state.navigation.selectedAgvId,
      x: webMap.width / 2,
      y: webMap.height / 2,
    });
  },
  setSelectedAgvPosition(state, payload) {
    this.commit("ros/agvs/setAgvPosition", {
      agvId: state.navigation.selectedAgvId,
      x: payload.x,
      y: payload.y,
    });
  },
  setSelectedAgvDirection(state, payload) {
    this.commit("ros/agvs/setAgvDirection", {
      agvId: state.navigation.selectedAgvId,
      theta: payload.theta,
    });
  },
  addGoalToCar(state) {
    let endPoint = {
      x: 0,
      y: 0,
      theta: 0,
    };
    const selectedJob = state.agvJobs.find(job => job.id === state.navigation.selectedJobId);
    if (selectedJob.tasks.length == 0) {
      const selectedAgv = state.agvs.find(agv => agv.id === state.navigation.selectedAgvId)
      endPoint.x = selectedAgv.webPose.position.x;
      endPoint.y = selectedAgv.webPose.position.y;
      endPoint.theta = selectedAgv.webPose.orientation.theta;
    } else {
      let lasttask = selectedJob.tasks[selectedJob.tasks.length - 1];
      endPoint.x = lasttask.position.x;
      endPoint.y = lasttask.position.y;
      endPoint.theta = lasttask.thetaEnd;
    }
    let point = {
      x: endPoint.x + Math.cos(endPoint.theta * Math.PI / 180) * 100,
      y: endPoint.y - Math.sin(endPoint.theta * Math.PI / 180) * 100,
    }
    this.commit("ros/jobs/pushTaskToJob", {
      jobId: state.navigation.selectedJobId,
      x: point.x,
      y: point.y,
    })
  },
  setSelectedAgvStatus(state, status) {
    this.commit("ros/agvs/setAgvStatus", {
      agvId: state.navigation.selectedAgvId,
      status: status,
    })
  },
  updateActionsOfSelectedTask(state, payload) {
    this.commit("ros/jobs/updateActions", {
      jobId: state.navigation.selectedJobId,
      index: state.navigation.selectedIndexTask,
      actions: payload.actions,
    });
  }
}