import { uuid } from "vue-uuid";

export default {
  pushTaskToJob(state, payload) {
    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    let theta = 0;
    if (job.tasks.length > 0) {
      let lastPoint = job.tasks[job.tasks.length - 1].position;
      theta = (Math.atan2(lastPoint.y - payload.y, payload.x - lastPoint.x) * 180) / Math.PI;
    }
    let id = uuid.v4();
    while (id in job.tasks) { id = uuid.v4(); }
    job.tasks.push({
      id: id,
      position: {
        x: payload.x,
        y: payload.y,
        z: 0,
      },
      thetaEnd: theta,
      actions: [],
    });
  },
  moveTaskPosition(state, payload) {
    if (isNaN(payload.movementX) || isNaN(payload.movementY)) return;

    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    let task = job.tasks.find((t) => t.id === payload.taskId);
    if (task == null) return;

    let newPosition = {
      x: task.position.x + payload.movementX,
      y: task.position.y + payload.movementY,
    };

    if(payload.smartPoints && payload.smartPoints.length > 0) {
      for(let i = 0; i < payload.smartPoints.length; i++) {
        if(Math.sqrt(Math.pow(payload.smartPoints[i].x - newPosition.x, 2) + Math.pow(payload.smartPoints[i].y - newPosition.y, 2)) < 15) {
          newPosition = {
            x: payload.smartPoints[i].x,
            y: payload.smartPoints[i].y,
          }
          break;
        } 
      }
    }
    task.position = newPosition;
  },
  setThetaEndOfTask(state, payload) {
    if (isNaN(payload.mouse.x) || isNaN(payload.mouse.y)) return;

    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    let task = job.tasks.find((t) => t.id === payload.taskId);
    if (task == null) return;

    let theta = Math.atan2(task.position.y - payload.mouse.y, payload.mouse.x - task.position.x);
    task.thetaEnd = theta * 180 / Math.PI;
  },
  insertTaskAfterToJob(state, payload) {
    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    let taskIndex = job.tasks.findIndex((t) => t.id === payload.taskId);
    if (taskIndex < 0 || taskIndex >= job.tasks.length - 1) return;

    let point = {
      x: (job.tasks[taskIndex].position.x + job.tasks[taskIndex + 1].position.x) / 2,
      y: (job.tasks[taskIndex].position.y + job.tasks[taskIndex + 1].position.y) / 2,
    };
    let lastPoint = job.tasks[taskIndex].position;
    job.tasks.splice(taskIndex + 1, 0, {
      id: uuid.v4(),
      position: {
        x: point.x,
        y: point.y,
        z: 0,
      },
      thetaEnd: (Math.atan2(lastPoint.y - point.y, point.x - lastPoint.x) * 180) / Math.PI,
      actions: [],
    });
  },
  insertTaskBeforeToJob(state, payload) {
    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    let taskIndex = job.tasks.findIndex((t) => t.id === payload.taskId);
    if (taskIndex >= job.tasks.length - 1) return;

    let lastPoint = null;
    if (taskIndex == 0) {
      lastPoint = payload.agvPosition;
    } else {
      lastPoint = job.tasks[taskIndex - 1].position;
    }
    let point = {
      x: (lastPoint.x + job.tasks[taskIndex].position.x) / 2,
      y: (lastPoint.y + job.tasks[taskIndex].position.y) / 2,
    };
    job.tasks.splice(taskIndex, 0, {
      id: uuid.v4(),
      position: {
        x: point.x,
        y: point.y,
        z: 0,
      },
      thetaEnd: (Math.atan2(lastPoint.y - point.y, point.x - lastPoint.x) * 180) / Math.PI,
      actions: [],
    });
  },
  removeTaskFromJob(state, payload) {
    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    let taskIndex = job.tasks.findIndex((t) => t.id === payload.taskId);
    if (taskIndex >= 0 && taskIndex < job.tasks.length) {
      job.tasks.splice(taskIndex, 1);
    }
  },
  clearTaskFromJob(state, payload) {
    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    job.tasks = [];
  },
  updateActions(state, payload) {
    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    let task = job.tasks.find(t => t.id === payload.taskId);
    if (job == null) return;

    task.actions = payload.actions;
    task.position = payload.position;
    task.thetaEnd = payload.thetaEnd;
  },
  setTasks(state, payload) {
    let job = state.find((j) => j.id === payload.jobId);
    if (job == null) return;

    job.tasks = payload.tasks;
  }
}