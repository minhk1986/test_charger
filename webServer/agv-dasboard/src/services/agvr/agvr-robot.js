import ROSLIB from "roslib";
import Vue from "vue";
import AgvrDiagnostics from "./agvr-diagnostics";
import AgvrAction from "./agvr-action";
import AgvrSim1000Pose from "./agvr-sim1000-pose";
import { ROS2D } from '@/utils/ros';
import * as AGV_ACTION_STATUS from './agv-action-status';
import * as AGV_ACTION from "./agv-action-type";

export default class AgvrRobot {
  #id = null;
  #agv = null;
  #mapId = null;
  #map = null;
  #ros = null;
  #rosUrl = null;
  #rosStore = null;

  #agvrDiagnostics = null;
  #agvrAction = null;
  #agvrSim1000Pose = null;

  #twistCmdVel = null;
  #timerPublishCmdVel = null;
  #intervalPush = 100;

  constructor(id, mapId) {
    this.#id = id;
    this.#mapId = mapId;
    this.#rosStore = Vue.prototype.$store;
    this.#map = this.#rosStore.state.agvMaps.find((m) => m.id === mapId);
    this.#agv = this.#rosStore.state.agvs.find((r) => r.id === this.#id);
    this.#rosUrl = this.#agv.rosMasterUri;
    this.#rosConnect();
  }

  #rosConnect = () => {
    this.#ros = new ROSLIB.Ros({ url: this.#rosUrl });
    this.#ros.on("connection", this.#onRosConnected);
    this.#ros.on("close", this.#onRosDisconnected);
    this.#ros.on("error", (e) => console.log("AGVR error: ", e));
  }

  #onRosConnected = () => {
    this.#agvrAction = new AgvrAction(this.#ros);
    this.#agvrDiagnostics = new AgvrDiagnostics(this.#ros, this.#id);
    this.#agvrSim1000Pose = new AgvrSim1000Pose(this.#ros, this.#id, this.#mapId);

    this.#rosStore.commit("agvs/setConnectionState", {
      agvId: this.#id,
      isConnected: true,
    })
  }

  #onRosDisconnected = () => {
    if (this.#agvrDiagnostics != null) {
      this.#agvrDiagnostics.dispose();
      this.#agvrDiagnostics = null;
    }
    if (this.#agvrAction != null) {
      this.#agvrAction.dispose();
      this.#agvrAction = null;
    }
    if (this.#agvrSim1000Pose != null) {
      this.#agvrSim1000Pose.dispose();
      this.#agvrSim1000Pose = null;
    }

    this.#rosStore.commit("agvs/setConnectionState", {
      agvId: this.#id,
      isConnected: false,
    });

    setTimeout(this.#rosConnect, 3000);
  }

  async pushCmdvel(twist) {
    if (twist == null) return;

    let speed = this.#agv.properties;
    this.#twistCmdVel = {
      linear: {
        x: twist.linear.x * speed.movementSpeed.x,
        y: twist.linear.y * speed.movementSpeed.y,
        z: twist.linear.z * speed.movementSpeed.z,
      },
      angular: {
        x: twist.angular.x * speed.rotateSpeed.x,
        y: twist.angular.y * speed.rotateSpeed.y,
        z: twist.angular.z * speed.rotateSpeed.z,
      },
    };

    if (this.#timerPublishCmdVel != null) clearInterval(this.#timerPublishCmdVel);
    await this.pushAction({
      value: AGV_ACTION.MANUAL,
      type: "geometry_msgs/Twist",
      data: this.#twistCmdVel,
    }, false);

    if (this.#twistCmdVel.linear.x != 0 || this.#twistCmdVel.angular.z != 0) {
      this.#timerPublishCmdVel = setInterval(async () => {
        if (this.#agvrAction == null || this.#twistCmdVel == null) return;
        await this.#agvrAction.pushAction({
          value: AGV_ACTION.MANUAL,
          type: "geometry_msgs/Twist",
          data: this.#twistCmdVel,
        }, false);
      }, this.#intervalPush);
    }
  }

  initializePose() {
    let agvPose = this.#agv.webPose;
    this.pushAction({
      value: AGV_ACTION.INITIAL_POSE,
      type: "geometry_msgs/PoseStamped",
      data: {
        header: {
          seq: 0,
          stamp: {
            sec: 0,
            nsec: 0,
          },
          frame_id: '',
        },
        pose: {
          position: {
            x: this.#map.rosMap.info.origin.position.x + agvPose.position.x * this.#map.rosMap.info.resolution / this.#map.webMap.scale,
            y: this.#map.rosMap.info.origin.position.y - ((agvPose.position.y / this.#map.webMap.scale) - this.#map.rosMap.info.height) * this.#map.rosMap.info.resolution,
            z: 0,
          },
          orientation: ROS2D.globalThetaToQuaternion(agvPose.orientation.z * Math.PI / 180),
        }
      },
    })
  }

  async pushAction(action, waitResult = true) {
    if (this.#agvrAction == null) return null;

    if(waitResult) console.log('push action ', action);
    return await this.#agvrAction.pushAction(action, waitResult);
  }

  async liftUp() {
    await this.pushAction({
      value: AGV_ACTION.LIFT_UP,
      type: "geometry_msgs/Twist",
      data: {
        linear: {
          x: 0,
          y: 0,
          z: 0,
        },
        angular: {
          x: 0,
          y: 0,
          z: 0,
        },
      },
    })
  }

  async liftDown() {
    await this.pushAction({
      value: AGV_ACTION.LIFT_DOWN,
      type: "geometry_msgs/Twist",
      data: {
        linear: { x: 0, y: 0, z: 0, },
        angular: { x: 0, y: 0, z: 0, },
      },
    })
  }

  async chargingStart() {
    await this.pushAction({
      value: AGV_ACTION.CHARGING_IN,
      type: "geometry_msgs/Twist",
      data: {
        linear: { x: 0, y: 0, z: 0, },
        angular: { x: 0, y: 0, z: 0, },
      },
    })
  }

  async chargingEnd() {
    let agvPose = this.#agv.sim1000Pose;
    let agvAngle = agvPose.poseyaw * Math.PI / 180000;
    let poseRotate = {
      orientation: ROS2D.globalThetaToQuaternion(agvAngle),
      position: {
        x: (agvPose.posex + 2000 * Math.cos(agvAngle)) / 1000,
        y: (agvPose.posey + 2000 * Math.sin(agvAngle)) / 1000,
        z: 0,
      },
    };
    
    await this.pushAction({
      value: AGV_ACTION.CHARGING_OUT,
      type: "geometry_msgs/PoseStamped",
      data: {
        header: {
          seq: 0,
          stamp: {
            sec: 0,
            nsec: 0,
          },
          frame_id: 'map',
        },
        pose: poseRotate,
      },
    });
    console.log('chargingEnd');
  }

  async liftIn() {
    await this.pushAction({
      value: AGV_ACTION.LIFT_IN,
      type: "geometry_msgs/Twist",
      data: {
        linear: { x: 0, y: 0, z: 0, },
        angular: { x: 0, y: 0, z: 0, },
      },
    })
  }

  async liftOut() {
    let agvPose = this.#agv.sim1000Pose;
    let agvAngle = agvPose.poseyaw * Math.PI / 180000;
    let poseRotate = {
      orientation: ROS2D.globalThetaToQuaternion(agvAngle),
      position: {
        x: (agvPose.posex + 2000 * Math.cos(agvAngle)) / 1000,
        y: (agvPose.posey + 2000 * Math.sin(agvAngle)) / 1000,
        z: 0,
      },
    };
    
    await this.pushAction({
      value: AGV_ACTION.LIFT_OUT,
      type: "geometry_msgs/PoseStamped",
      data: {
        header: {
          seq: 0,
          stamp: {
            sec: 0,
            nsec: 0,
          },
          frame_id: 'map',
        },
        pose: poseRotate,
      },
    });
    console.log('liftOut');
  }

  async runJob() {
    let job = this.#rosStore.state.agvJobs.find(j => j.id === this.#agv.jobId);
    if (job == null || job.tasks.length == 0) return;

    let mapScale = this.#map.webMap.scale;
    let mapRosInfo = this.#map.rosMap.info;
    let result = 0;
    let done = true;
    this.#rosStore.commit("agvs/setRunningState", {
      agvId: this.#id,
      isRunnging: true,
    });
    for (var i = 0; i < job.tasks.length; i++) {
      let task = job.tasks[i];
      this.#rosStore.commit("agvs/setAgvTask", {
        agvId: this.#id,
        taskId: task.id,
      })

      let agvPose = this.#agv.sim1000Pose;
      let pose = {
        orientation: ROS2D.globalThetaToQuaternion(task.thetaEnd * Math.PI / 180),
        position: {
          x: mapRosInfo.origin.position.x + task.position.x * mapRosInfo.resolution / mapScale,
          y: mapRosInfo.origin.position.y - ((task.position.y / mapScale) - mapRosInfo.height) * mapRosInfo.resolution,
          z: 0,
        },
      };
      let poseRotate = {
        orientation: ROS2D.globalThetaToQuaternion(Math.atan2(pose.position.y - agvPose.posey / 1000, pose.position.x - agvPose.posex / 1000)),
        position: {
          x: agvPose.posex / 1000,
          y: agvPose.posey / 1000,
          z: 0,
        },
      };
      result = await this.pushAction({
        value: AGV_ACTION.ROTATE_GOAL,
        type: "geometry_msgs/PoseStamped",
        data: {
          header: {
            seq: 0,
            stamp: {
              sec: 0,
              nsec: 0,
            },
            frame_id: 'map',
          },
          pose: poseRotate,
        },
      });
      console.log('result action rotate = ', result);
      if(result != AGV_ACTION_STATUS.SUCCEEDED) done = false;
      if(!done) break;

      result = await this.pushAction({
        value: AGV_ACTION.MOVE_GOAL,
        type: "geometry_msgs/PoseStamped",
        data: {
          header: { 
            seq: 0, 
            stamp: {
              sec: 0,
              nsec: 0,
            }, 
            frame_id: 'map', 
          },
          pose,
        },
      });
      console.log('result action move goal = ', result);
      if(result != AGV_ACTION_STATUS.SUCCEEDED) done = false;
      if(!done) break;

      for(let j = 0; j < task.actions.length; j++) {
        let action = task.actions[j];
        switch(action.action) {
          case AGV_ACTION.CHARGING_OUT:
            await this.chargingEnd();
            break;
          case AGV_ACTION.LIFT_OUT:
            await this.liftOut();
            break;
          case AGV_ACTION.CHARGING_IN:
          case AGV_ACTION.LIFT_IN:
          case AGV_ACTION.LIFT_UP:
          case AGV_ACTION.LIFT_DOWN:
            result = await this.pushAction({
              value: action.action,
              type: "geometry_msgs/PoseStamped",
              data: {
                header: { 
                  seq: 0, 
                  stamp: {
                    sec: 0,
                    nsec: 0,
                  }, 
                  frame_id: 'map', 
                },
                pose,
              },
            });
            console.log('result action doing action ', action.name, " = ", result);
            if(result != AGV_ACTION_STATUS.SUCCEEDED) done = false;
        }
        if(!done) break;
      }
      if(!done) break;
    }
    this.#rosStore.commit("agvs/setRunningState", {
      agvId: this.#id,
      isRunnging: false,
    });
    if(done) {
      this.#rosStore.commit("agvJobs/clearTaskFromJob", {jobId: this.#agv.jobId});
    }
  }
}