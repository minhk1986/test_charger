import ROSLIB from "roslib";
import Vue from 'vue';
import { ROS2D } from '@/utils/ros';

export default class AgvrInitializePose {
  #initPoseTopic = null;
  #rosStore = null;

  constructor(ros) {
    if (!(ros instanceof ROSLIB.Ros)) return;

    this.#rosStore = Vue.prototype.$store;
    this.#initPoseTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/initialpose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    });
  }

  initialize() {
    if (this.#initPoseTopic == null) return;

    let agvPose = this.#rosStore.state.agvs.find(agv => agv.id === this.#rosStore.state.navigation.selectedAgvId).webPose;
    let map = this.#rosStore.state.agvMaps.find(map => map.id === this.#rosStore.state.navigation.selectedMapId);

    let initPose = {
      position: {
        x: map.rosMap.info.origin.position.x + agvPose.position.x * map.rosMap.info.resolution / map.webMap.scale,
        y: map.rosMap.info.origin.position.y - ((agvPose.position.y / map.webMap.scale) - map.rosMap.info.height) * map.rosMap.info.resolution,
        z: 0,
      },
      orientation: ROS2D.globalThetaToQuaternion(agvPose.orientation.theta * Math.PI / 180),
    };

    let message = {
      header: { seq: 0, stamp: 0, frame_id: 'map', },
      pose: {
        pose: initPose,
        covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
      },
    }

    this.#initPoseTopic.publish(new ROSLIB.Message(message));
  }

  dispose() {
    this.#initPoseTopic = null;
  }
}