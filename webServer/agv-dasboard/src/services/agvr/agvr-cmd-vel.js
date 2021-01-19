import ROSLIB from "roslib";
import Vue from 'vue';

export default class AgvrCmdVel {
  #cmdvelTopic = null;
  #rosStore = null;
  #timerPublishCmdVel = null;
  #intervalPush = 100;
  #agv = null;
  #twistCmdVel = null;

  constructor(ros, agvId, interval) {
    if (!Number.isInteger(interval) || !(ros instanceof ROSLIB.Ros)) return;
    this.#intervalPush = interval;
    this.#rosStore = Vue.prototype.$store;
    this.#agv = this.#rosStore.state.agvs.find(a => a.id === agvId);
    this.#cmdvelTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/Twist",
    });
  }

  pushTwist(twist) {
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

    if (this.#cmdvelTopic == null || this.#twistCmdVel == null) return;

    if (this.#timerPublishCmdVel != null) clearInterval(this.#timerPublishCmdVel);
    this.#cmdvelTopic.publish(new ROSLIB.Message(this.#twistCmdVel));

    if (this.#twistCmdVel.linear.x != 0 || this.#twistCmdVel.angular.z != 0) {
      this.#timerPublishCmdVel = setInterval(() => {
        if (this.#cmdvelTopic == null || this.#twistCmdVel == null) return;
        this.#cmdvelTopic.publish(new ROSLIB.Message(this.#twistCmdVel));
      }, this.#intervalPush);
    }
  }

  dispose() {
    if (this.#timerPublishCmdVel != null) clearInterval(this.#timerPublishCmdVel);
    this.#cmdvelTopic = null;
  }
}