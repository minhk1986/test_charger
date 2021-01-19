import ROSLIB from "roslib";
import Vue from "vue";
import { Mutex } from 'async-mutex';


export default class AgvrMoveBase {
  #rosNotify = null;
  #moveGoalTopic = null;
  #ros = null;
  #moveMutex = null;

  constructor(options = { ros: null, store: null }) {
    if (!(options.ros instanceof ROSLIB.Ros)) return;

    this.#moveMutex = new Mutex();

    this.#ros = options.ros;
    this.#rosNotify = Vue.prototype.$notify;
    this.#moveGoalTopic = new ROSLIB.Topic({
      ros: this.#ros,
      name: "/move_base_simple/goal",
      messageType: 'geometry_msgs/PoseStamped'
    });
  }

  async pushGoal(pose) {
    if (this.#moveGoalTopic == null || pose == null) return;
    let stamp = Date.now();
    let secs = Math.floor(stamp / 1000);
    let nsecs = (stamp % 1000) * 1000000;
    let goalId = `goal-webap-${secs}.${nsecs}`;
    let message = {
      header: {
        seq: 0,
        stamp: {
          secs: secs,
          nsecs: nsecs,
        },
        frame_id: "map",
      },
      pose: pose,
    }
    this.#moveGoalTopic.publish(new ROSLIB.Message(message));
    this.#confirmMoveGoal(goalId);

    await this.#moveMutex.acquire();
  }

  dispose() {
    this.#moveGoalTopic = null;
  }

  #confirmMoveGoal = (goalId) => {
    let moveBaseStatus = new ROSLIB.Topic({
      ros: this.#ros,
      name: "/move_base/status",
      messageType: "actionlib_msgs/GoalStatusArray",
    });
    moveBaseStatus.subscribe((message) => {
      if (!Array.isArray(message.status_list)) return;
      let goalStatus = message.status_list.find(s => s.goal_id && s.goal_id.id == goalId);
      if (goalStatus == null) return;
      if (goalStatus.status > 0) {
        moveBaseStatus.unsubscribe();
        switch (goalStatus.status) {
          case 1: //ACTIVE
            this.#onMoveGoalActive(goalId);
            break;
          case 3: //SUCCEEDED
            this.#moveGoalSuccess(goalStatus.text);
            break;
          default:
            console.log(goalStatus);
            this.#rosNotify({
              group: 'ros',
              type: 'warn',
              title: 'Move status',
              text: goalStatus.text,
            });
            break;
        }
      }
    });
  }

  #onMoveGoalActive = (goalId) => {
    console.log("onMoveGoalActive");
    let moveBaseResult = new ROSLIB.Topic({
      ros: this.#ros,
      name: "/move_base/result",
      messageType: "move_base_msgs/MoveBaseActionResult",
    });
    moveBaseResult.subscribe((message) => {
      if (message.status.goal_id.id != goalId) return;
      moveBaseResult.unsubscribe();
      if (message.status.status === 3) {
        this.#moveGoalSuccess(message.status.text);
      }
      else {
        console.log(message.status);
        this.#rosNotify({
          group: 'ros',
          type: 'warn',
          title: 'Move status',
          text: message.status.text,
        });
      }
    })
  }

  #moveGoalSuccess = async (message) => {
    console.log("moveGoalSuccess");
    this.#rosNotify({
      group: 'ros',
      type: 'success',
      title: 'Move status',
      text: message,
    });
    this.#moveMutex.release();
  }
}