import ROSLIB from "roslib";
import * as ACTION_STATUS from './agv-action-status';
import { uuid } from "vue-uuid";

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

export default class AgvrAction {
  #agvActionTopic = null;
  #agvActionStatusTopic = null;
  #actions = {};

  constructor(ros) {
    if (!(ros instanceof ROSLIB.Ros)) return;

    this.#agvActionTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/agv_action',
      messageType: 'agv_define/agv_action',
    });

    this.#agvActionStatusTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/action_status',
      messageType: 'agv_define/agv_action',
    });

    this.#agvActionStatusTopic.subscribe(this.#actionStatusTopicReceived);
  }

  #actionStatusTopicReceived = (message) => {
    this.#actions[message.action_id] = message.status;
  }
  

  async pushAction(action, waitResult = false) {
    if (this.#agvActionTopic == null) return ACTION_STATUS.REJECTED;

    let actionId = `action-${uuid.v4()}`;
    if(waitResult === true) {
      this.#actions[actionId] = ACTION_STATUS.PENDING;
    }

    this.#agvActionTopic.publish(new ROSLIB.Message({
      header: { seq: 0, stamp: 0, frame_id: '', },
      action: action.value,
      status: ACTION_STATUS.ACTIVE,
      action_id: actionId,
      type: action.type,
      data: JSON.stringify(action.data),
    }))

    if(waitResult === true) {
      let count = 0;
      while(this.#actions[actionId] === ACTION_STATUS.PENDING && count < 300) {
        count++;
        await sleep(500);
      }

      if(this.#actions[actionId] === ACTION_STATUS.ACTIVE) {
        while(this.#actions[actionId] === ACTION_STATUS.ACTIVE) {
          await sleep(500);
        }
      }
      return this.#actions[actionId]; 
    }
  }

  #publishAction = (action) => {
    if (this.#agvActionTopic == null) return;

    let actionId = `action-${uuid.v4()}`;
    this.#agvActionTopic.publish(new ROSLIB.Message({
      header: { seq: 0, stamp: 0, frame_id: '', },
      action: action.value,
      status: ACTION_STATUS.ACTIVE,
      action_id: actionId,
      type: action.type,
      data: JSON.stringify(action.data),
    }))
  }

  dispose() {
    this.#agvActionTopic = null;
  }
}