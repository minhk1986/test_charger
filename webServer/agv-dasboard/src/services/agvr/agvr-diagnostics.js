import ROSLIB from "roslib";
import Vue from 'vue';

export default class AgvrDiagnostics {
  #rosStore = null;
  #diagnosticsTopic = null;
  #agvId = null;

  constructor(ros, id) {
    if (!(ros instanceof ROSLIB.Ros)) return;

    this.#agvId = id;
    this.#rosStore = Vue.prototype.$store;
    this.#diagnosticsTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/diagnostics_agg',
      messageType: 'diagnostic_msgs/DiagnosticArray'
    });

    this.#diagnosticsTopic.subscribe(this.#onDiagnosticsReceived);
  }

  dispose() {
    if(this.#diagnosticsTopic != null) {
      this.#diagnosticsTopic.unsubscribe();
      this.#diagnosticsTopic = null;
    }
  }

  #onDiagnosticsReceived = (message) => {
    this.#rosStore.dispatch("agvs/setAgvStatus", {
      agvId: this.#agvId,
      status: message.status,
    });
  }
}