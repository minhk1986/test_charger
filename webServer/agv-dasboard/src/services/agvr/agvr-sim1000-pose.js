import ROSLIB from "roslib";
import Vue from "vue";

export default class AgvrSim1000Pose {
  #id = null;
  #map = null;
  #rosStore = null;
  #simPoseTopic = null;

  constructor(ros, id, mapId) {
    if (!(ros instanceof ROSLIB.Ros)) return;

    this.#rosStore = Vue.prototype.$store;
    this.#id = id;
    this.#map = this.#rosStore.state.agvMaps.find((m) => m.id === mapId);
    this.#simPoseTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/sick_lidar_localization/driver/result_telegrams",
      messageType: "sick_lidar_localization/SickLocResultPortTelegramMsg",
    });
    this.#simPoseTopic.subscribe(this.#onSimPoseReceived);
  }

  dispose() {
    if (this.#simPoseTopic != null) {
      this.#simPoseTopic.unsubscribe();
      this.#simPoseTopic = null;
    }
  }
  #onSimPoseReceived = (message) => {
    if (this.#rosStore.state.navigation.subscribePose) {
      this.#rosStore.commit("agvs/setSim1000Pose", {
        pose: message.telegram_payload,
        agvId: this.#id,
        rosMap: this.#map.rosMap,
        mapScale: this.#map.webMap.scale,
      })
    }
  }
}