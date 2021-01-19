import ROSLIB from "roslib";
import Vue from "vue";

const FIXED_MAP_ID = "6defbba5-0f1f-4b43-a32e-a24247bac7d8";

export default class AgvrMap {
  #id = null;
  #ros = null;
  #rosUrl = null;
  #rosStore = null;
  #mapTopic = null;
  #simPoseTopic = null;

  constructor(id) {
    this.#id = id;
    this.#rosStore = Vue.prototype.$store;
    this.#rosUrl = this.#rosStore.state.agvMaps.find((m) => m.id === this.#id).rosMasterUri;
    this.#rosConnect();
  }

  #rosConnect = () => {
    this.#ros = new ROSLIB.Ros({ url: this.#rosUrl });
    this.#ros.on("connection", this.#onRosConnected);
    this.#ros.on("close", this.#onRosDisconnected);
    this.#ros.on("error", (e) => console.log("AGVR error: ", e));
  }

  #onRosConnected = () => {
    this.#mapTopic = new ROSLIB.Topic({
      ros: this.#ros,
      name: "/map",
      messageType: "nav_msgs/OccupancyGrid",
    });
    this.#mapTopic.subscribe(this.#onMapTopicReceived);
    this.#rosStore.commit("agvMaps/setConnectionState", {
      id: this.#id,
      isConnected: true,
    })
  }

  #onRosDisconnected = () => {
    if (this.#mapTopic != null) {
      this.#mapTopic = null;
      this.#rosStore.commit("agvMaps/setConnectionState", {
        id: this.#id,
        isConnected: true,
      })
    }
    setTimeout(this.#rosConnect, 3000);
  }

  dispose() {
    this.#mapTopic = null;
  }

  #onMapTopicReceived = (map) => {
    this.#rosStore.commit("agvMaps/setRosMap", { id: FIXED_MAP_ID, map, });
  }
}