import AgvrMap from "./agvr-map";
import AgvrRobot from "./agvr-robot";
import Vue from "vue";

export default class AGVR {
  #agvrMap = null;
  robots = {};

  constructor(mapId) {
    let agvIds = Vue.prototype.$store.state.agvMaps.find((m) => m.id === mapId).agvIds;
    this.#agvrMap = new AgvrMap(mapId);
    for (var i = 0; i < agvIds.length; i++) {
      this.robots[agvIds[i]] = new AgvrRobot(agvIds[i], mapId);
    }
  }
}