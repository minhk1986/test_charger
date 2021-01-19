import { AGVR } from "@/services";

export default {
  install: (Vue, mapId) => {

    Vue.prototype.$agvr = new AGVR(mapId);
    let store = Vue.prototype.$store;

    function updateWindowResize() {
      store.dispatch("navigation/changeWindowSize", {
        width: window.innerWidth,
        height: window.innerHeight - 96,
      });
    }

    function resumPanZoom() {
      if (store.state.navigation.taskMenuContext.isShow ||
        store.state.navigation.agvMenuContext.isShow ||
        store.state.navigation.agvInitPoseMenuContext.isShow) return;
      store.dispatch("navigation/enableMapPanZoom");
      store.dispatch("navigation/selectedNone");
    }

    function windowKeydownHandle(e) {
      switch (e.key) {
        case "Delete":
          store.dispatch("agvJobs/removeTaskFromJob", {
            jobId: store.state.navigation.jobId,
            taskId: store.state.navigation.taskId,
          });
          break;
      }
    }

    window.addEventListener("mouseup", resumPanZoom);
    window.addEventListener("touchend", resumPanZoom);
    window.addEventListener("keydown", windowKeydownHandle);
    window.addEventListener("resize", updateWindowResize);

    updateWindowResize();
  }
}