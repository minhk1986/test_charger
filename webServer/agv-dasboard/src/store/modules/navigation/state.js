export default {
  selectedMapId: "6defbba5-0f1f-4b43-a32e-a24247bac7d8",
  selectedAgvId: "8ee52be3-90d0-4ea9-8af5-b0c689a59030",
  selectedJobId: "cca29e49-d556-4c1a-af17-3f870b50ca06",
  selectionType: -1,
  selectedIndexTask: -1,
  mapPanZoomEnable: true,
  taskMenuContext: {
    position: {
      x: 0,
      y: 0,
    },
    isShow: false,
  },
  agvMenuContext: {
    position: {
      x: 0,
      y: 0,
    },
    isShow: false,
  },
  agvInitPoseMenuContext: {
    position: {
      x: 0,
      y: 0,
    },
    isShow: false,
  },
  actionManagement: {
    isShow: false,
  },
  joystickButton: {
    isShow: false,
  },
  movementButton: {
    isShow: false,
  },
  navigationState: "navigation",
  viewHeight: 0,
  viewWidth: 0,

  mapId: process.env.VUE_APP_AGV_MAP_ID,
  jobId: "cca29e49-d556-4c1a-af17-3f870b50ca06",
  agvId: "8ee52be3-90d0-4ea9-8af5-b0c689a59030",
  taskId: "",
  subscribePose: false,
}