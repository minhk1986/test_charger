<template>
  <v-container fluid class="pa-0 agv-navigation">
    <panZoom
      class="panzoom-map-viewer"
      :selector="`#${graphicID}`"
      :options="{
        minZoom: 0.1,
        maxZoom: 5,
        zoomSpeed: 0.05,
        zoomDoubleClickSpeed: 1,
        onDoubleClick: onPanZoomDoubleClick,
        autocenter: true,
      }"
      @init="onInitZoomPan"
    >
      <svg
        xmlns="http://www.w3.org/2000/svg"
        class="map-viewer-svg"
        :height="(agvMapImage == null || viewHeight > agvMapImage.height) ? viewHeight : agvMapImage.height"
        :width="(agvMapImage == null || viewWidth > agvMapImage.width) ? viewWidth : agvMapImage.width"
        :id="graphicID"
        @mousemove="onMouseMove"
        @contextmenu="onContextMenu"
      >
        <g
          @touchstart="onTouchStart"
          @touchmove="onTouchMove"
          @touchend="onTouchEnd"
        >
          <AgvrMapImage :mapId="navigationMapId" />
          <g>
            <rect v-for="(r, index) in agvRects" :key="`rect-map-${index}`" :x="r.x - r.width / 2" :y="r.y - r.height / 2" :width="r.width" :height="r.height" :style="r.style" />
          </g>
          <AgvrMapSmartPoint :mapId="navigationMapId" />

          <WayFromTaskToTask
            v-for="jobId in agvJobsIdOnMap"
            :key="`line-des-${jobId}`"
            :jobId="jobId"
          />
          
          <WayAgvToTaskPosition
            v-for="agvId in agvIdsOnMap"
            :key="`way-from-agv-to-task-${agvId}`"
            :agvId="agvId"
          />

          <AgvrEndTaskDirection />

          <AgvrTasksOnMap
            v-for="jobId in agvJobsIdOnMap"
            :key="`agv-task-${jobId}`"
            :jobId="jobId"
          />

          <AgvrRobot
            v-for="agvId in agvIdsOnMap"
            :key="`agv-${agvId}`"
            :agvId="agvId"
            :agvMapResolution="agvMapResolution"
            :agvMapScale="agvMapScale"
          />
        </g>
      </svg>
    </panZoom>
    <AgvrRobotNavigationMenuContext />
    <AgvrTaskMenuContext />
    <AgvrActionManagement :jobId="navigationJobId" :taskId="navigationTaskId" />
    <span v-if="joystickButtonState" class="joystick">
      <AgvrJoyStickButton />
    </span>
    <span v-if="movementButtonState" class="movement">
      <AgvrMovementButton />
    </span>
  </v-container>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
import { uuid } from "vue-uuid";
import AgvrMapImage from "@/components/AgvrMapImage";
import WayAgvToTaskPosition from "@/components/WayAgvToTaskPosition";
import AgvrRobot from "@/components/AgvrRobot";
import WayFromTaskToTask from "@/components/WayFromTaskToTask";
import AgvrTasksOnMap from "@/components/AgvrTasksOnMap";
import AgvrEndTaskDirection from "@/components/AgvrEndTaskDirection";
import AgvrActionManagement from "@/components/AgvrActionManagement";
import AgvrJoyStickButton from "@/components/AgvrJoyStickButton";
import AgvrMovementButton from "@/components/AgvrMovementButton";
import AgvrMapSmartPoint from "@/components/AgvrMapSmartPoint";
import {
  AgvrRobotNavigationMenuContext,
  AgvrTaskMenuContext,
} from "@/components/MenuContexts";

const { mapState: navigationMapState, mapActions: navigationMapActions } = createNamespacedHelpers("navigation");
const { mapState: agvMapsMapState, mapGetters: agvMapsGetters } = createNamespacedHelpers("agvMaps");
const { mapActions: agvJobsMapActions } = createNamespacedHelpers("agvJobs");

export default {
  name: "AgvNavigation",
  components: {
    AgvrRobot,
    AgvrEndTaskDirection,
    AgvrMapImage,
    WayAgvToTaskPosition,
    WayFromTaskToTask,
    AgvrTasksOnMap,
    AgvrRobotNavigationMenuContext,
    AgvrTaskMenuContext,
    AgvrActionManagement,
    AgvrJoyStickButton,
    AgvrMovementButton,
    AgvrMapSmartPoint,
  },
  props: {
    map: {
      type: String,
      default: "",
    },
  },
  data: () => ({
    graphicID: `agv-map-zoompan-${uuid.v4()}`,
    panZoom: null,
    touchClientX: -1,
    touchClientY: -1,
    sgvWidth: 0,
    sgvHeight: 0,
  }),
  computed: {
    ...navigationMapState({
      navigationMapId: (state) => state.mapId,
      navigationJobId: (state) => state.jobId,
      navigationTaskId: (state) => state.taskId,
      navigationAgvId: (state) => state.agvId,
      navigationMapPanZoomEnable: (state) => state.mapPanZoomEnable,
      navigationSelectionType: (state) => state.selectionType,
      viewWidth: (state) => state.viewWidth,
      viewHeight: (state) => state.viewHeight,
      joystickButtonState: (state) => state.joystickButton.isShow,
      movementButtonState: (state) => state.movementButton.isShow,
    }),
    ...agvMapsGetters([
      "convertMapPoseToWebPose"
    ]),
    ...agvMapsMapState({
      agvMapImage(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.webMap.Image : null;
      },
      agvIdsOnMap(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.agvIds : [];
      },
      agvJobsIdOnMap(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.jobIds : [];
      },
      agvMapResolution(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.rosMap.info.resolution : 1;
      },
      agvMapScale(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.webMap.scale : 1;
      },
      agvMapSmartPoints(state) {
        let agvMap = state.find((m) => m.id === this.navigationMapId);

        return (agvMap != null) ? agvMap.smartWebPoints : [];
      },
      agvRects(state) {
        let agvMap = state.find((m) => m.id === this.navigationMapId);
        const mapId = this.navigationMapId;

        return (agvMap != null) ? agvMap.rects.map(r => {
          return {
            ...r,
            ...this.convertMapPoseToWebPose({id: mapId, x: r.x, y: r.y, width: r.width, height: r.height}),
          }
        }) : [];
      }
    }),
  },
  methods: {
    ...agvJobsMapActions([
      "pushTaskToJob",
      "moveTaskPosition",
      "setThetaEndOfTask",
    ]),
    ...navigationMapActions({
      navigationSubscribePose: "subscribePose",
      navigationUnsubscribePose: "unsubscribePose",
    }),
    onPanZoomDoubleClick(e) {
      let transform = this.panZoom.getTransform();
      let desPoint = {
        x: (e.layerX - transform.x) / transform.scale,
        y: (e.layerY - transform.y) / transform.scale,
      };

      this.pushTaskToJob({
        jobId: this.navigationJobId,
        x: desPoint.x,
        y: desPoint.y,
      });
    },
    onInitZoomPan(e) {
      this.panZoom = e;
    },
    onMouseMove(e) {
      let transform = this.panZoom.getTransform();
      switch (this.navigationSelectionType) {
        case 1:
          this.moveTaskPosition({
            jobId: this.navigationJobId,
            taskId: this.navigationTaskId,
            movementX: e.movementX / transform.scale,
            movementY: e.movementY / transform.scale,
            smartPoints: this.agvMapSmartPoints,
          });
          break;
        case 2:
          this.setThetaEndOfTask({
            jobId: this.navigationJobId,
            taskId: this.navigationTaskId,
            mouse: {
              x: (e.layerX - transform.x) / transform.scale,
              y: (e.layerY - transform.y) / transform.scale,
            },
          });
          break;
      }
    },
    onTouchStart(e) {
      let touch = e.changedTouches[0];
      this.touchClientX = touch.clientX;
      this.touchClientY = touch.clientY;
    },
    onTouchEnd() {
      this.touchClientX = -1;
      this.touchClientY = -1;
    },
    onTouchMove(e) {
      e.preventDefault();
      if (this.touchClientX > 0 && this.touchClientY > 0) {
        let touch = e.changedTouches[0];
        this.onMouseMove({
          movementX: touch.clientX - this.touchClientX,
          movementY: touch.clientY - this.touchClientY,
          layerX: touch.clientX,
          layerY: touch.clientY,
        });

        this.touchClientX = touch.clientX;
        this.touchClientY = touch.clientY;
      }
    },
    onContextMenu(e) {
      e.preventDefault();
      e.stopPropagation();
    },
  },
  watch: {
    navigationMapPanZoomEnable(val) {
      if (val) {
        if (this.panZoom.isPaused()) {
          this.panZoom.resume();
        }
      } else {
        if (!this.panZoom.isPaused()) {
          this.panZoom.pause();
        }
      }
    },
    agvMapImage(val) {
      if (val == null) return;
      this.sgvWidth = val.width;
      this.sgvHeight = val.height;
    },
  },
  mounted() {
    this.navigationSubscribePose();
    if (this.agvMapImage != null) {
      this.sgvWidth = this.agvMapImage.width;
      this.sgvHeight = this.agvMapImage.height;
    }
  },
  beforeDestroy(){
    this.navigationUnsubscribePose();
  }
};
</script>

<style scoped>
.panzoom-map-viewer {
  border: none;
  width: 100%;
  height: 100%;
}
.map-viewer-svg {
  position: relative;
}
.agv-navigation .joystick {
  position: absolute;
  bottom: 4px;
  left: 4px;
}
.agv-navigation .movement {
  position: absolute;
  bottom: 4px;
  right: 4px;
}
</style>
