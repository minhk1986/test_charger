<template>
  <v-container fluid class="pa-0 agv-navigation">
    <panZoom
      class="panzoom-map-initialize-pose-viewer"
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
          <AgvrRobotDirection :agvId="navigationAgvId" />
          <AgvrRobot
            :selectable="true"
            :agvId="navigationAgvId"
            :agvMapResolution="agvMapResolution"
            :agvMapScale="agvMapScale"
          />
        </g>
      </svg>
    </panZoom>
    <AgvrRobotInitPoseMenuContext />
  </v-container>
</template>
<script>
import { createNamespacedHelpers, mapState } from "vuex";
import { uuid } from "vue-uuid";
import AgvrMapImage from "@/components/AgvrMapImage";
import AgvrRobot from "@/components/AgvrRobot";
import AgvrRobotDirection from "@/components/AgvrRobotDirection";
import {
  AgvrRobotInitPoseMenuContext,
} from "@/components/MenuContexts";

const { mapState: navigationMapState } = createNamespacedHelpers("navigation");
const { mapState: agvMapsMapState } = createNamespacedHelpers("agvMaps");
const { mapActions: agvsMapActions } = createNamespacedHelpers("agvs");

export default {
  name: "AgvInitializeNavigation",
  components: {
    AgvrRobot,
    AgvrRobotDirection,
    AgvrMapImage,
    AgvrRobotInitPoseMenuContext,
  },
  data() {
    return {
      graphicID: `agv-map-zoompan-initialize-pose-${uuid.v4()}`,
      panZoom: null,
      touchClientX: -1,
      touchClientY: -1,
      sgvWidth: 0,
      sgvHeight: 0,
    };
  },
  computed: {
    ...navigationMapState({
      navigationMapId: (state) => state.mapId,
      navigationAgvId: (state) => state.agvId,
    }),
    ...agvMapsMapState({
      agvMapImage(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.webMap.Image : null;
      },
      agvMapResolution(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.rosMap.info.resolution : 1;
      },
      agvMapScale(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.webMap.scale : 1;
      },
    }),
    ...mapState({
      navigationWebMap: (state) =>
        state.ros.maps[state.navigation.selectedMapId].webMap,
      viewWidth: (state) => state.navigation.viewWidth,
      viewHeight: (state) => state.navigation.viewHeight,
      navigationSelectionType: (state) => state.navigation.selectionType,
      navigationMapPanZoomEnable: (state) => state.navigation.mapPanZoomEnable,
    }),
  },
  methods: {
    ...agvsMapActions({
      moveAgvPosition: "moveAgvPosition",
      changeAgvDirection: "changeAgvDirection",
    }),
    onInitZoomPan(e) {
      this.panZoom = e;
    },
    onMouseMove(e) {
      let transform = this.panZoom.getTransform();
      switch (this.navigationSelectionType) {
        case 3:
          this.moveAgvPosition({
            movementX: e.movementX / transform.scale,
            movementY: e.movementY / transform.scale,
            agvId: this.navigationAgvId,
          });
          break;
        case 4:
          this.changeAgvDirection({
            mouse: {
              x: (e.layerX - transform.x) / transform.scale,
              y: (e.layerY - transform.y) / transform.scale,
            },
            agvId: this.navigationAgvId,
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
    onPanZoomDoubleClick() {},
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
    if (this.agvMapImage != null) {
      this.sgvWidth = this.agvMapImage.width;
      this.sgvHeight = this.agvMapImage.height;
    }
  },
};
</script>

<style scoped>
.panzoom-map-initialize-pose-viewer {
  border: none;
  width: 100%;
  height: 100%;
}
</style>