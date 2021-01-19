<template>
  <v-container class="pa-0" fluid>
    <v-row no-gutters justify="center">
      <v-col class="pa-1 pt-2" xs="12" sm="4" md="3" lg="2">
        <v-text-field
          type="number"
          color="black"
          :prepend-inner-icon="'mdi-axis-x-arrow'"
          append-icon="mdi-alpha-m"
          label="X axit"
          outlined
          dense
          :step="0.001"
          v-model="positionX"
          :readonly="$router.currentRoute.path != '/navigation/init'"
          @keypress.enter="updatePosition"
          @blur="updatePosition"
          hide-details
        />
      </v-col>
      <v-col class="pa-1 pt-2" xs="12" sm="4" md="3" lg="2">
        <v-text-field
          type="number"
          color="black"
          :prepend-inner-icon="'mdi-axis-y-arrow'"
          append-icon="mdi-alpha-m"
          label="Y axit"
          outlined
          dense
          :step="0.001"
          v-model="positionY"
          :readonly="$router.currentRoute.path != '/navigation/init'"
          @keypress.enter="updatePosition"
          @blur="updatePosition"
          hide-details
        />
      </v-col>
      <v-col class="pa-1 pt-2" xs="12" sm="4" md="3" lg="2">
        <v-text-field
          type="number"
          color="black"
          :prepend-inner-icon="'mdi-axis-z-rotate-counterclockwise'"
          append-icon="mdi-alpha-d"
          label="Z angle"
          outlined
          dense
          :step="0.001"
          v-model="angleZ"
          :readonly="$router.currentRoute.path != '/navigation/init'"
          @keypress.enter="updateAngle"
          @blur="updateAngle"
          hide-details
        />
      </v-col>
      <div class="d-flex">
        <span class="align-self-center">{{agvSim1000PoseQuality}}%</span>
      </div>
    </v-row>
  </v-container>
</template>

<script>
import { createNamespacedHelpers } from "vuex";

const { mapState: navigationMapState } = createNamespacedHelpers("navigation");
const {
  mapState: agvsMapState,
  mapActions: agvsmapActions,
} = createNamespacedHelpers("agvs");
const { mapState: agvMapsMapState } = createNamespacedHelpers("agvMaps");

export default {
  name: "AgvrPoseEditor",
  data() {
    return {
      tmpPositionX: 0,
      tmpPositionY: 0,
      tmpAngleZ: 0,
    };
  },
  computed: {
    ...navigationMapState({
      navigationMapId: (state) => state.mapId,
      navigationAgvId: (state) => state.agvId,
    }),
    ...agvsMapState({
      agvWebPose(state) {
        let agv = state.find((a) => a.id === this.navigationAgvId);
        return agv != null
          ? agv.webPose
          : {
              position: { x: 0, y: 0, z: 0 },
              orientation: { x: 0, y: 0, z: 0 },
            };
      },
      agvSim1000PoseQuality(state) {
        let agv = state.find((a) => a.id === this.navigationAgvId);
        return agv != null ? agv.sim1000Pose.quality : 0;
      },
    }),
    ...agvMapsMapState({
      agvMapScale(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map != null ? map.webMap.scale : 1;
      },
      agvRosMapInfo(state) {
        let map = state.find((m) => m.id === this.navigationMapId);
        return map.rosMap.info;
      },
    }),
    positionX: {
      get() {
        return (
          this.agvRosMapInfo.origin.position.x +
          (this.agvWebPose.position.x * this.agvRosMapInfo.resolution) /
            this.agvMapScale
        ).toFixed(2);
      },
      set(val) {
        this.tmpPositionX = parseFloat(val);
      },
    },
    positionY: {
      get() {
        return (
          this.agvRosMapInfo.origin.position.y -
          (this.agvWebPose.position.y / this.agvMapScale -
            this.agvRosMapInfo.height) *
            this.agvRosMapInfo.resolution
        ).toFixed(2);
      },
      set(val) {
        this.tmpPositionY = parseFloat(val);
      },
    },
    angleZ: {
      get: function () {
        return this.agvWebPose.orientation.z.toFixed(0);
      },
      set: function (val) {
        this.tmpAngleZ = parseFloat(val);
      },
    },
  },
  methods: {
    ...agvsmapActions({
      setAgvDirection: "setAgvDirection",
      setAgvPosition: "setAgvPosition",
    }),
    updatePosition() {
      this.setAgvPosition({
        x:
          ((this.tmpPositionX - this.agvRosMapInfo.origin.position.x) *
            this.agvMapScale) /
          this.agvRosMapInfo.resolution,
        y:
          ((this.agvRosMapInfo.origin.position.y - this.tmpPositionY) /
            this.agvRosMapInfo.resolution +
            this.agvRosMapInfo.height) *
          this.agvMapScale,
        z: 0,
        agvId: this.navigationAgvId,
      });
    },
    updateAngle() {
      this.setAgvDirection({
        x: 0,
        y: 0,
        z: this.tmpAngleZ,
        agvId: this.navigationAgvId,
      });
    },
  },
  mounted() {
    this.tmpPositionX = this.positionX;
    this.tmpPositionY = this.positionY;
    this.tmpAngleZ = this.angleZ;
  },
};
</script>

<style scoped>
.pose-input {
  caret-color: black;
}
</style>