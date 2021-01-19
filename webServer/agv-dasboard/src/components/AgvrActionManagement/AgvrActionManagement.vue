<template>
  <v-navigation-drawer
    v-model="isShow"
    absolute
    temporary
    right
    width="500"
    :style="{ boxShadow: 'none' }"
  >
    <v-container fluid class="pa-0">
      <v-row>
        <v-col class="d-flex flex-column px-6 pb-0">
          <v-text-field
            dense
            hide-details
            class="mb-6"
            type="number"
            label="Position X"
            outlined
            :step="0.001"
            :prepend-inner-icon="'mdi-axis-x-arrow'"
            append-icon="mdi-alpha-m"
            v-model="positionX"
          />
          <v-text-field
            dense
            hide-details
            class="mb-6"
            type="number"
            label="Position Y"
            outlined
            :step="0.001"
            :prepend-inner-icon="'mdi-axis-y-arrow'"
            append-icon="mdi-alpha-m"
            v-model="positionY"
          />
          <v-text-field
            dense
            hide-details
            class="mb-2"
            type="number"
            label="Angle"
            outlined
            :step="0.001"
            :prepend-inner-icon="'mdi-axis-z-rotate-counterclockwise'"
            append-icon="mdi-alpha-d"
            v-model="thetaEnd"
          />
        </v-col>
      </v-row>
      <v-row class="flex-shrink-1">
        <v-col class="pt-0">
          <v-stepper class="action-management-stepper" v-model="step" vertical>
            <template v-for="(stepItem, index) in actionCollection">
              <div :key="`step-${stepItem.step}`" @contextmenu="(e) => openMenuContext(e, index)">
                <v-stepper-step
                  :step="stepItem.step"
                  :edit-icon="'mdi-puzzle-edit-outline'"
                  complete
                  editable
                >{{ stepItem.name }}</v-stepper-step>
              </div>

              <v-stepper-content :key="`stepper-content-${stepItem.step}`" :step="stepItem.step">
                <v-container fluid class="pa-0">
                  <v-text-field
                    class="mt-2"
                    label="Step name"
                    v-model="stepItem.name"
                    append-icon="mdi-square-edit-outline"
                    outlined
                    dense
                  />
                  <v-combobox
                    label="Action"
                    v-model="stepItem.action"
                    :items="actionLists"
                    dense
                    hide-details
                    outlined
                  >
                    <template v-slot:selection="{ item }">
                      <span class="pr-2">{{ actionTextLists[item] }}</span>
                    </template>
                    <template v-slot:item="{ item }">
                      <span class="pr-2">{{ actionTextLists[item] }}</span>
                    </template>
                  </v-combobox>
                </v-container>
              </v-stepper-content>
            </template>
            <div @contextmenu="(e) => e.preventDefault()">
              <v-stepper-step
                :step="-1"
                edit-icon="mdi-plus"
                complete
                editable
                @click="addLastStep"
              />
            </div>
          </v-stepper>
        </v-col>
      </v-row>
      <v-row>
        <v-spacer />
        <v-col cols="auto" class="px-6">
          <v-btn depressed color="primary" v-text="'Save'" @click="onSave" />
        </v-col>
      </v-row>
    </v-container>

    <v-menu v-model="isShowMenu" :position-x="menuX" :position-y="menuY" absolute offset-y>
      <v-list>
        <v-list-item @click="addAboveStep">
          <v-list-item-icon>
            <v-icon>mdi-plus</v-icon>
          </v-list-item-icon>
          <v-list-item-title v-text="'Above'" />
        </v-list-item>
        <v-list-item @click="addBelowStep">
          <v-list-item-icon>
            <v-icon>mdi-plus</v-icon>
          </v-list-item-icon>
          <v-list-item-title v-text="'Below'" />
        </v-list-item>
        <v-list-item @click="deleteStep">
          <v-list-item-icon>
            <v-icon>mdi-delete</v-icon>
          </v-list-item-icon>
          <v-list-item-title v-text="'Delete'" />
        </v-list-item>
      </v-list>
    </v-menu>
  </v-navigation-drawer>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
import { uuid } from "vue-uuid";
import * as AgvActionType from "@/services/agvr/agv-action-type";

const {
  mapState: agvJobsMapState,
  mapActions: agvJobsMapActions,
} = createNamespacedHelpers("agvJobs");
const {
  mapState: navigationMapState,
  mapActions: navigationMapActions,
} = createNamespacedHelpers("navigation");
const { mapState: agvMapsMapState } = createNamespacedHelpers("agvMaps");

export default {
  name: "AgvrActionManagement",
  props: {
    jobId: {
      type: String,
      default: "",
    },
    taskId: {
      type: String,
      default: "",
    },
  },
  data() {
    return {
      step: -1,
      actionLists: [
        AgvActionType.FLOAT,
        AgvActionType.CHARGING_IN,
        AgvActionType.CHARGING_OUT,
        AgvActionType.LIFT_IN,
        AgvActionType.LIFT_UP,
        AgvActionType.LIFT_DOWN,
        AgvActionType.LIFT_OUT,
      ],
      actionTextLists: {
        [AgvActionType.FLOAT]: "Do nothing",
        [AgvActionType.CHARGING_IN]: "Charging start",
        [AgvActionType.CHARGING_OUT]: "Charging end",
        [AgvActionType.LIFT_IN]: "Lift in",
        [AgvActionType.LIFT_UP]: "Lift up",
        [AgvActionType.LIFT_DOWN]: "Lift down",
        [AgvActionType.LIFT_OUT]: "Lift out",
      },
      tmpPositionX: 0,
      tmpPositionY: 0,
      tmpAngleZ: 0,
      actionCollection: [],
      isShowMenu: false,
      menuX: 0,
      menuY: 0,
      selectedStep: -1,
    };
  },
  methods: {
    ...navigationMapActions({
      navigationShowActionManagement: "showActionManagement",
      navigationHideActionManagement: "hideActionManagement",
    }),
    ...agvJobsMapActions({
      updateActions: "updateActions",
    }),
    openMenuContext(e, step) {
      this.selectedStep = step;
      e.preventDefault();
      this.isShowMenu = false;
      this.menuX = e.clientX;
      this.menuY = e.clientY;
      this.$nextTick(() => {
        this.isShowMenu = true;
      });
    },
    addAboveStep() {
      if (this.selectedStep < 0) return;
      this.actionCollection.splice(this.selectedStep, 0, {
        step: uuid.v4(),
        name: "New step",
        action: this.actionLists[0],
      });
    },
    addBelowStep() {
      if (this.selectedStep < 0) return;
      this.actionCollection.splice(this.selectedStep + 1, 0, {
        step: uuid.v4(),
        name: "New step",
        action: this.actionLists[0],
      });
    },
    deleteStep() {
      if (this.selectedStep < 0) return;
      this.actionCollection.splice(this.selectedStep, 1);
      this.step = -1;
    },
    addLastStep() {
      this.actionCollection.push({
        step: uuid.v4(),
        name: "New step",
        action: this.actionLists[0],
      });
    },
    onSave() {
      this.updateActions({
        jobId: this.jobId,
        taskId: this.taskId,
        actions: this.actionCollection,
        position: {
          x:
            ((parseFloat(this.tmpPositionX) -
              this.agvRosMapInfo.origin.position.x) *
              this.agvMapScale) /
            this.agvRosMapInfo.resolution,
          y:
            ((this.agvRosMapInfo.origin.position.y -
              parseFloat(this.tmpPositionY)) /
              this.agvRosMapInfo.resolution +
              this.agvRosMapInfo.height) *
            this.agvMapScale,
          z: 0,
        },
        thetaEnd: parseFloat(this.tmpAngleZ),
      });
    },
  },
  computed: {
    ...navigationMapState({
      navigationActionManagementShow: (state) => state.actionManagement.isShow,
      navigationMapId: (state) => state.mapId,
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
    ...agvJobsMapState({
      agvSelectedTask(state) {
        let job = state.find((j) => j.id === this.jobId);
        if (job == null) return null;
        return job.tasks.find((t) => t.id === this.taskId);
      },
    }),
    isShow: {
      get() {
        return this.navigationActionManagementShow;
      },
      set(val) {
        if (val) {
          this.navigationShowActionManagement();
        } else {
          this.navigationHideActionManagement();
        }
      },
    },
    positionX: {
      get() {
        if (
          this.agvSelectedTask == null ||
          this.agvRosMapInfo == null ||
          this.agvMapScale == null
        )
          return 0;
        return (
          this.agvRosMapInfo.origin.position.x +
          (this.agvSelectedTask.position.x * this.agvRosMapInfo.resolution) /
            this.agvMapScale
        ).toFixed(2);
      },
      set(val) {
        this.tmpPositionX = val;
      },
    },
    positionY: {
      get() {
        if (
          this.agvSelectedTask == null ||
          this.agvRosMapInfo == null ||
          this.agvMapScale == null
        )
          return 0;

        
        return (
          this.agvRosMapInfo.origin.position.y -
          (this.agvSelectedTask.position.y / this.agvMapScale -
            this.agvRosMapInfo.height) *
            this.agvRosMapInfo.resolution
        ).toFixed(2);
      },
      set(val) {
        this.tmpPositionY = val;
      },
    },
    thetaEnd: {
      get: function () {
        if (this.agvSelectedTask == null) return 0;
        return this.agvSelectedTask.thetaEnd.toFixed(0);
      },
      set: function (val) {
        this.tmpAngleZ = val;
      },
    },
  },
  watch: {
    isShowMenu(val) {
      if (val == false) this.selectedStep = -1;
    },
    isShow(val) {
      if (val) {
        this.tmpPositionX = this.positionX;
        this.tmpPositionY = this.positionY;
        this.tmpAngleZ = this.thetaEnd;
        this.actionCollection = this.agvSelectedTask != null ? this.agvSelectedTask.actions : [];
      }
    },
  },
  mounted() {
    this.tmpPositionX = this.positionX;
    this.tmpPositionY = this.positionY;
    this.tmpAngleZ = this.thetaEnd;
    this.actionCollection = this.agvSelectedTask != null ? this.agvSelectedTask.actions : [];
  },
};
</script>

<style scoped>
.action-management-stepper {
  box-shadow: none;
}
</style>
