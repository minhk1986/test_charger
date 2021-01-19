<template>
  <v-menu
    v-model="isShow"
    :position-x="navigationAgvMenuContext.position.x"
    :position-y="navigationAgvMenuContext.position.y"
    absolute
    offset-y
    rounded="0"
  >
    <v-list dense>
      <v-list-item-group class="pa-0" color="primary">
        <v-list-item @click="onCreateGoal">
          <v-list-item-icon>
            <v-icon v-text="'mdi-map-marker-plus'" />
          </v-list-item-icon>
          <v-list-item-content>
            <v-list-item-title v-text="'Create goal'" />
          </v-list-item-content>
        </v-list-item>
        <v-list-item @click="onRun">
          <v-list-item-icon>
            <v-icon v-text="'mdi-play'" />
          </v-list-item-icon>
          <v-list-item-content>
            <v-list-item-title v-text="'Run'" />
          </v-list-item-content>
        </v-list-item>
        <v-menu bottom :offset-x="true">
          <template v-slot:activator="{ on, attrs }">
            <v-list-item v-bind="attrs" v-on="on">
              <v-list-item-icon>
                <v-icon v-text="'mdi-function-variant'" />
              </v-list-item-icon>
              <v-list-item-content>
                <v-list-item-title v-text="'Actions'" />
              </v-list-item-content>
              <v-list-item-action>
                <v-icon v-text="'mdi-chevron-right'" />
              </v-list-item-action>
            </v-list-item>
          </template>
          <v-list>
            <v-list-item @click="onLiftIn">
              <v-list-item-icon>
                <v-icon v-text="'mdi-arrow-expand-right'" />
              </v-list-item-icon>
              <v-list-item-content>
                <v-list-item-title v-text="'Lift In'" />
              </v-list-item-content>
            </v-list-item>
            <v-list-item @click="onLiftUp">
              <v-list-item-icon>
                <v-icon v-text="'mdi-arrow-expand-up'" />
              </v-list-item-icon>
              <v-list-item-content>
                <v-list-item-title v-text="'Lift Up'" />
              </v-list-item-content>
            </v-list-item>
            <v-list-item @click="onLiftDown">
              <v-list-item-icon>
                <v-icon v-text="'mdi-arrow-expand-down'" />
              </v-list-item-icon>
              <v-list-item-content>
                <v-list-item-title v-text="'Lift Down'" />
              </v-list-item-content>
            </v-list-item>
            <v-list-item @click="onLiftOut">
              <v-list-item-icon>
                <v-icon v-text="'mdi-arrow-expand-left'" />
              </v-list-item-icon>
              <v-list-item-content>
                <v-list-item-title v-text="'Lift Out'" />
              </v-list-item-content>
            </v-list-item>
            <v-list-item @click="onChargingStart">
              <v-list-item-icon>
                <v-icon v-text="'mdi-power-plug'" />
              </v-list-item-icon>
              <v-list-item-content>
                <v-list-item-title v-text="'Charging In'" />
              </v-list-item-content>
            </v-list-item>
            <v-list-item @click="onChargingEnd">
              <v-list-item-icon>
                <v-icon v-text="'mdi-power-plug-off'" />
              </v-list-item-icon>
              <v-list-item-content>
                <v-list-item-title v-text="'Charging Out'" />
              </v-list-item-content>
            </v-list-item>
          </v-list>
        </v-menu>
        <v-list-item @click="onLogTask">
          <v-list-item-icon>
            <v-icon v-text="'mdi-math-log'" />
          </v-list-item-icon>
          <v-list-item-content>
            <v-list-item-title v-text="'Log Tasks'" />
          </v-list-item-content>
        </v-list-item>
        <v-menu bottom :offset-x="true">
          <template v-slot:activator="{ on, attrs }">
            <v-list-item v-bind="attrs" v-on="on">
              <v-list-item-icon>
                <v-icon v-text="'mdi-function-variant'" />
              </v-list-item-icon>
              <v-list-item-content>
                <v-list-item-title v-text="'Missions'" />
              </v-list-item-content>
              <v-list-item-action>
                <v-icon v-text="'mdi-chevron-right'" />
              </v-list-item-action>
            </v-list-item>
          </template>
          <v-list>
            <v-list-item v-for="(mission, index) in agvMapMissions" :key="`agv-map-mission-${index}`" @click="onSetMission(mission.tasks)">
              <v-list-item-content>
                <v-list-item-title v-text="mission.name" />
              </v-list-item-content>
            </v-list-item>
          </v-list>
        </v-menu>
      </v-list-item-group>
    </v-list>
  </v-menu>
</template>

<script>
import { createNamespacedHelpers } from "vuex";

const {
  mapState: navigationMapState,
  mapActions: navigationMapAction
} = createNamespacedHelpers("navigation");

const { mapState: agvsMapState } = createNamespacedHelpers("agvs");
const { mapState: agvMapsMapState } = createNamespacedHelpers("agvMaps");
const {
  mapState: agvJobsMapState,
  mapActions: agvJobsMapActions,
  mapGetters: agvJobsMapGetters
} = createNamespacedHelpers("agvJobs");

export default {
  name: "AgvrRobotNavigationMenuContext",
  computed: {
    ...navigationMapState({
      navigationAgvMenuContext: state => state.agvMenuContext,
      navigationAgvId: state => state.agvId,
      navigationJobId: state => state.jobId,
      navigationMapId: state => state.mapId
    }),
    ...agvJobsMapState({
      getListTask(state) {
        let job = state.find(j => j.id === this.navigationJobId);
        return job?.tasks ?? [];
      }
    }),
    ...agvMapsMapState({
      agvMapMissions(state) {
        let map = state.find(m => m.id === this.navigationMapId);
        return map?.missions ?? [];
      }
    }),
    ...agvJobsMapGetters({
      getLastTask: "getLastTask"
    }),
    ...agvsMapState({
      agvJobId(state) {
        let agv = state.find(a => a.id === this.navigationAgvId);
        return agv != null ? agv.jobId : null;
      },
      agvWebPose(state) {
        let agv = state.find(a => a.id === this.navigationAgvId);
        return agv != null ? agv.webPose : null;
      }
    }),
    isShow: {
      get() {
        return this.navigationAgvMenuContext.isShow;
      },
      set(val) {
        if (val) {
          this.navigationShowAgvMenuContext();
        } else {
          this.navigationHideAgvMenuContext();
        }
      }
    }
  },
  methods: {
    ...navigationMapAction({
      navigationHideAgvMenuContext: "hideAgvMenuContext",
      navigationShowAgvMenuContext: "showAgvMenuContext"
    }),
    ...agvJobsMapActions({
      pushTaskToJob: "pushTaskToJob",
      setTasksToJob: "setTasks",
    }),
    onCreateGoal() {
      let point = null;
      let task = this.getLastTask(this.agvJobId);
      if (task != null) {
        point = {
          x: task.position.x + Math.cos((task.thetaEnd * Math.PI) / 180) * 100,
          y: task.position.y - Math.sin((task.thetaEnd * Math.PI) / 180) * 100
        };
      } else {
        point = {
          x:
            this.agvWebPose.position.x +
            Math.cos((this.agvWebPose.orientation.z * Math.PI) / 180) * 100,
          y:
            this.agvWebPose.position.y -
            Math.sin((this.agvWebPose.orientation.z * Math.PI) / 180) * 100
        };
      }
      if (point != null) {
        this.pushTaskToJob({
          jobId: this.agvJobId,
          x: point.x,
          y: point.y
        });
      }
    },
    onRun() {
      this.$agvr.robots[this.navigationAgvId].runJob();
      this.isShow = false;
    },
    onLiftIn() {
      this.$agvr.robots[this.navigationAgvId].liftIn();
      this.isShow = false;
    },
    onLiftUp() {
      this.$agvr.robots[this.navigationAgvId].liftUp();
      this.isShow = false;
    },
    onLiftDown() {
      this.$agvr.robots[this.navigationAgvId].liftDown();
      this.isShow = false;
    },
    onLiftOut() {
      this.$agvr.robots[this.navigationAgvId].liftOut();
      this.isShow = false;
    },
    onChargingStart() {
      this.$agvr.robots[this.navigationAgvId].chargingStart();
      this.isShow = false;
    },
    onChargingEnd() {
      this.$agvr.robots[this.navigationAgvId].chargingEnd();
      this.isShow = false;
    },
    onLogTask() {
      console.log(JSON.stringify(this.getListTask));
    },
    onSetMission(tasks) {
      this.setTasksToJob({
        jobId: this.agvJobId,
        tasks: tasks,
      });
      this.isShow = false;
    }
  }
};
</script>