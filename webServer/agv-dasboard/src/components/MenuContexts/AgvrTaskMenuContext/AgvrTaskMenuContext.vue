<template>
  <v-menu
    v-model="isShow"
    :position-x="navigationTaskMenuContext.position.x"
    :position-y="navigationTaskMenuContext.position.y"
    absolute
    offset-y
    rounded="0"
  >
    <v-list dense>
      <v-list-item-group class="pa-0" color="primary">
        <v-list-item @click="openActionManagement">
          <v-list-item-icon>
            <v-icon>mdi-clipboard-list-outline</v-icon>
          </v-list-item-icon>
          <v-list-item-content>
            <v-list-item-title>Actions</v-list-item-title>
          </v-list-item-content>
        </v-list-item>
        <v-list-item @click="insertBefore">
          <v-list-item-icon>
            <v-icon>mdi-table-column-plus-before</v-icon>
          </v-list-item-icon>
          <v-list-item-content>
            <v-list-item-title>Add before</v-list-item-title>
          </v-list-item-content>
        </v-list-item>
        <v-list-item @click="insertAfter">
          <v-list-item-icon>
            <v-icon>mdi-table-column-plus-after</v-icon>
          </v-list-item-icon>
          <v-list-item-content>
            <v-list-item-title>Add after</v-list-item-title>
          </v-list-item-content>
        </v-list-item>
        <v-list-item @click="removeTask">
          <v-list-item-icon>
            <v-icon>mdi-delete</v-icon>
          </v-list-item-icon>
          <v-list-item-content>
            <v-list-item-title>Delete</v-list-item-title>
          </v-list-item-content>
        </v-list-item>
      </v-list-item-group>
    </v-list>
  </v-menu>
</template>
<script>
import { createNamespacedHelpers } from "vuex";

const {
  mapState: navigationMapState,
  mapActions: navigationMapActions,
} = createNamespacedHelpers("navigation");
const { mapActions: agvJobsMapActions } = createNamespacedHelpers("agvJobs");
const { mapState: agvsMapState } = createNamespacedHelpers("agvs");

export default {
  name: "AgvrTaskMenuContext",
  computed: {
    ...navigationMapState({
      navigationTaskMenuContext: (state) => state.taskMenuContext,
      navigationJobId: (state) => state.jobId,
      navigationTaskId: (state) => state.taskId,
      navigationAgvId: (state) => state.agvId,
    }),
    ...agvsMapState({
      agvPosition(state) {
        return state[this.navigationAgvId].webPose.position;
      },
    }),
    isShow: {
      get() {
        return this.navigationTaskMenuContext.isShow;
      },
      set(val) {
        if (val) {
          this.navigationShowTaskMenuContext();
        } else {
          this.navigationHideTaskMenuContext();
        }
      },
    },
  },
  methods: {
    ...agvJobsMapActions({
      insertTaskAfterToJob: "insertTaskAfterToJob",
      insertTaskBeforeToJob: "insertTaskBeforeToJob",
      removeTaskFromJob: "removeTaskFromJob",
    }),
    ...navigationMapActions({
      navigationShowTaskMenuContext: "showTaskMenuContext",
      navigationHideTaskMenuContext: "hideTaskMenuContext",
      navigationShowActionManagement: "showActionManagement",
    }),
    openActionManagement() {
      this.navigationShowActionManagement();
    },
    insertAfter() {
      this.insertTaskAfterToJob({
        jobId: this.navigationJobId,
        taskId: this.navigationTaskId,
      });
    },
    insertBefore() {
      this.insertTaskBeforeToJob({
        jobId: this.navigationJobId,
        taskId: this.navigationTaskId,
        agvPosition: this.agvPosition,
      });
    },
    removeTask(){
      this.removeTaskFromJob({
        jobId: this.navigationJobId,
        taskId: this.navigationTaskId,
      })
    }
  },
};
</script>