<template>
  <v-container fluid>
    <v-card elevation="6">
      <v-card-title>Status devices</v-card-title>
      <v-data-table
        :headers="headers"
        :items="statusDevices"
        :items-per-page="5"
        :search="search"
        loading-text="Loading... Please wait"
        :loading="isLoading"
      >
        <template v-slot:top>
          <v-row no-gutters>
            <v-spacer />
            <v-col xs="12" sm="8" md="6" lg="4">
              <v-text-field v-model="search" label="Search" class="mx-4" />
            </v-col>
          </v-row>
        </template>
        <template v-slot:[`item.name`]="{ item }">
          <v-icon
            v-text="getIconLevel(item.level)"
            :color="getColorLevel(item.level)"
          ></v-icon>
          <span>{{ item.name }}</span>
        </template>
        <template v-slot:[`item.actions`]="{ item }">
          <v-icon
            @click="() => viewValues(item)"
            :color="getColorLevel(item.level)"
          >
            mdi-view-list-outline
          </v-icon>
        </template>
      </v-data-table>
    </v-card>
    <v-dialog v-model="dialogShow" max-width="800">
      <v-card>
        <v-card-title
          :class="`headline white--text ${getColorLevel(dialogItem.level)}`"
        >
          {{ dialogItem.name }}
        </v-card-title>
        <v-card-subtitle
          :class="`py-2 white--text ${getColorLevel(dialogItem.level)}`"
          >{{ dialogItem.hardware_id }}</v-card-subtitle
        >
        <v-card-text>{{ dialogItem.message }}</v-card-text>
        <v-simple-table class="mt-2" height="400px">
          <tbody>
            <tr v-for="item in dialogItem.values" :key="item.name">
              <td>{{ item.key }}</td>
              <td>{{ item.value }}</td>
            </tr>
          </tbody>
        </v-simple-table>
      </v-card>
    </v-dialog>
  </v-container>
</template>
<script>
import { createNamespacedHelpers } from "vuex";

const { mapState: navigationMapState, } = createNamespacedHelpers("navigation");
const { mapState: agvsMapState } = createNamespacedHelpers("agvs");

export default {
  name: "AgvrStatusDevices",
  data() {
    return {
      search: "",
      isLoading: true,
      headers: [
        { text: "Componet", value: "name" },
        { text: "Hardware ID", value: "hardware_id" },
        { text: "Message", value: "message" },
        { text: "", value: "actions", sortable: false, width: 50 },
      ],
      dialogShow: false,
      dialogItem: {
        level: 0,
        name: "",
        message: "",
        hardware_id: "",
        values: [],
      },
    };
  },
  computed: {
    ...navigationMapState({
      navigationAgvId: (state) => state.agvId,
    }),
    ...agvsMapState({
      statusDevices(state) {
        let agv = state.find(a => a.id === this.navigationAgvId);
        return agv != null ? agv.statusDevices : [];
      }
    }),
  },
  mounted() {
    if (this.statusDevices != null && this.statusDevices.length > 0) {
      this.isLoading = false;
    }
  },
  methods: {
    viewValues(item) {
      this.dialogItem = item;
      this.dialogShow = true;
    },
    getColorLevel(level) {
      switch (level) {
        case 0:
          return "green darken-1";
        case 1:
          return "amber darken-1";
        case 2:
          return "red darken-1";
        case 3:
          return "blue-grey darken-1";
        default:
          return "black";
      }
    },
    getIconLevel(level) {
      switch (level) {
        case 0:
          return "mdi-check-circle-outline";
        case 1:
          return "mdi-alert";
        case 2:
          return "mdi-minus-circle";
        case 3:
          return "mdi-close-circle";
        default:
          return "";
      }
    },
  },
  watch: {
    statusDevices(val) {
      if (val != null && val.length > 0) {
        this.isLoading = false;
      }
    },
    dialogShow(val) {
      if (val === false) {
        this.dialogItem = {
          level: 0,
          name: "",
          message: "",
          hardware_id: "",
          values: [],
        };
      }
    },
  },
};
</script>