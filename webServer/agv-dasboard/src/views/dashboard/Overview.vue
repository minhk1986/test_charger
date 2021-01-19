<template>
  <v-container fluid>
    <v-container fluid>
      <v-row>
        <v-col cols="12" xs="12" sm="12" md="6" lg="3" xl="3">
          <AgvrBattery />
        </v-col>
        <v-col cols="12" xs="12" sm="12" md="6" lg="3" xl="3">
          <AgvrSignalStrength />
        </v-col>
      </v-row>
    </v-container>

    <v-row>
      <v-col>
        <AgvrStatusDevices />
      </v-col>
    </v-row>
  </v-container>
</template>

<script>
import {
  AgvrStatusDevices,
  AgvrBattery,
  AgvrSignalStrength,
} from "@/components/Overviews";

export default {
  name: "DashboardOverview",
  components: {
    AgvrStatusDevices,
    AgvrBattery,
    AgvrSignalStrength,
  },
};
</script>
