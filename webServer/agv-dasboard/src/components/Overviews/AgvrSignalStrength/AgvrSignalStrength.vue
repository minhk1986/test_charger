<template>
  <v-card class="mt-4" elevation="8">
    <v-card-title>
      <v-card class="card-title-card" color="green darken-2" elevation="8" >
        <v-card-text>
          <v-icon x-large color="white" v-text="batteryIcon"/>
        </v-card-text>
      </v-card>
      <v-spacer/>
      -{{batteryLevel}} dbm
    </v-card-title>
    <v-divider/>
    <v-card-text class="pa-2">
      Updated {{timeUpdated.toUTCString()}}
    </v-card-text>
  </v-card>
</template>
<script>
export default {
  name: "AgvrBattery",
  data() {
    return {
      batteryLevel: 40,
      batteryIcon: "mdi-signal-cellular-outline",
      timeUpdated: new Date(),
      timerUpdate: null,
    };
  },
  methods: {
    getBatteryIcon() {
      if(this.batteryLevel < 5) return "mdi-signal-cellular-outline";
      if(this.batteryLevel < 15) return "mdi-signal-cellular-1";
      if(this.batteryLevel < 25) return "mdi-signal-cellular-2";
      return "mdi-signal-cellular-3";
    }
  },
  mounted(){
    this.batteryIcon = this.getBatteryIcon();
    this.timerUpdate = setInterval(() => {
      this.timeUpdated = new Date();
    }, 5000);
  },
  beforeDestroy(){
    clearInterval(this.timerUpdate);
  },
}
</script>

<style scoped>
.card-title-card{
  margin-top: -40px;
}
</style>