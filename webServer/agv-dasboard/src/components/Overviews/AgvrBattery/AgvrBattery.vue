<template>
  <v-card class="mt-4" elevation="8">
    <v-card-title>
      <v-card class="card-title-card" color="green darken-2" elevation="8" >
        <v-card-text>
          <v-icon x-large color="white" v-text="batteryIcon"/>
        </v-card-text>
      </v-card>
      <v-spacer/>
      {{batteryLevel}}%
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
      batteryLevel: 75,
      batteryIcon: "mdi-battery-alert-variant-outline",
      timeUpdated: new Date(),
      timerUpdate: null,
    };
  },
  methods: {
    getBatteryIcon() {
      if(this.batteryLevel < 5) return "mdi-battery-alert-variant-outline";
      if(this.batteryLevel < 15) return "mdi-battery-10";
      if(this.batteryLevel < 25) return "mdi-battery-20";
      if(this.batteryLevel < 35) return "mdi-battery-30";
      if(this.batteryLevel < 45) return "mdi-battery-40";
      if(this.batteryLevel < 55) return "mdi-battery-50";
      if(this.batteryLevel < 65) return "mdi-battery-60";
      if(this.batteryLevel < 75) return "mdi-battery-70";
      if(this.batteryLevel < 85) return "mdi-battery-80";
      if(this.batteryLevel < 95) return "mdi-battery-90";
      return "mdi-battery";
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