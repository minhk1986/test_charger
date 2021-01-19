<template>
  <g>
    <image 
      v-for="point in agvMapSmartPoints"
      :key="`agv-map-smart-point-${point.id}`"
      :x="point.x - 15"
      :y="point.y - 15"
      :width="30"
      :height="30"
      href="@/assets/images/add.svg" />
  </g>
</template>

<script>
import { createNamespacedHelpers } from "vuex";

const { mapState: agvMapsState } = createNamespacedHelpers("agvMaps");

export default {
  name: "AgvrMapSmartPoint",
  props: {
    mapId: {
      type: String,
      default: "",
    },
  },
  computed: {
    ...agvMapsState({
      agvMapSmartPoints(state) {
        let agvMap = state.find((m) => m.id === this.mapId);

        return (agvMap != null) ? agvMap.smartWebPoints : [];
      },
    }),
  },
}
</script>