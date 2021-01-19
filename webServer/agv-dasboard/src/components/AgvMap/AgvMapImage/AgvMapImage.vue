<template>
  <foreignObject
    class="panzoom-map-image"
    :width="navigationSvgMap.width"
    :height="navigationSvgMap.height"
  >
    <canvas ref="mapImageRef" />
  </foreignObject>
</template>

<script>
import { mapState } from "vuex";
export default {
  name: "AgvMapImage",
  data: () => ({
    mapCanvas: null,
  }),
  computed: {
    ...mapState({
      navigationMap: (state) =>
        state.agvMaps.find(map => map.id === state.navigation.selectedMapId).rosMap,
      navigationSvgMap: (state) =>
        state.agvMaps.find(map => map.id === state.navigation.selectedMapId).webMap,
    }),
  },
  mounted() {
    this.mapCanvas = this.$refs.mapImageRef;
    if (this.navigationMap.data.length > 0) {
      this.setMapData();
    }
  },
  methods: {
    setMapData() {
      this.mapCanvas.width = this.navigationSvgMap.width;
      this.mapCanvas.height = this.navigationSvgMap.height;
      let ctx = this.mapCanvas.getContext("2d");
      let imageData = ctx.createImageData(
        this.navigationSvgMap.ImageData.width,
        this.navigationSvgMap.ImageData.height
      );
      imageData.data.set(this.navigationSvgMap.ImageData.data);
      ctx.putImageData(imageData, 0, 0);
    },
  },
  watch: {
    navigationMap() {
      this.setMapData();
    },
  },
};
</script>
<style scoped>
.panzoom-map-image {
  z-index: -1;
  position: relative;
}
</style>