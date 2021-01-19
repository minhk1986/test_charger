<template>
  <foreignObject class="panzoom-map-image" :width="width" :height="height">
    <canvas ref="mapImageRef" />
  </foreignObject>
</template>

<script>
import { createNamespacedHelpers } from "vuex";

const { mapState: agvMapsState } = createNamespacedHelpers("agvMaps");

export default {
  name: "AgvrMapImage",
  props: {
    mapId: {
      type: String,
      default: "",
    },
  },
  data: () => ({
    mapCanvas: null,
    width: 0,
    height: 0,
  }),
  computed: {
    ...agvMapsState({
      agvMapImage(state) {
        let agvMap = state.find((m) => m.id === this.mapId);
        return agvMap != null ? agvMap.webMap.Image : null;
      },
    }),
  },
  mounted() {
    this.mapCanvas = this.$refs.mapImageRef;
    this.setMapImage();
  },
  methods: {
    setMapImage() {
      let mapImage = this.agvMapImage;
      if (mapImage == null) return;
      this.width = mapImage.width;
      this.height = mapImage.height;
      this.mapCanvas.width = mapImage.width;
      this.mapCanvas.height = mapImage.height;
      let ctx = this.mapCanvas.getContext("2d");
      let imageData = ctx.createImageData(mapImage.width, mapImage.height);
      imageData.data.set(mapImage.data);
      ctx.putImageData(imageData, 0, 0);
    },
  },
  watch: {
    agvMapImage() {
      this.setMapImage();
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