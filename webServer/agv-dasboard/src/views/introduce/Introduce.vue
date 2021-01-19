<template>
  <v-container class="fill-height pa-0" fluid ref="containerCarousel">
    <v-carousel v-model="imageModel" :height="`${carouselheight}px`">
      <v-carousel-item
        v-for="(image, index) in images"
        :key="index"
        :src="image.src"
        reverse-transition="fade-transition"
        transition="fade-transition"
        class="carousel-item-image"
      >
      </v-carousel-item>
    </v-carousel>
  </v-container>
</template>

<script>
export default {
  name: "IntroduceView",
  data() {
    return {
      imageModel: 0,
      carouselheight: 0,
      images: [
        {
          src: require("@/assets/images/introduces/1.jpg"),
        },
        {
          src: require("@/assets/images/introduces/2.jpg"),
        },
        {
          src: require("@/assets/images/introduces/3.jpg"),
        },
        {
          src: require("@/assets/images/introduces/4.jpg"),
        },
        {
          src: require("@/assets/images/introduces/5.jpg"),
        },
        {
          src: require("@/assets/images/introduces/6.jpg"),
        },
      ],
    };
  },
  mounted() {
    this.carouselheight = this.$refs.containerCarousel.offsetHeight;
    this.$refs.containerCarousel.addEventListener("resize", (e) => {
      console.log(e);
    });
  },
};
</script>

<style>
.carousel-item-image .v-image__image--cover {
  background-size: auto 100%;
}
</style>
