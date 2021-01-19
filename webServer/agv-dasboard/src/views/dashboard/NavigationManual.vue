<template>
  <v-container fluid class="fill-height pa-0 d-flex justify-center">
    <v-row>
      <v-btn icon x-large small @click="onContextMenu">
        <v-icon large dark>mdi-dots-vertical</v-icon>
      </v-btn>
    </v-row>
    <AgvrJoyStickButton :width="400" :height="400" :handleWidth="120" :handleHeight="120" />
    <AgvrRobotNavigationMenuContext/>
  </v-container>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
import AgvrJoyStickButton from "@/components/AgvrJoyStickButton";
import { AgvrRobotNavigationMenuContext } from "@/components/MenuContexts";

const { mapActions: navigationMapAction } = createNamespacedHelpers("navigation");

export default {
  name: "NavigationManual",
  components: {
    AgvrJoyStickButton,
    AgvrRobotNavigationMenuContext,
  },
  methods: {
    ...navigationMapAction({
      navigationShowAgvMenuContext: "showAgvMenuContext",
    }),
    onContextMenu(e) {
      e.preventDefault();
      e.stopPropagation();
      this.$nextTick(() => {
        this.navigationShowAgvMenuContext({
          x: e.clientX,
          y: e.clientY,
        });
      });
    },
  },
};
</script>