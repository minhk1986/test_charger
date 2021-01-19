<template>
  <v-navigation-drawer v-model="showNavigationMenu" absolute temporary>
    <v-list>
      <v-list-item two-line>
        <v-img
          src="http://prati.com.vn/wp-content/uploads/2018/09/LogoPRATI_Home_En.png"
        />
      </v-list-item>
      <v-divider />
      <v-list-item v-for="item in nanigationMenu" :key="item.title" link :to="item.to">
        <v-list-item-icon>
          <v-icon>{{ item.icon }}</v-icon>
        </v-list-item-icon>

        <v-list-item-content>
          <v-list-item-title>{{ item.title }}</v-list-item-title>
        </v-list-item-content>
      </v-list-item>
    </v-list>
  </v-navigation-drawer>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
const { mapState, mapActions } = createNamespacedHelpers("dashboard");

export default {
  name: "NavigationMenu",
  methods: {
    ...mapActions(["toggleNavigationMenu"]),
  },
  computed: {
    ...mapState({
      isShowNavigationMenu: (state) => state.isShowNavigationMenu,
      nanigationMenu: (state) => state.navigationMenu,
    }),
    showNavigationMenu: {
      get() {
        return this.isShowNavigationMenu;
      },
      set(isShow) {
        if (isShow != this.isShowNavigationMenu)
          this.toggleNavigationMenu(isShow);
      },
    },
  },
};
</script>