<template>
  <v-container fluid>
    <v-row>
      <v-col xs="12" sm="8" md="6" lg="4">
        <v-list>
          <template v-for="(setting, index) in settingMeta">
            <SettingGroupComponent
              v-if="setting.type === 'group'"
              :key="`setting-group-${setting.text}-${index}`"
              :pathStore="[...setting.pathStore]"
              :text="setting.text"
              :settings="setting.settings"
            />
            <SettingComponent
              v-else
              :pathStore="[...setting.pathStore]"
              :text="setting.text"
              :type="setting.type"
              :unit="setting.unit"
              :step="setting.step"
              :key="`setting-component-${setting.text}-${index}`"
            />
          </template>
        </v-list>
      </v-col>
    </v-row>
  </v-container>
</template>

<script>
import { createNamespacedHelpers } from "vuex";
import SettingComponent from "@/components/SettingComponent";
import SettingGroupComponent from "@/components/SettingGroupComponent";
const { mapState } = createNamespacedHelpers("settings");
export default {
  name: "Setting",
  components: {
    SettingComponent,
    SettingGroupComponent,
  },
  computed: {
    ...mapState({
      settingMeta: (state) => state.settings,
    }),
  },
};
</script>
