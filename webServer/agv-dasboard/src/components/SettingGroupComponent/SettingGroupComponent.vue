<template>
  <v-list-group :value="false" no-action sub-group>
    <template v-slot:activator>
      <v-list-item-content>
        <v-list-item-title>{{ text }}</v-list-item-title>
      </v-list-item-content>
    </template>
    <template v-for="(setting, index) in settings">
      <SettingGroupComponent
        v-if="setting.type === 'group'"
        :key="`setting-group-${text}-${index}`"
        :pathStore="[...pathStore,...setting.pathStore]"
        :text="setting.text"
        :settings="setting.settings"
      />
      <SettingComponent
        v-else
        :pathStore="[...pathStore,...setting.pathStore]"
        :text="setting.text"
        :type="setting.type"
        :unit="setting.unit"
        :step="setting.step"
        :key="`setting-component-${text}-${index}`"
      />
    </template>
  </v-list-group>
</template>

<script>
// import { mapState } from "vuex";
import SettingComponent from "@/components/SettingComponent";
export default {
  name: "SettingGroupComponent",
  components: {
    SettingComponent,
  },
  props: {
    pathStore: {
      type: Array,
      default: () => [],
    },
    text: {
      type: String,
      default: "",
    },
    settings: {
      type: Array,
      default: () => [],
    },
  },
};
</script>