<template>
  <v-list-item>
    <v-list-item-content>
      <v-text-field
        :type="inputType"
        :label="text"
        outlined
        v-model="valueInput"
        :rules="inputRules"
        :suffix="unit"
        :step="step"
      />
    </v-list-item-content>
  </v-list-item>
</template>

<script>
import { mapState, mapActions } from "vuex";
import StoreProcess from "@/utils/store-process";

export default {
  name: "setting-component",
  props: {
    pathStore: {
      type: Array,
      default: () => [],
    },
    text: {
      type: String,
      default: "",
    },
    type: {
      type: String,
      default: "",
    },
    unit: {
      type: String,
      default: "",
    },
    step: {
      type: Number,
      default: 0.1,
    },
  },
  computed: {
    ...mapState({
      value(state) {
        return StoreProcess.getDeepProperty(state, this.pathStore);
      },
    }),
    valueInput: {
      get() {
        return this.value;
      },
      set(val) {
        this.setProperty({
          value: this.getValue(val) || 0,
          properties: this.pathStore,
        });
      },
    },
    inputType() {
      switch (this.type) {
        case "double":
        case "integer":
          return "number";
        default:
          return "text";
      }
    },
    inputRules() {
      let rules = [(value) => !!value || value == 0 || "Required.",];
      return rules;
    },
  },
  methods: {
    ...mapActions(["setProperty"]),
    getValue(val) {
      switch (this.type) {
        case "double":
          return parseFloat(val);
        case "integer":
          return parseInt(val);
        default:
          return val;
      }
    },
  },
};
</script>