<template>
  <v-container class="fill-height" fluid>
    <v-row align="center" justify="center">
      <v-col cols="12" sm="8" md="4">
        <v-card class="elevation-12">
          <v-toolbar color="primary" dark flat>
            <v-toolbar-title>Login form</v-toolbar-title>
            <v-spacer></v-spacer>
          </v-toolbar>
          <v-card-text>
            <v-form>
              <v-text-field
                label="Login"
                name="login"
                prepend-icon="mdi-account"
                type="text"
                v-model="user"
                @keypress.enter="login"
              ></v-text-field>
              <v-text-field
                id="password"
                label="Password"
                name="password"
                prepend-icon="mdi-lock"
                type="password"
                v-model="password"
                @keypress.enter="login"
              ></v-text-field>
            </v-form>
          </v-card-text>
          <v-card-subtitle class="red--text" v-text="message"/>
          <v-card-actions>
            <v-spacer></v-spacer>
            <v-btn color="primary" @click="login">Login</v-btn>
          </v-card-actions>
        </v-card>
      </v-col>
    </v-row>
  </v-container>
</template>

<script>
import { createNamespacedHelpers } from "vuex";

const { mapActions } = createNamespacedHelpers("account");
export default {
  name: "LoginView",
  data() {
    return {
      user: "",
      password: "",
      message: "",
    };
  },
  methods: {
    ...mapActions(["setToken"]),
    login() {
      if (this.user === "robotics" && this.password === "robotics2020") {
        this.setToken("robotics");
        this.$router.push("/");
      }
      else{
        this.password = "";
        this.message = "User or password not match";
      }
    },
  },
};
</script>
