<template>
  <div class="waypoint-item">
    <div class="identification">
      <p>{{ waypoint.name }}, ID: {{ waypoint.id }}</p>
    </div>
    <div class="buttons">
      <button class="red" @click="$emit('add', { list: list, index: index })">
        Add
      </button>
      <button
        :class="[waypoint.post ? 'green' : 'red']"
        @click="$emit('togglePost', { list: list, index: index })"
      >
        Post
      </button>
      <button
        :class="[waypoint.gate ? 'green' : 'red']"
        @click="$emit('toggleGate', { list: list, index: index })"
      >
        Gate
      </button>
      <button
        class="red"
        @click="$emit('delete', { list: list, index: index })"
      >
        Delete
      </button>
      <!-- TODO: Fix Find Button -->
      <!-- <button
        :class="[index === highlightedWaypoint ? 'green' : 'red']"
        @click="$emit('find', { list: list, index: index })"
      >
        Find
      </button> -->
    </div>
    <div class="location">
      <div>
        <p>{{ output.lat.d }}º</p>
        <p v-if="min_enabled">{{ output.lat.m }}'</p>
        <p v-if="sec_enabled">{{ output.lat.s }}"</p>
        N <b>&nbsp;|</b>
        <p>{{ output.lon.d }}º</p>
        <p v-if="min_enabled">{{ output.lon.m }}'</p>
        <p v-if="sec_enabled">{{ output.lon.s }}"</p>
        W
      </div>
    </div>
  </div>
</template>

<script>
import { mapGetters } from "vuex";
import { convertDMS } from "../utils";

export default {
  props: {
    waypoint: {
      type: Object,
      required: true,
    },

    list: {
      type: Number,
      required: true,
    },

    index: {
      type: Number,
      required: true,
    },
  },

  computed: {
    ...mapGetters("map", {
      odom_format: "odomFormat",
    }),

    ...mapGetters("autonomy", {
      highlightedWaypoint: "highlightedWaypoint",
    }),

    min_enabled: function () {
      return this.odom_format != "D";
    },

    sec_enabled: function () {
      return this.odom_format == "DMS";
    },

    output: function () {
      return {
        lat: convertDMS({ d: this.waypoint.lat, m: 0, s: 0 }, this.odom_format),
        lon: convertDMS({ d: this.waypoint.lon, m: 0, s: 0 }, this.odom_format),
      };
    },
  },
};
</script>

<style scoped>
.waypoint-item {
  background-color: rgb(180, 180, 180);
  border-radius: 5px;
  padding: 1px 10px 10px 10px;
  border: 1px solid black;

  margin: 5px;
}
.location {
  grid-area: location;
}

.location p {
  display: inline;
}

.buttons {
  grid-area: buttons;
  align-self: center;
  justify-self: center;
  display: block;
}

.red {
  background-color: red;
}

.green {
  background-color: green;
}

button {
  width: auto;
  height: auto;
  padding: 7px;
  font-weight: bold;
}

p {
  margin: 5px;
}
</style>
