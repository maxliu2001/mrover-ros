<template>
  <div class="waypoint-item">
    <div class="name">
      <p>{{ waypoint.name }}</p>
    </div>
    <div class="location">
      <p>{{ waypoint.lat }}ºN, {{ waypoint.lon }}ºE</p>
    </div>
    <div class="buttons">
      <button class="red" @click="$emit('delete', { index: index })">X</button>
      <button
        :class="[index === highlightedWaypoint ? 'green' : 'red']"
        @click="$emit('find', { index: index })"
      >
        Find
      </button>
    </div>
  </div>
</template>

<script>
import { mapGetters } from "vuex";

export default {
  props: {
    waypoint: {
      type: Object,
      required: true,
    },
    index: {
      type: Number,
      required: true,
    },
  },

  computed: {
    ...mapGetters("erd", {
      highlightedWaypoint: "highlightedWaypoint",
    }),
  },
};
</script>

<style scoped>
.waypoint-item {
  display: grid;
  grid-template-columns: 4fr 1fr;
  grid-template-rows: 1fr 1fr;
  grid-template-areas: "name buttons" "location buttons";
  background-color: rgb(180, 180, 180);
  border-radius: 5px;
  padding: 10px;
  border: 1px solid black;
  margin: 5px;
}

.name {
  grid-area: name;
}

.location {
  grid-area: location;
}

.buttons {
  grid-area: buttons;
  align-self: center;
  justify-self: center;
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
  margin: 0px;
}
</style>
