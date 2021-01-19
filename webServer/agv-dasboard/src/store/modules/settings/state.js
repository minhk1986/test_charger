export default {
  settings: [
    {
      pathStore: ["ros", "agvs", "8ee52be3-90d0-4ea9-8af5-b0c689a59030", "properties"],
      text: "AGV",
      type: "group",
      settings: [{
        pathStore: ["movementSpeed"],
        text: "Movement",
        type: "group",
        settings: [{
          pathStore: ["x"],
          text: "X axit",
          type: "double",
          default: 0,
          unit: "m/s",
          step: 0.1,
        },
        {
          pathStore: ["y"],
          text: "Y axit",
          type: "double",
          default: 0,
          unit: "m/s",
          step: 0.1,
        },
        {
          pathStore: ["z"],
          text: "Z axit",
          type: "double",
          default: 0,
          unit: "m/s",
          step: 0.1,
        }
        ],
      },
      {
        pathStore: ["rotateSpeed"],
        text: "Rotate",
        type: "group",
        settings: [{
          pathStore: ["x"],
          text: "X axit",
          type: "double",
          default: 0,
          unit: "rad/s",
          step: 0.1,
        },
        {
          pathStore: ["y"],
          text: "Y axit",
          type: "double",
          default: 0,
          unit: "rad/s",
          step: 0.1,
        },
        {
          pathStore: ["z"],
          text: "Z axit",
          type: "double",
          default: 0,
          unit: "rad/s",
          step: 0.1,
        }
        ],
      }
      ],
    },
  ]
}