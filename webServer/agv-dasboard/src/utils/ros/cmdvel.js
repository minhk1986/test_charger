export default {
  controlMetaToTwistMessage: (control) => {
    let linear = {
      x: control.y,
      y: 0,
      z: 0,
    }
    let angular = {
      x: 0,
      y: 0,
      z: -control.x,
    }

    return {
      linear,
      angular,
    }
  }
}