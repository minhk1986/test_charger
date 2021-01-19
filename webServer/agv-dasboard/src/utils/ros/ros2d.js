import ROSLIB from "roslib";

export default {
  quaternionToGlobalTheta(orientation) {
    var q0 = orientation.w;
    var q1 = orientation.x;
    var q2 = orientation.y;
    var q3 = orientation.z;
    return -Math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 / Math.PI;
  },
  globalThetaToQuaternion(theta) {
    let qz =  Math.sin(theta/2.0);
    let qw =  Math.cos(theta/2.0);
    return new ROSLIB.Quaternion({
      x : 0,
      y : 0,
      z : qz,
      w : qw
    });
  },
}