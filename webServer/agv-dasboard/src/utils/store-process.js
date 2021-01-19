function getDeepProperty(object, properties) {
  if (properties == null || properties.length <= 0) return object;
  return getDeepProperty(object[properties[0]], [...properties].splice(1, properties.length - 1));
}
function setDeepProperty(object, value, properties) {
  if (properties == null) return;
  if (properties.length <= 0) {
    object = value;
    return;
  }
  if (properties.length == 1) {
    object[properties[0]] = value;
    return;
  }
  return setDeepProperty(object[properties[0]], value, [...properties].splice(1, properties.length - 1));
}

export default {
  getDeepProperty,
  setDeepProperty,
}