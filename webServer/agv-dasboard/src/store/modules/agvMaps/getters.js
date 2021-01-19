export default {
    convertMapPoseToWebPose(state) {
        return (payload) => {
            let map = state.find((m) => m.id === payload.id);
            if (map == null) return { x: 0, y: 0, }

            let mapScale = map.webMap.scale;
            let rosMapInfo = map.rosMap.info;
            return {
                x: (- rosMapInfo.origin.position.x + payload.x) / rosMapInfo.resolution * mapScale,
                y: (rosMapInfo.height + (rosMapInfo.origin.position.y - payload.y) / rosMapInfo.resolution) * mapScale,
                width: payload.width / rosMapInfo.resolution * mapScale,
                height: payload.height / rosMapInfo.resolution * mapScale,
            }
        }
    }
}