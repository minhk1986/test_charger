import actions from './actions'
import mutations from './mutations'
import state from './state'
import getters from './getters'
import modules from './modules'

export default {
    namespaced: true,
    state,
    getters,
    mutations,
    actions,
    modules
}