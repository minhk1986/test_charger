import Overview from '@/views/dashboard/Overview';
import Navigation from '@/views/dashboard/Navigation';
import Setting from '@/views/dashboard/Setting';
import Security from '@/views/dashboard/Security';
import NavigationManual from '@/views/dashboard/NavigationManual';

import {
  AgvNavigation,
  AgvInitializeNavigation,
} from "@/components/AgvNavigation";

export default [
  {
    path: '',
    component: Overview,
    meta: {
      title: 'Overview'
    }
  },
  {
    path: 'navigation',
    component: Navigation,
    children: [
      {
        path: '/',
        component: AgvNavigation,
        meta: {
          title: 'Navigation'
        }
      },
      {
        path: 'init',
        component: AgvInitializeNavigation,
        meta: {
          title: 'Navigation Initialize Pose'
        }
      },
    ]
  },
  {
    path: 'manual',
    component: NavigationManual,
    meta: {
      title: 'Navigation Manual'
    }
  },
  {
    path: 'settings',
    component: Setting,
    meta: {
      title: 'Setting'
    }
  },
  {
    path: 'security',
    component: Security,
    meta: {
      title: 'Security'
    }
  },
]
