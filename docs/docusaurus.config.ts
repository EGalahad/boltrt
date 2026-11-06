import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const repo = 'https://github.com/EGalahad/boltrt';
const fonts =
  'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600&family=Inter+Tight:wght@600;700' +
  '&family=JetBrains+Mono:wght@400;500&display=swap';

const config: Config = {
  title: 'Bolt Runtime',
  tagline: 'Bolt any policy onto any robot',

  url: process.env.DOCS_SITE_URL ?? 'https://haoyangweng.pages.dev',
  baseUrl: '/boltrt/',

  organizationName: 'EGalahad',
  projectName: 'boltrt',

  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'throw',
    },
  },

  stylesheets: [fonts],

  presets: [
    [
      'classic',
      {
        docs: {
          path: '.',
          include: ['**/*.md'],
          exclude: [
            'licenses/**',
            'media/**',
            'src/**',
            'static/**',
            'build/**',
            '.docusaurus/**',
            'node_modules/**',
          ],
          sidebarPath: './sidebars.ts',
          breadcrumbs: false,
          routeBasePath: '/',
          editUrl: `${repo}/tree/main/docs/`,
        },
        blog: false,
        theme: {
          customCss: ['./src/css/base.css', './src/css/theme.css'],
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    colorMode: {defaultMode: 'light', respectPrefersColorScheme: false},
    navbar: {
      title: 'Bolt Runtime',
      hideOnScroll: false,
      style: 'dark',
      items: [
        {type: 'doc', docId: 'getting-started/quickstart', label: 'Get started', position: 'left'},
        {type: 'doc', docId: 'core/architecture', label: 'Core', position: 'left'},
        {type: 'doc', docId: 'tasks/track/overview', label: 'Tracking', position: 'left'},
        {type: 'doc', docId: 'real-robots', label: 'Real robots', position: 'left'},
        {type: 'doc', docId: 'extending/add-observation', label: 'Extend', position: 'left'},
        {href: repo, label: 'GitHub', position: 'right', className: 'navbar-github'},
      ],
    },
    footer: {
      style: 'light',
      links: [
        {
          title: 'Docs',
          items: [
            {label: 'Introduction', to: '/intro'},
            {label: 'Quick start', to: '/getting-started/quickstart'},
            {label: 'Core', to: '/core/architecture'},
            {label: 'Tracking task', to: '/tasks/track/overview'},
          ],
        },
        {
          title: 'Project',
          items: [
            {label: 'GitHub', href: repo},
            {label: 'Performance', to: '/reference/performance'},
            {label: 'License', to: '/reference/licensing'},
            {label: 'Python sim2real', href: 'https://github.com/EGalahad/sim2real'},
          ],
        },
      ],
      copyright: `Bolt Runtime · Apache-2.0 · ${new Date().getFullYear()}`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'yaml', 'cpp', 'json', 'cmake'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
