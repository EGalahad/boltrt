import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const repo = 'https://github.com/EGalahad/boltrt';

const config: Config = {
  title: 'Bolt Runtime',
  tagline: 'Bolt any policy onto any robot',

  url: 'https://egalahad.github.io',
  baseUrl: '/boltrt/',

  organizationName: 'EGalahad',
  projectName: 'boltrt',

  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'throw',
    },
  },

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
          routeBasePath: '/',
          editUrl: `${repo}/tree/main/docs/`,
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Bolt Runtime',
      items: [
        {type: 'docSidebar', sidebarId: 'docsSidebar', position: 'left', label: 'Docs'},
        {href: repo, label: 'GitHub', position: 'right'},
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {label: 'Quick start', to: '/getting-started/quickstart'},
            {label: 'Core', to: '/core/architecture'},
            {label: 'Tracking task', to: '/tasks/track/overview'},
          ],
        },
        {
          title: 'More',
          items: [
            {label: 'GitHub', href: repo},
            {label: 'Python sim2real', href: 'https://github.com/EGalahad/sim2real'},
          ],
        },
      ],
      copyright: `Copyright ${new Date().getFullYear()} Bolt Runtime contributors. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'yaml', 'cpp', 'json', 'cmake'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
