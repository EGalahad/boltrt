import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  docsSidebar: [
    'intro',
    {
      type: 'category',
      collapsible: false,
      label: 'Getting Started',
      items: ['getting-started/installation', 'getting-started/quickstart'],
    },
    {
      type: 'category',
      collapsible: false,
      label: 'Core',
      items: [
        'core/architecture',
        'core/configuration',
        'core/observations',
        'core/backends',
        'core/inference',
      ],
    },
    {
      type: 'category',
      collapsible: false,
      label: 'Tasks',
      items: [
        'tasks/overview',
        {
          type: 'category',
      collapsible: false,
          label: 'Tracking',
          items: [
            'tasks/track/overview',
            'tasks/track/motion',
            'tasks/track/teleop',
            'tasks/track/policies',
          ],
        },
      ],
    },
    'real-robots',
    {
      type: 'category',
      collapsible: false,
      label: 'Extending',
      items: [
        'extending/add-observation',
        'extending/add-robot',
        'extending/add-backend',
        'extending/add-task',
        'extending/agent-skill',
      ],
    },
    {
      type: 'category',
      collapsible: false,
      label: 'Reference',
      items: ['reference/cli', 'reference/performance', 'reference/licensing'],
    },
  ],
};

export default sidebars;
