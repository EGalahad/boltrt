import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  docsSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: ['getting-started/installation', 'getting-started/quickstart'],
    },
    {
      type: 'category',
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
      label: 'Tasks',
      items: [
        'tasks/overview',
        {
          type: 'category',
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
      label: 'Reference',
      items: ['reference/cli', 'reference/performance', 'reference/licensing'],
    },
  ],
};

export default sidebars;
