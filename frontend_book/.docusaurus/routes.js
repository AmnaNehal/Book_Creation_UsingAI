import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', 'b2f'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '182'),
    exact: true
  },
  {
    path: '/blog/authors',
    component: ComponentCreator('/blog/authors', '0b7'),
    exact: true
  },
  {
    path: '/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/blog/authors/all-sebastien-lorber-articles', '4a1'),
    exact: true
  },
  {
    path: '/blog/authors/yangshun',
    component: ComponentCreator('/blog/authors/yangshun', 'a68'),
    exact: true
  },
  {
    path: '/blog/first-blog-post',
    component: ComponentCreator('/blog/first-blog-post', '89a'),
    exact: true
  },
  {
    path: '/blog/long-blog-post',
    component: ComponentCreator('/blog/long-blog-post', '9ad'),
    exact: true
  },
  {
    path: '/blog/mdx-blog-post',
    component: ComponentCreator('/blog/mdx-blog-post', 'e9f'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '287'),
    exact: true
  },
  {
    path: '/blog/tags/docusaurus',
    component: ComponentCreator('/blog/tags/docusaurus', '704'),
    exact: true
  },
  {
    path: '/blog/tags/facebook',
    component: ComponentCreator('/blog/tags/facebook', '858'),
    exact: true
  },
  {
    path: '/blog/tags/hello',
    component: ComponentCreator('/blog/tags/hello', '299'),
    exact: true
  },
  {
    path: '/blog/tags/hola',
    component: ComponentCreator('/blog/tags/hola', '00d'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', 'd2b'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '439'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '841'),
        routes: [
          {
            path: '/docs/tags',
            component: ComponentCreator('/docs/tags', 'fce'),
            exact: true
          },
          {
            path: '/docs/tags/ai',
            component: ComponentCreator('/docs/tags/ai', 'bd3'),
            exact: true
          },
          {
            path: '/docs/tags/autonomous',
            component: ComponentCreator('/docs/tags/autonomous', 'ecf'),
            exact: true
          },
          {
            path: '/docs/tags/humanoid',
            component: ComponentCreator('/docs/tags/humanoid', '446'),
            exact: true
          },
          {
            path: '/docs/tags/llm',
            component: ComponentCreator('/docs/tags/llm', 'cef'),
            exact: true
          },
          {
            path: '/docs/tags/planning',
            component: ComponentCreator('/docs/tags/planning', '904'),
            exact: true
          },
          {
            path: '/docs/tags/python',
            component: ComponentCreator('/docs/tags/python', '954'),
            exact: true
          },
          {
            path: '/docs/tags/robotics',
            component: ComponentCreator('/docs/tags/robotics', '40e'),
            exact: true
          },
          {
            path: '/docs/tags/ros-2',
            component: ComponentCreator('/docs/tags/ros-2', '361'),
            exact: true
          },
          {
            path: '/docs/tags/urdf',
            component: ComponentCreator('/docs/tags/urdf', 'f09'),
            exact: true
          },
          {
            path: '/docs/tags/vla',
            component: ComponentCreator('/docs/tags/vla', '3dd'),
            exact: true
          },
          {
            path: '/docs/tags/voice',
            component: ComponentCreator('/docs/tags/voice', '026'),
            exact: true
          },
          {
            path: '/docs',
            component: ComponentCreator('/docs', '490'),
            routes: [
              {
                path: '/docs/category/tutorial---basics',
                component: ComponentCreator('/docs/category/tutorial---basics', '20e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/category/tutorial---extras',
                component: ComponentCreator('/docs/category/tutorial---extras', '9ad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/ai-robot-brain/isaac-ros-perception',
                component: ComponentCreator('/docs/docs/ai-robot-brain/isaac-ros-perception', 'ffe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/ai-robot-brain/navigation-nav2',
                component: ComponentCreator('/docs/docs/ai-robot-brain/navigation-nav2', 'ac3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/ai-robot-brain/nvidia-isaac-sim',
                component: ComponentCreator('/docs/docs/ai-robot-brain/nvidia-isaac-sim', 'e6b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/digital-twin/physics-simulation',
                component: ComponentCreator('/docs/docs/digital-twin/physics-simulation', 'b23'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/digital-twin/sensor-simulation',
                component: ComponentCreator('/docs/docs/digital-twin/sensor-simulation', 'a49'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/digital-twin/unity-environments',
                component: ComponentCreator('/docs/docs/digital-twin/unity-environments', '425'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/ros2-nervous-system/fundamentals',
                component: ComponentCreator('/docs/docs/ros2-nervous-system/fundamentals', '21a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/ros2-nervous-system/python-agents',
                component: ComponentCreator('/docs/docs/ros2-nervous-system/python-agents', '413'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/ros2-nervous-system/urdf-modeling',
                component: ComponentCreator('/docs/docs/ros2-nervous-system/urdf-modeling', '61f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/vla-integration/capstone-humanoid',
                component: ComponentCreator('/docs/docs/vla-integration/capstone-humanoid', 'e07'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/vla-integration/language-planning',
                component: ComponentCreator('/docs/docs/vla-integration/language-planning', 'bd6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/docs/vla-integration/voice-to-action',
                component: ComponentCreator('/docs/docs/vla-integration/voice-to-action', '5d8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/congratulations',
                component: ComponentCreator('/docs/tutorial-basics/congratulations', '458'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-blog-post',
                component: ComponentCreator('/docs/tutorial-basics/create-a-blog-post', '108'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-document',
                component: ComponentCreator('/docs/tutorial-basics/create-a-document', '8fc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/create-a-page',
                component: ComponentCreator('/docs/tutorial-basics/create-a-page', '951'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/deploy-your-site',
                component: ComponentCreator('/docs/tutorial-basics/deploy-your-site', '4f5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-basics/markdown-features',
                component: ComponentCreator('/docs/tutorial-basics/markdown-features', 'b05'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-extras/manage-docs-versions',
                component: ComponentCreator('/docs/tutorial-extras/manage-docs-versions', '978'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/tutorial-extras/translate-your-site',
                component: ComponentCreator('/docs/tutorial-extras/translate-your-site', 'f9a'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
