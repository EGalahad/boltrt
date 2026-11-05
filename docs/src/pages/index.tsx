import React from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import overview from '@site/media/overview.png';

const features = [
  {
    title: 'Modular observations',
    body: 'Adapt a new policy by composing registered observation terms in its YAML. Ten policy families load as-is.',
    to: '/core/observations',
  },
  {
    title: 'Seamless sim-to-real',
    body: 'MuJoCo and real robots implement one interface, decoupled from the policy. Switching is just --backend.',
    to: '/core/backends',
  },
  {
    title: 'Fast native C++',
    body: 'The full policy step runs 3.25× faster than the Python stack, with no Python in the control path.',
    to: '/reference/performance',
  },
];

const command = `boltrt --task tracking --robot g1 --backend mujoco \\
  --policy mimic-lite/v1_1/policy.yaml \\
  --motion motions/walk1_subject1.npz`;

export default function Home(): JSX.Element {
  return (
    <Layout title="Bolt any policy onto any robot" description="Native C++ deployment runtime for robot policies">
      <main className="bolt-home">
        <section className="bolt-hero">
          <p className="bolt-eyebrow">Bolt Runtime</p>
          <h1 className="bolt-title">Bolt any policy onto any robot.</h1>
          <p className="bolt-lede">
            A native C++ runtime that deploys learned robot policies in MuJoCo and on hardware,
            with one policy loop and plug-in observations.
          </p>
          <div className="bolt-actions">
            <Link className="button button--primary button--lg" to="/getting-started/quickstart">Get started</Link>
            <Link className="button button--secondary button--lg" href="https://github.com/EGalahad/boltrt">GitHub</Link>
          </div>
          <pre className="bolt-command"><code>{command}</code></pre>
        </section>
        <section className="bolt-figure">
          <img src={overview} alt="Modular observations, seamless sim-to-real and policy-step latency" />
        </section>
        <section className="bolt-features">
          {features.map((f) => (
            <Link key={f.title} className="bolt-feature" to={f.to}>
              <h3>{f.title}</h3>
              <p>{f.body}</p>
              <span className="bolt-more">Learn more →</span>
            </Link>
          ))}
        </section>
        <section className="bolt-policies">
          <p className="bolt-eyebrow">Runs out of the box</p>
          <p className="bolt-policy-list">
            MimicLite · BFM-Zero · GRIT · HEFT · HoloMotion · Humanoid-GPT · ScaleBFM · SONIC · TeleopIT · TWIST2
          </p>
          <Link to="/tasks/track/policies">Supported policies and parity results →</Link>
        </section>
        <section className="bolt-agent">
          <h2>Bring your own policy with a coding agent</h2>
          <p>
            The repository ships a <code>boltrt-extend</code> skill. Ask Claude Code or Codex to
            “deploy my checkpoint on G1” and it follows the same rules and checks as these docs.
          </p>
          <Link to="/extending/agent-skill">About the skill →</Link>
        </section>
      </main>
    </Layout>
  );
}
