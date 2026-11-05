# Coding-agent skill

The repository ships a skill for coding agents in
`.agents/skills/boltrt-extend/` (also linked from `.claude/skills/`). It walks
an agent through adding robots, policies, observations, backends and tasks,
including the ownership rules, the verification steps on these pages and a list
of known pitfalls indexed by symptom.

Agents that support skills pick it up automatically when asked to, for example,
"deploy my checkpoint on G1" or "add an observation for my policy".
