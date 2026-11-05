# Add a task

A task changes what the loop does around the policy, as tracking does with
reference motion.

1. Create `src/tasks/<name>/` and derive a loop from `PolicyLoop`. Override only
   the hooks you need: `prepare_task`, `start_task`, `enter_policy_task`,
   `update_task`, `handle_task_event`, `task_finished`,
   `update_task_status`, `append_task_record`, `bind_task_context`.
2. Put task data that observations need in a `TaskContext` subclass and bind it
   in `bind_task_context`; generic observations never see it.
3. Register the task, its CLI options and its observation terms with
   `TaskRegistry` from `apps/boltrt.cpp`.

Never add task behavior to `PolicyLoop` itself; the base loop must keep running
with no task at all.
