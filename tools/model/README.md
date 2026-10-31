# The model the firmware is generated from

| | |
|---|---|
| `mukesh.urdf` | the robot's model, **vendored** from `zeus_26_mukesh_deploy/mukesh_description/urdf/` |
| `mukesh_urdf.sha256` | its sha256, which `Appli/App/zeus_kinematics_model.h` also carries |

`tools/gen_kinematics.py` reads the copy here, not the workspace's, so a
regeneration on a laptop and one in CI read the same bytes and neither needs the
Pi workspace checked out.

**Why vendored.** This is the model the RL policy was trained against. The
estimator has to measure the same robot or every foot position it reports
belongs to a leg the policy has never seen — so the firmware should be able to
prove, on its own, which model it was built from. `stm32_zeuss` is private, so
the copy is not published.

## CI checks three things

1. **tables vs the model here** — catches `zeus_kinematics_model.h` edited by
   hand or committed from a different model. Always runs.
2. **the pin vs the model here** — catches one of the pair being refreshed
   without the other. Always runs.
3. **the model here vs the workspace's** — catches the model moving on without
   the firmware following. Needs the private repo, so it only runs when a
   read-only `WORKSPACE_TOKEN` secret exists, and prints a GitHub warning
   naming what was not checked when it does not.

The walk simulation also reads the copy here, so it runs unconditionally now.

## Refreshing it

All four steps, together — a model refreshed without regenerating fails check 1,
and a regeneration without re-pinning fails check 2.

```bash
cp ../zeus_26_mukesh_deploy/mukesh_description/urdf/mukesh.urdf tools/model/
sha256sum tools/model/mukesh.urdf | cut -d' ' -f1 > tools/model/mukesh_urdf.sha256
env -u PYTHONPATH ~/kinvenv/bin/python tools/gen_kinematics.py
tools/hosttest/run.sh
```

`env -u PYTHONPATH` matters when ROS is sourced: ROS's eigenpy breaks
Pinocchio's. The venv needs the pins in `tools/requirements-kinematics.txt` —
`pin==2.7.0` with `numpy<2`, because every 3.x/4.x wheel dies on import.

## To turn check 3 on

Add a fine-grained PAT with read access to
`Maverick280306/zeus_26_mukesh_deploy` as the `WORKSPACE_TOKEN` repository
secret.
