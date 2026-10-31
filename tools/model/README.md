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

## Check 3 without CI — do this instead

```bash
tools/model/check_drift.sh                      # finds the sibling clone
tools/model/check_drift.sh /path/to/workspace   # or say where it is
```

It compares all four hashes — upstream, vendored, pin, generated tables — and
prints the refresh commands when they disagree. Run it after pulling the
workspace. **This is the recommended way**, because it needs no credential at
all and the laptop already has both clones.

## Check 3 in CI, if you really want it

**A fine-grained PAT cannot do this.** Fine-grained tokens can only be scoped
to repositories owned by your own account or by an organisation you belong to,
and the workspace is a private repo owned by **another personal account**
(`Maverick280306`). It will not appear as a resource owner however you are
added to the repo. That is a GitHub limitation, not a permissions problem.

Three real options, best first:

1. **Read-only deploy key.** The workspace's owner adds the public half to that
   repo (Settings → Deploy keys, "Allow write access" OFF); you store the
   private half here as a secret and have the checkout step use `ssh-key`. The
   most tightly scoped thing available: one repo, read only, no account rights,
   nothing else reachable if it leaks. Needs one action from the owner.

2. **The owner creates the fine-grained token.** He *can* scope one to his own
   repo, read-only on Contents, and hand it over for the `WORKSPACE_TOKEN`
   secret. It authenticates as him and it expires, so it needs renewing.

3. **A classic PAT of your own.** Works immediately with no involvement from
   anyone, because classic tokens just carry your existing access. But the only
   scope that reaches a private repo is `repo`, which is **read AND write on
   every repository you can touch** — far more authority than this check needs.
   Use it only if the alternatives are blocked, and revoke it when done.

The workflow already has the plumbing: set `WORKSPACE_TOKEN` and the step stops
skipping.
