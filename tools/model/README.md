# The model the firmware was generated from

`mukesh_urdf.sha256` is the sha256 of
`zeus_26_mukesh_deploy/mukesh_description/urdf/mukesh.urdf` — the URDF
`tools/gen_kinematics.py` read to write `Appli/App/zeus_kinematics_model.h`.

**Why a hash and not the URDF.** The Pi workspace is a **private** repository
and `stm32_zeuss` is **public**. Committing the model here would publish it. A
hash discloses nothing, so it lives here and the file does not.

**What this buys.** CI can always check that the generated tables and this pin
agree — that catches the common mistake, someone editing
`zeus_kinematics_model.h` by hand or committing tables from a different model,
without needing the private repo at all.

**What it does not buy.** It cannot tell you the *upstream* URDF has moved on.
That check needs the real file, so CI only runs it when a read-only token for
the workspace repo is available as the `WORKSPACE_TOKEN` secret. Without it
that step and the walk simulation are **skipped, and say so loudly** — an
unchecked step that looks checked is worse than one that admits it.

To turn the full check on, add a fine-grained PAT with read access to
`Maverick280306/zeus_26_mukesh_deploy` as the `WORKSPACE_TOKEN` repository
secret. Secrets are not exposed to pull requests from forks, so a public repo
holding one is not leaking it.

**When the model changes**, regenerate and re-pin together:

```bash
env -u PYTHONPATH ~/kinvenv/bin/python tools/gen_kinematics.py
sha256sum ../zeus_26_mukesh_deploy/mukesh_description/urdf/mukesh.urdf \
  | cut -d' ' -f1 > tools/model/mukesh_urdf.sha256
tools/hosttest/run.sh
```
