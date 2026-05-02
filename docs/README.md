# Documentation Map

Tato slozka drzi lidskou dokumentaci a dlouhodobe poznamky mimo root workspace.

## Struktura

- `guides/` — operatorni a workflow navody
- `private/` — lokalni necitovane nebo citlive podklady, ignorovane v gitu
- `test_pahes1-5/` — finalni Phase 1-5 / Phase 15 E2E zaznamy a known issues
- lokalni ignorovane slozky jako `archive/`, `internal/`, `plans/` a
  `prompts/` mohou existovat v pracovnim adresari, ale nejsou publikovane na
  GitHub jako soucast finalniho milestone

## Dulezite soubory

- `guides/E2E_OPERATOR_GUIDE.md` — operatorni guide pro plnou E2E misi
- `test_pahes1-5/known_issues.md` — known issues z uspesneho finalniho E2E testu
- `topic_contract.md` — centralni ROS2 topic/QoS/payload contract; lidsky
  doplnek ke zdrojove pravde v `TelemetryHub`

Root workspace je vyhrazeny hlavne pro:
- realne entrypointy a launchery (`scout_launcher.py`, `isaac_launcher.py`, `reset.sh`)
- AI instrukcni soubory (`CLAUDE.md`, `codex.md`)
- top-level projektove slozky (`src/`, `swarm_center/`, `worlds/`, `launch_files/`)

## Source Of Truth

Pro finalni Phase 1-5 milestone je hlavni dokumentace:

- root `README.md`
- `launch_files/phase15_Ndrone_e2e_runbook.txt`
- `guides/E2E_OPERATOR_GUIDE.md`
- `topic_contract.md`
- `test_pahes1-5/known_issues.md`

Stare Isaac/Phase 1-3 runbooky a archivni poznamky zustavaji pro kontext, ale
jsou lokalni-only a nejsou aktualni navod pro finalni E2E demo.
