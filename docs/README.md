# Documentation Map

Tato slozka drzi lidskou dokumentaci a dlouhodobe poznamky mimo root workspace.

## Struktura

- `guides/` — operatorni a workflow navody
- `internal/` — technicke poznamky z vyvoje, debugging a historicke zavery
- `plans/` — rozpracovane planovaci materialy a navrhy
- `private/` — lokalni necitovane nebo citlive podklady, ignorovane v gitu
- `test_pahes1-5/` — finalni Phase 1-5 / Phase 15 E2E zaznamy a known issues

## Dulezite soubory

- `guides/E2E_OPERATOR_GUIDE.md` — operatorni guide pro plnou E2E misi
- `test_pahes1-5/known_issues.md` — known issues z uspesneho finalniho E2E testu
- `topic_contract.md` — centralni ROS2 topic/QoS/payload contract; lidsky
  doplnek ke zdrojove pravde v `TelemetryHub`
- `internal/technical_notes.md` — technicke poznamky k architekture a bugfixum
- `plans/scout_development_plan.docx` — starsi vyvojovy plan

Root workspace je vyhrazeny hlavne pro:
- realne entrypointy a launchery (`scout_launcher.py`, `isaac_launcher.py`, `reset.sh`)
- AI instrukcni soubory (`CLAUDE.md`, `codex.md`)
- top-level projektove slozky (`src/`, `swarm_center/`, `worlds/`, `launch_files/`)
