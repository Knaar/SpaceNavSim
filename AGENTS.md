# SpaceNavSim Agent Rules

## Approval gate

- Workflow: discuss -> propose -> agree -> execute.
- Until the user explicitly writes the Russian command represented by Unicode code points U+0434 U+0435 U+043B U+0430 U+0439 (case-insensitive), perform read-only work only: inspect, diagnose, analyze, answer, and propose. Requests, problem descriptions, and other imperative wording do not authorize implementation.
- Before requesting that command, specify every proposed filesystem operation with its exact path; any affected classes, Blueprints, maps, assets, or settings; the behavior change and reason; the checks to perform; and the exact journal file to create or append.
- That command authorizes only the already proposed scope. Do not add adjacent work. Blueprint, map, Data Asset, build, and editor actions require explicit inclusion in the agreed scope.
- If the user revises or discusses a proposal without that command, update the proposal and remain read-only.
- Before approval, do not create, edit, delete, or move files; change code, assets, configuration, plugins, or editor settings; run processes that change state; mutate Git state; deploy; send messages; or modify external services.

## Paths

- If a destination directory is unspecified, ask before creating, moving, or splitting files. Do not infer a folder layout.
- List exact paths for every filesystem operation before approval. Approval applies only to listed operations.

## Builds and Unreal Editor

- Do not initiate or add builds, tests, project generation, packaging, Unreal Editor, PIE, or Standalone Game to a plan on your own.
- A general approval command does not authorize these actions. Run only the specific operation the user separately and explicitly requested within the agreed scope.
- If such an operation is necessary, explain why and wait for the direct request.

## Language and logs

- Communicate with the user in Russian. Use English only in every agent-authored project file: names, identifiers, comments, strings, documentation, configuration, and journals. Never introduce Cyrillic characters into project files.
- All UE_LOG message text and other runtime/debug log text must be in English.
- Record every completed change in its related Logs/SpaceNavSim/<scenario>/ journal. Use concise English bullets covering changed files or settings, behavior change, performed checks, and whether a build or editor check ran. Work is incomplete until the entry exists.
- Before requesting approval, name the exact journal file. Creating a new journal requires the same approval as any other file.

## Editable Actor settings

- Every designer-editable Actor field, including editable C++ properties and Blueprint variables, belongs to a purpose-based Details category with the full path Settings|Topic (for example Settings|Input, Settings|Data, Settings|Movement, Settings|SFX, or Settings|Debug).
- Do not use a bare Settings category, an unrelated top-level category, or no category. Keep equivalent settings in the same mechanic under the same full path. When an existing Actor is in scope, align its old user-facing categories with this convention.
- Every C++ Actor with displayed settings must declare UCLASS PrioritizeCategories with every full displayed Settings|Topic path. Update it when categories change. A derived C++ Actor used as a Blueprint base must include its own and inherited displayed Settings paths.
- Transform and Materials are engine component sections and may remain above Settings; Actor metadata cannot guarantee their positions.
- Components, runtime state, replicated informational fields, and delegates that are not designer settings belong in technical categories such as ClassName|Components, ClassName|Runtime, or ClassName|Events, never Settings.
