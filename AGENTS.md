# SpaceNavSim Agent Rules

## <span style="color:#c00">CRITICAL: EXACT REQUEST ONLY</span>

- Implement only the classes, fields, methods, behavior, assets, and files the user explicitly requests. Never add inferred or adjacent functionality, even if it seems necessary for a complete mechanic.
- If the requested scope leaves a feature incomplete, leave it incomplete and explain the limit. Discuss any additional work before changing files.

### Hard task boundary

- The user's explicit request is the complete scope. Do not add related investigation, edits, checks, tests, builds, editor actions, cleanup, documentation, or journal entries because they seem useful, necessary, or customary.
- Before every tool call or filesystem operation, identify the exact requested item it directly serves. If it serves no requested item, do not perform it. Reading active project instructions as required by this file is the sole automatic exception.
- Use the narrowest inspection needed to answer each requested question. Once every requested item is answered or completed, stop. Do not continue diagnosing an adjacent issue.
- Treat each additional operation as a separate proposal. State its exact scope and wait for the user's explicit approval before investigating or performing it. Never infer approval from silence, an earlier task, or a general desire for a complete result.
- If another instruction appears to require an unrequested operation, ask the user about that operation first. A refusal means the operation is forbidden; complete the requested scope without it when possible.

## Approval gate

- Workflow: discuss -> propose -> agree -> execute.
- Until the user explicitly writes the Russian command represented by Unicode code points U+0434 U+0435 U+043B U+0430 U+0439 (case-insensitive), perform read-only work only: inspect, diagnose, analyze, answer, and propose. Requests, problem descriptions, and other imperative wording do not authorize implementation.
- Before requesting that command, specify every proposed filesystem operation with its exact path; any affected classes, Blueprints, maps, assets, or settings; the behavior change and reason; and the checks to perform. Name a journal file only if the user explicitly requested a journal operation.
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
- Do not create or append a journal unless the user explicitly requests that journal operation. If a workflow calls for a journal but the user has not requested one, ask first. A refusal forbids the journal write.
- When a journal is explicitly requested, use concise English bullets covering changed files or settings, behavior change, performed checks, and whether a build or editor check ran. Name the exact journal file before requesting approval. Creating a new journal requires the same approval as any other file.

## Editable Actor settings

- Every designer-editable Actor field, including editable C++ properties and Blueprint variables, belongs to a purpose-based Details category with the full path Settings|Topic (for example Settings|Input, Settings|Data, Settings|Movement, Settings|SFX, or Settings|Debug).
- Do not use a bare Settings category, an unrelated top-level category, or no category. Keep equivalent settings in the same mechanic under the same full path. When an existing Actor is in scope, align its old user-facing categories with this convention.
- Every C++ Actor with displayed settings must declare UCLASS PrioritizeCategories with every full displayed Settings|Topic path. Update it when categories change. A derived C++ Actor used as a Blueprint base must include its own and inherited displayed Settings paths.
- Transform and Materials are engine component sections and may remain above Settings; Actor metadata cannot guarantee their positions.
- Components, runtime state, replicated informational fields, and delegates that are not designer settings belong in technical categories such as ClassName|Components, ClassName|Runtime, or ClassName|Events, never Settings.
