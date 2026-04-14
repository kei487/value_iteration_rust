# matlab/src Refactoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reorganize `matlab/src/` from a flat 12-file directory into `pipeline/`, `reference/`, and `util/` subdirectories, renaming 6 files to remove `_algo`/`_reference` suffixes.

**Architecture:** Move files into role-based subdirectories. Rename functions (MATLAB requires filename = function name). Update all call sites across src, test, cosim, fixedpoint, and model directories.

**Tech Stack:** MATLAB (file moves + text edits only, no new code)

---

### Task 1: Create subdirectories and move files

**Files:**
- Create: `matlab/src/pipeline/` (directory)
- Create: `matlab/src/reference/` (directory)
- Create: `matlab/src/util/` (directory)

- [ ] **Step 1: Create the three subdirectories**

```bash
mkdir -p matlab/src/pipeline matlab/src/reference matlab/src/util
```

- [ ] **Step 2: Move pipeline files (with rename)**

```bash
git mv matlab/src/vi_sweep_stream_algo.m matlab/src/pipeline/vi_sweep_stream.m
git mv matlab/src/stream_strip_algo.m matlab/src/pipeline/stream_strip.m
git mv matlab/src/load_row_algo.m matlab/src/pipeline/load_row.m
git mv matlab/src/compute_row_algo.m matlab/src/pipeline/compute_row.m
git mv matlab/src/store_row_algo.m matlab/src/pipeline/store_row.m
git mv matlab/src/cost_of.m matlab/src/pipeline/cost_of.m
```

- [ ] **Step 3: Move reference files (with rename)**

```bash
git mv matlab/src/vi_full_reference.m matlab/src/reference/vi_full_reference.m
git mv matlab/src/compute_action_table_reference.m matlab/src/reference/compute_action_table.m
```

- [ ] **Step 4: Move utility files**

```bash
git mv matlab/src/coerce_transition_model.m matlab/src/util/coerce_transition_model.m
git mv matlab/src/unpack_transitions.m matlab/src/util/unpack_transitions.m
git mv matlab/src/make_goal_mask.m matlab/src/util/make_goal_mask.m
```

- [ ] **Step 5: Verify src/ only contains vi_params.m**

```bash
ls matlab/src/*.m
```

Expected: only `matlab/src/vi_params.m`

- [ ] **Step 6: Commit**

```bash
git add -A matlab/src/
git commit -m "refactor(matlab): move src files into pipeline/reference/util subdirs"
```

---

### Task 2: Rename function declarations in moved files

Update the `function` line and doc comment in each renamed file so the MATLAB function name matches the new filename.

**Files:**
- Modify: `matlab/src/pipeline/vi_sweep_stream.m:1-3`
- Modify: `matlab/src/pipeline/stream_strip.m:1-5`
- Modify: `matlab/src/pipeline/load_row.m:1-4`
- Modify: `matlab/src/pipeline/compute_row.m:1-4`
- Modify: `matlab/src/pipeline/store_row.m:1-5`
- Modify: `matlab/src/reference/compute_action_table.m:1-2`

- [ ] **Step 1: Update vi_sweep_stream.m function declaration**

Change line 1-3 from:
```matlab
function [value_table, max_delta] = vi_sweep_stream_algo(value_table, ...
    value_table_rd, penalty_table, goal_mask, trans, map_x, map_y, cu_id)
%VI_SWEEP_STREAM_ALGO Top-level streaming VI kernel.
```
to:
```matlab
function [value_table, max_delta] = vi_sweep_stream(value_table, ...
    value_table_rd, penalty_table, goal_mask, trans, map_x, map_y, cu_id)
%VI_SWEEP_STREAM Top-level streaming VI kernel.
```

- [ ] **Step 2: Update stream_strip.m function declaration**

Change line 1-5 from:
```matlab
function [value_table, strip_max_delta] = stream_strip_algo(value_table, ...
    value_table_rd, penalty_table, goal_mask, trans_model, map_x, map_y, ...
    strip_x0, strip_w, cu_id)
%STREAM_STRIP_ALGO Process one X-strip with sliding window.
%   Paper-aligned streaming Bellman sweep.
```
to:
```matlab
function [value_table, strip_max_delta] = stream_strip(value_table, ...
    value_table_rd, penalty_table, goal_mask, trans_model, map_x, map_y, ...
    strip_x0, strip_w, cu_id)
%STREAM_STRIP Process one X-strip with sliding window.
%   Paper-aligned streaming Bellman sweep.
```

- [ ] **Step 3: Update load_row.m function declaration**

Change line 1-4 from:
```matlab
function [val_row, pen_row, goal_row] = load_row_algo(value_table, penalty_table, ...
                                                       goal_mask, gy, strip_x0, ...
                                                       strip_w, map_x, map_y)
%LOAD_ROW_ALGO Load one row with halo from value/penalty tables.
```
to:
```matlab
function [val_row, pen_row, goal_row] = load_row(value_table, penalty_table, ...
                                                  goal_mask, gy, strip_x0, ...
                                                  strip_w, map_x, map_y)
%LOAD_ROW Load one row with halo from value/penalty tables.
```

- [ ] **Step 4: Update compute_row.m function declaration**

Change line 1-4 from:
```matlab
function [val_buf, row_max_delta] = compute_row_algo(val_buf, pen_buf, ...
                                                     goal_buf, trans_model, ...
                                                     win_center, strip_w, cu_id)
%COMPUTE_ROW_ALGO Bellman update for one row in the sliding window.
```
to:
```matlab
function [val_buf, row_max_delta] = compute_row(val_buf, pen_buf, ...
                                                goal_buf, trans_model, ...
                                                win_center, strip_w, cu_id)
%COMPUTE_ROW Bellman update for one row in the sliding window.
```

- [ ] **Step 5: Update store_row.m function declaration**

Change line 1-5 from:
```matlab
function value_table = store_row_algo(val_row, value_table, ...
                                       gy, strip_x0, strip_w, map_x)
%STORE_ROW_ALGO Store one row (inner cells, no halo) back to value table.
```
to:
```matlab
function value_table = store_row(val_row, value_table, ...
                                  gy, strip_x0, strip_w, map_x)
%STORE_ROW Store one row (inner cells, no halo) back to value table.
```

- [ ] **Step 6: Update compute_action_table.m function declaration**

Change line 1-2 from:
```matlab
function action_table = compute_action_table_reference(value_table, penalty_table, ...
    goal_mask, transitions, map_x, map_y)
%COMPUTE_ACTION_TABLE_REFERENCE Compute argmin action using paper semantics.
```
to:
```matlab
function action_table = compute_action_table(value_table, penalty_table, ...
    goal_mask, transitions, map_x, map_y)
%COMPUTE_ACTION_TABLE Compute argmin action using paper semantics.
```

- [ ] **Step 7: Commit**

```bash
git add matlab/src/pipeline/ matlab/src/reference/
git commit -m "refactor(matlab): rename functions to match new filenames"
```

---

### Task 3: Update internal call sites within src/

Update calls between src files to use new function names.

**Files:**
- Modify: `matlab/src/pipeline/vi_sweep_stream.m:26`
- Modify: `matlab/src/pipeline/stream_strip.m:22,36,42,51`
- Modify: `matlab/src/reference/vi_full_reference.m:48,52`

- [ ] **Step 1: Update vi_sweep_stream.m — stream_strip_algo → stream_strip**

Change line 26:
```matlab
        [value_table, strip_delta] = stream_strip_algo(value_table, ...
```
to:
```matlab
        [value_table, strip_delta] = stream_strip(value_table, ...
```

- [ ] **Step 2: Update stream_strip.m — load_row_algo → load_row (line 22)**

Change line 22:
```matlab
        [val_buf(slot, :, :), pen_row, goal_row] = load_row_algo(value_table_rd, ...
```
to:
```matlab
        [val_buf(slot, :, :), pen_row, goal_row] = load_row(value_table_rd, ...
```

- [ ] **Step 3: Update stream_strip.m — compute_row_algo → compute_row (line 36)**

Change line 36:
```matlab
        [val_buf, row_delta] = compute_row_algo(val_buf, pen_buf, goal_buf, ...
```
to:
```matlab
        [val_buf, row_delta] = compute_row(val_buf, pen_buf, goal_buf, ...
```

- [ ] **Step 4: Update stream_strip.m — store_row_algo → store_row (line 42)**

Change line 42:
```matlab
        value_table = store_row_algo(squeeze(val_buf(win_center, :, :)), ...
```
to:
```matlab
        value_table = store_row(squeeze(val_buf(win_center, :, :)), ...
```

- [ ] **Step 5: Update stream_strip.m — load_row_algo → load_row (line 51)**

Change line 51:
```matlab
        [val_buf(evict_slot, :, :), pen_row, goal_row] = load_row_algo(value_table_rd, ...
```
to:
```matlab
        [val_buf(evict_slot, :, :), pen_row, goal_row] = load_row(value_table_rd, ...
```

- [ ] **Step 6: Update vi_full_reference.m — local compute_action_table_reference → compute_action_table**

Change line 48:
```matlab
    action_table = compute_action_table_reference(value_table, penalty_table, ...
```
to:
```matlab
    action_table = compute_action_table(value_table, penalty_table, ...
```

Also change the local function definition at line 52:
```matlab
function action_table = compute_action_table_reference(value_table, penalty_table, ...
```
to:
```matlab
function action_table = compute_action_table(value_table, penalty_table, ...
```

Note: This is a **local** (nested) function inside `vi_full_reference.m`. It shadows the standalone `reference/compute_action_table.m` when called from within `vi_full_reference`. Both need the same name for consistency.

- [ ] **Step 7: Commit**

```bash
git add matlab/src/
git commit -m "refactor(matlab): update internal call sites for renamed functions"
```

---

### Task 4: Update test call sites

**Files:**
- Modify: `matlab/test/TestAlgorithmUnits.m:51,59,80,83,114,149`
- Modify: `matlab/test/TestSolverIntegration.m:30,33,41,76,79,89`

- [ ] **Step 1: Update TestAlgorithmUnits.m — load_row_algo → load_row**

Three call sites at lines 51, 59, 80. Change every occurrence of `load_row_algo(` to `load_row(`.

Line 51:
```matlab
            [val_row, pen_row] = load_row(value_table, penalty_table, ...
```

Line 59:
```matlab
            [val_oob, pen_oob] = load_row(value_table, penalty_table, ...
```

Line 80:
```matlab
            [val_row, ~] = load_row(value_table, penalty_table, ...
```

- [ ] **Step 2: Update TestAlgorithmUnits.m — store_row_algo → store_row**

Line 83:
```matlab
            value_table = store_row(val_row, value_table, ...
```

- [ ] **Step 3: Update TestAlgorithmUnits.m — compute_row_algo → compute_row**

Line 114:
```matlab
            [val_buf_out, row_max_delta] = compute_row(val_buf, pen_buf, ...
```

- [ ] **Step 4: Update TestAlgorithmUnits.m — stream_strip_algo → stream_strip**

Line 149:
```matlab
            [value_out, strip_delta] = stream_strip(value, value, ...
```

- [ ] **Step 5: Update TestSolverIntegration.m — vi_sweep_stream_algo → vi_sweep_stream**

Four call sites at lines 30, 33, 76, 79. Change every occurrence of `vi_sweep_stream_algo(` to `vi_sweep_stream(`.

Line 30:
```matlab
                [ml_value, delta0] = vi_sweep_stream(ml_value, ml_value, ...
```

Line 33:
```matlab
                [ml_value, delta1] = vi_sweep_stream(ml_value, ml_value, ...
```

Line 76:
```matlab
                [fpga_value, delta0] = vi_sweep_stream(fpga_value, ...
```

Line 79:
```matlab
                [fpga_value, delta1] = vi_sweep_stream(fpga_value, ...
```

- [ ] **Step 6: Update TestSolverIntegration.m — compute_action_table_reference → compute_action_table**

Two call sites at lines 41 and 89. Change every occurrence of `compute_action_table_reference(` to `compute_action_table(`.

Line 41:
```matlab
            ml_action = compute_action_table(ml_value, penalty, ...
```

Line 89:
```matlab
            fpga_action = compute_action_table(fpga_value, penalty, ...
```

- [ ] **Step 7: Commit**

```bash
git add matlab/test/
git commit -m "refactor(matlab): update test call sites for renamed functions"
```

---

### Task 5: Update cosim, fixedpoint, and model call sites

**Files:**
- Modify: `matlab/run_matlab_tests.m:8`
- Modify: `matlab/cosim/cosim_tb.m:9,34,37`
- Modify: `matlab/fixedpoint/fp_config.m:45,46`
- Modify: `matlab/model/create_model.m:31`

- [ ] **Step 1: Update run_matlab_tests.m — addpath**

Line 8 currently adds only `src`. Add subdirectories:

Change line 8:
```matlab
    addpath(fullfile(root_dir, 'src'));
```
to:
```matlab
    addpath(fullfile(root_dir, 'src'));
    addpath(fullfile(root_dir, 'src', 'pipeline'));
    addpath(fullfile(root_dir, 'src', 'reference'));
    addpath(fullfile(root_dir, 'src', 'util'));
```

- [ ] **Step 2: Update cosim_tb.m — addpath**

Line 9 currently adds `../src` to the path. Since files are now in subdirectories, add all three plus the root:

Change line 9:
```matlab
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
```
to:
```matlab
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'pipeline'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'reference'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'util'));
```

- [ ] **Step 3: Update cosim_tb.m — vi_sweep_stream_algo → vi_sweep_stream**

Line 34:
```matlab
            [ml_value, d0] = vi_sweep_stream(ml_value, ml_value, ...
```

Line 37:
```matlab
            [ml_value, d1] = vi_sweep_stream(ml_value, ml_value, ...
```

- [ ] **Step 4: Update fp_config.m — addpath**

Line 6 currently adds `../src`. Add subdirectories:

Change line 6:
```matlab
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
```
to:
```matlab
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'pipeline'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'reference'));
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src', 'util'));
```

- [ ] **Step 5: Update fp_config.m — vi_sweep_stream_algo → vi_sweep_stream**

Line 45:
```matlab
        [value, ~] = vi_sweep_stream(value, value, penalty, trans, 32, 32, 0);
```

Line 46:
```matlab
        [value, ~] = vi_sweep_stream(value, value, penalty, trans, 32, 32, 1);
```

- [ ] **Step 6: Update create_model.m — addpath**

Line 10 currently adds `../src`. Add subdirectories:

Change line 10:
```matlab
    addpath(fullfile(model_dir, '..', 'src'));
```
to:
```matlab
    addpath(fullfile(model_dir, '..', 'src'));
    addpath(fullfile(model_dir, '..', 'src', 'pipeline'));
    addpath(fullfile(model_dir, '..', 'src', 'reference'));
    addpath(fullfile(model_dir, '..', 'src', 'util'));
```

- [ ] **Step 7: Update create_model.m — comment reference**

Line 31:
```matlab
    % The function references vi_sweep_stream.m
```

- [ ] **Step 8: Commit**

```bash
git add matlab/run_matlab_tests.m matlab/cosim/ matlab/fixedpoint/ matlab/model/
git commit -m "refactor(matlab): update runner/cosim/fixedpoint/model for new src layout"
```

---

### Task 6: Update Makefile matlab targets

**Files:**
- Modify: `Makefile` (check matlab-related targets for any src path references)

- [ ] **Step 1: Check Makefile for src path references**

```bash
grep -n "matlab/src" Makefile
```

If no direct path references to individual src files, skip this task. The Makefile invokes MATLAB which uses the project path, so no changes are needed unless paths are hardcoded.

- [ ] **Step 2: Commit (if changes made)**

```bash
git add Makefile
git commit -m "refactor(matlab): update Makefile for new src layout"
```

---

### Task 7: Verify tests pass

- [ ] **Step 1: Run MATLAB tests**

```bash
make matlab-sim
```

Expected: all tests in `TestAlgorithmUnits` and `TestSolverIntegration` pass.

- [ ] **Step 2: If tests fail, check error messages**

Common failure modes:
- "Undefined function" → a call site was missed. Grep for old names:
  ```bash
  grep -rn "stream_strip_algo\|load_row_algo\|compute_row_algo\|store_row_algo\|vi_sweep_stream_algo\|compute_action_table_reference" matlab/
  ```
- Path not found → MATLAB project needs subdirectories added to path (manual step, see below).

- [ ] **Step 3: Commit verification note**

No commit needed — this is a verification step only.

---

### Task 8: Post-refactor manual step (user action)

This task requires the user to perform in MATLAB GUI.

- [ ] **Step 1: Document instructions for user**

After all code changes, the user must open MATLAB and:

1. Open the project: double-click `matlab/value_iteration_fpga.prj`
2. In the Project panel, add these folders to the project path:
   - `src/pipeline/`
   - `src/reference/`
   - `src/util/`
3. Remove any stale references to files that no longer exist at their old paths
4. Save the project

This ensures MATLAB can find all functions when running from the project context (as opposed to the `addpath` calls in cosim/fixedpoint/model which handle standalone execution).
