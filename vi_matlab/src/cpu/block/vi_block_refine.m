function [value_table, iters, total_updates, final_delta] = vi_block_refine( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, ...
    max_iters, block_w, block_h, local_sweeps, threshold)
%VI_BLOCK_REFINE Block-frontier value iteration with fine updates per block.
%   The scheduler is coarse: it tracks changed spatial blocks.  The backup is
%   fine: every active block updates all theta states with the reference
%   Bellman operator.  With threshold=0 this converges to the same fixed point
%   as vi_full_reference, but usually touches fewer blocks on sparse maps.

    if nargin < 8 || isempty(block_w)
        block_w = 8;
    end
    if nargin < 9 || isempty(block_h)
        block_h = block_w;
    end
    if nargin < 10 || isempty(local_sweeps)
        local_sweeps = 2;
    end
    if nargin < 11 || isempty(threshold)
        threshold = 0;
    end

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    NT = p.N_THETA;
    trans_model = coerce_transition_model(transitions);

    bw = max(1, floor(block_w));
    bh = max(1, floor(block_h));
    local_passes = max(1, floor(local_sweeps));
    n_bx = ceil(map_x / bw);
    n_by = ceil(map_y / bh);

    [mx, my, ~] = vi_frontier_max_displacement(trans_model);
    rx = ceil(mx / bw);
    ry = ceil(my / bh);

    value_table(goal_mask) = 0;
    passable_blocks = blocks_from_mask(penalty_table ~= OB, map_x, map_y, ...
        bw, bh, n_bx, n_by);
    frontier_blocks = blocks_from_mask(any(value_table < MV, 3) | ...
        any(goal_mask, 3), map_x, map_y, bw, bh, n_bx, n_by);

    total_updates = 0;
    iters = 0;
    final_delta = MV;

    while any(frontier_blocks(:)) && iters < max_iters
        iters = iters + 1;
        active_blocks = dilate_blocks(frontier_blocks, n_bx, n_by, rx, ry);
        active_blocks = active_blocks & passable_blocks;

        next_frontier = false(n_by, n_bx);
        max_delta = 0;

        for by = 1:n_by
            y0 = (by - 1) * bh + 1;
            y1 = min(by * bh, map_y);
            for bx = 1:n_bx
                if ~active_blocks(by, bx)
                    continue;
                end

                x0 = (bx - 1) * bw + 1;
                x1 = min(bx * bw, map_x);
                [value_table, block_updates, block_delta] = update_block( ...
                    value_table, penalty_table, goal_mask, trans_model, ...
                    x0, x1, y0, y1, map_x, map_y, local_passes, OB, NT);

                total_updates = total_updates + block_updates;
                if block_delta > max_delta
                    max_delta = block_delta;
                end
                if block_delta > threshold
                    next_frontier(by, bx) = true;
                end
            end
        end

        final_delta = max_delta;
        frontier_blocks = next_frontier;
        if max_delta <= threshold
            break;
        end
    end

    value_table(goal_mask) = 0;
end

function [value_table, updates, max_delta] = update_block( ...
    value_table, penalty_table, goal_mask, trans_model, x0, x1, y0, y1, ...
    map_x, map_y, local_passes, OB, NT)

    updates = 0;
    max_delta = 0;
    for pass = 1:local_passes
        pass_delta = 0;
        for iy = y0:y1
            for ix = x0:x1
                if penalty_table(iy, ix) == OB
                    continue;
                end

                for it = 1:NT
                    if goal_mask(iy, ix, it)
                        value_table(iy, ix, it) = 0;
                        continue;
                    end

                    old_val = value_table(iy, ix, it);
                    v_new = vi_frontier_bellman(value_table, penalty_table, ...
                        trans_model, ix, iy, it, map_x, map_y);
                    if v_new < old_val
                        value_table(iy, ix, it) = v_new;
                        updates = updates + 1;
                        d = old_val - v_new;
                        if d > pass_delta
                            pass_delta = d;
                        end
                    end
                end
            end
        end

        if pass_delta > max_delta
            max_delta = pass_delta;
        end
        if pass_delta == 0
            break;
        end
    end
end

function blocks = blocks_from_mask(mask, map_x, map_y, bw, bh, n_bx, n_by)
    blocks = false(n_by, n_bx);
    for by = 1:n_by
        y0 = (by - 1) * bh + 1;
        y1 = min(by * bh, map_y);
        for bx = 1:n_bx
            x0 = (bx - 1) * bw + 1;
            x1 = min(bx * bw, map_x);
            region = mask(y0:y1, x0:x1);
            blocks(by, bx) = any(region(:));
        end
    end
end

function out = dilate_blocks(blocks, n_bx, n_by, rx, ry)
    out = false(n_by, n_bx);
    for by = 1:n_by
        for bx = 1:n_bx
            if ~blocks(by, bx)
                continue;
            end

            x0 = max(1, bx - rx);
            x1 = min(n_bx, bx + rx);
            y0 = max(1, by - ry);
            y1 = min(n_by, by + ry);
            out(y0:y1, x0:x1) = true;
        end
    end
end
