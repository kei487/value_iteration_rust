function [value_table, iters, total_updates] = vi_frontier_3d_coarse_theta( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, ...
    max_iters, coarse_step, refine_iters)
%VI_FRONTIER_3D_COARSE_THETA Coarse-theta approximate solve plus refine.
%   Solves only every coarse_step theta layer using snapped transition theta,
%   upsamples that value field to all layers, then runs exact frontier refine.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    NT = p.N_THETA;
    trans_model = coerce_transition_model(transitions);

    step = floor(coarse_step);
    if step <= 1
        [value_table, iters, total_updates] = vi_frontier_3d( ...
            value_table, penalty_table, goal_mask, trans_model, ...
            map_x, map_y, max_iters);
        return;
    end

    refine_cap = floor(refine_iters);
    if refine_cap < 0
        refine_cap = 0;
    elseif refine_cap > max_iters
        refine_cap = max_iters;
    end
    coarse_cap = max_iters - refine_cap;

    [value_table, coarse_iters, coarse_updates] = run_coarse_theta( ...
        value_table, penalty_table, goal_mask, trans_model, ...
        map_x, map_y, coarse_cap, step, MV, OB, NT);

    value_table = upsample_coarse_theta(value_table, step, NT);
    value_table(goal_mask) = 0;

    refine_done = 0;
    refine_updates = 0;
    if refine_cap > 0
        [value_table, refine_done, refine_updates] = vi_frontier_3d( ...
            value_table, penalty_table, goal_mask, trans_model, ...
            map_x, map_y, refine_cap);
    end

    iters = coarse_iters + refine_done;
    total_updates = coarse_updates + refine_updates;
end

function [value_table, iters, total_updates] = run_coarse_theta( ...
    value_table, penalty_table, goal_mask, trans_model, map_x, map_y, ...
    max_iters, step, MV, OB, NT)

    [mx, my, mt] = vi_frontier_max_displacement(trans_model);

    coarse_layers = false(1, NT);
    for it = 1:step:NT
        coarse_layers(it) = true;
    end

    coarse_goal = false(size(goal_mask));
    for it = 1:NT
        cit = nearest_coarse_theta(it, step, NT);
        coarse_goal(:, :, cit) = coarse_goal(:, :, cit) | goal_mask(:, :, it);
    end

    value_table(:) = MV;
    value_table(coarse_goal) = 0;

    passable_2d = bb_from_logical2d(penalty_table ~= OB, map_x, map_y);
    passable_bb = repmat(passable_2d, [1, 1, NT]);

    coarse_layer_mask = false(size(goal_mask));
    for it = 1:NT
        if coarse_layers(it)
            coarse_layer_mask(:, :, it) = true;
        end
    end
    coarse_layer_bb = bb_from_logical3d(coarse_layer_mask, map_x, map_y, NT);
    goal_bb = bb_from_logical3d(coarse_goal, map_x, map_y, NT);
    not_goal_bb = bitcmp(goal_bb);

    frontier = bb_from_logical3d(coarse_goal, map_x, map_y, NT);

    total_updates = 0;
    iters = 0;

    while bb_popcount(frontier) > 0 && iters < max_iters
        iters = iters + 1;

        candidates = bb_dilate3d(frontier, map_x, mx, my, mt);
        candidates = bitand(candidates, passable_bb);
        candidates = bitand(candidates, coarse_layer_bb);
        candidates = bitand(candidates, not_goal_bb);

        pts = bb_enumerate3d(candidates, map_x, map_y, NT);
        new_frontier = bb_alloc3d(map_x, map_y, NT);

        for n = 1:size(pts, 1)
            ix = pts(n, 1);
            iy = pts(n, 2);
            it = pts(n, 3);
            old_val = value_table(iy, ix, it);
            v_new = vi_frontier_bellman_coarse_theta(value_table, ...
                penalty_table, trans_model, ix, iy, it, map_x, map_y, ...
                step, NT);
            if v_new < old_val
                value_table(iy, ix, it) = v_new;
                total_updates = total_updates + 1;
                new_frontier = bb_set3d(new_frontier, ix, iy, it);
            end
        end

        frontier = new_frontier;
    end
end

function value_table = upsample_coarse_theta(value_table, step, NT)
    for it = 1:NT
        cit = nearest_coarse_theta(it, step, NT);
        if cit ~= it
            value_table(:, :, it) = value_table(:, :, cit);
        end
    end
end

function v_new = vi_frontier_bellman_coarse_theta(value_table, penalty_table, ...
    trans_model, ix, iy, it, map_x, map_y, step, NT)

    p = vi_params();
    MV = double(p.MAX_VALUE);
    PB = double(p.PROB_BASE);
    NA = p.N_ACTIONS;

    v_new = MV;
    for a = 1:NA
        accum = 0;
        n_out = trans_model.n_outcomes(a, it);
        valid = true;
        for k = 1:n_out
            nx = ix + trans_model.dix(a, it, k);
            ny = iy + trans_model.diy(a, it, k);
            nt = it + trans_model.dit(a, it, k);

            if nt < 1
                nt = nt + NT;
            elseif nt > NT
                nt = nt - NT;
            end
            nt = nearest_coarse_theta(nt, step, NT);

            if nx < 1 || nx > map_x || ny < 1 || ny > map_y
                accum = MV;
                valid = false;
                break;
            end

            step_cost = cost_of(value_table(ny, nx, nt), penalty_table(ny, nx));
            if step_cost == MV
                accum = MV;
                valid = false;
                break;
            end

            accum = accum + step_cost * trans_model.prob(a, it, k);
        end

        if valid
            c = floor(accum / PB);
            if c >= MV
                c = MV - 1;
            end
        else
            c = MV;
        end
        if c < v_new
            v_new = c;
        end
    end
end

function cit = nearest_coarse_theta(it, step, NT)
    idx0 = it - 1;
    q = floor((idx0 + floor(step / 2)) / step);
    cit = q * step + 1;
    while cit > NT
        cit = cit - NT;
    end
end
