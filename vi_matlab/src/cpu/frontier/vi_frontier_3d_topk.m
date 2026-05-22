function [value_table, iters, total_updates] = vi_frontier_3d_topk( ...
    value_table, penalty_table, goal_mask, transitions, map_x, map_y, ...
    max_iters, top_k)
%VI_FRONTIER_3D_TOPK Approximate 3D frontier VI using top-k outcomes.
%   Keeps the highest-probability transition outcomes per action/theta and
%   normalizes the Bellman expectation by the retained probability mass.

    p = vi_params();
    MV = double(p.MAX_VALUE);
    OB = double(p.PENALTY_OBSTACLE);
    NT = p.N_THETA;
    full_trans_model = coerce_transition_model(transitions);
    trans_model = vi_frontier_prune_topk(full_trans_model, top_k);

    [mx, my, mt] = vi_frontier_max_displacement(trans_model);

    value_table(goal_mask) = 0;

    passable_2d = bb_from_logical2d(penalty_table ~= OB, map_x, map_y);
    passable_bb = repmat(passable_2d, [1, 1, NT]);

    goal_bb = bb_from_logical3d(goal_mask, map_x, map_y, NT);
    not_goal_bb = bitcmp(goal_bb);

    frontier = bb_from_logical3d(value_table < MV, map_x, map_y, NT);

    total_updates = 0;
    iters = 0;

    while bb_popcount(frontier) > 0 && iters < max_iters
        iters = iters + 1;

        candidates = bb_dilate3d(frontier, map_x, mx, my, mt);
        candidates = bitand(candidates, passable_bb);
        candidates = bitand(candidates, not_goal_bb);

        pts = bb_enumerate3d(candidates, map_x, map_y, NT);
        new_frontier = bb_alloc3d(map_x, map_y, NT);

        for n = 1:size(pts, 1)
            ix = pts(n, 1);
            iy = pts(n, 2);
            it = pts(n, 3);
            old_val = value_table(iy, ix, it);
            v_new = vi_frontier_bellman_norm(value_table, penalty_table, ...
                trans_model, ix, iy, it, map_x, map_y);
            if v_new == MV
                v_new = vi_frontier_bellman(value_table, penalty_table, ...
                    full_trans_model, ix, iy, it, map_x, map_y);
            end
            if v_new < old_val
                value_table(iy, ix, it) = v_new;
                total_updates = total_updates + 1;
                new_frontier = bb_set3d(new_frontier, ix, iy, it);
            end
        end

        frontier = new_frontier;
    end

    value_table(goal_mask) = 0;
end

function model = vi_frontier_prune_topk(model, top_k)
    p = vi_params();
    NA = p.N_ACTIONS;
    NT = p.N_THETA;
    MO = p.MAX_OUTCOMES;

    keep_k = floor(top_k);
    if keep_k < 1
        keep_k = 1;
    elseif keep_k > MO
        keep_k = MO;
    end

    for a = 1:NA
        for it = 1:NT
            n_out = model.n_outcomes(a, it);
            if n_out <= keep_k
                continue;
            end

            used = false(1, MO);
            dix_new = zeros(1, MO);
            diy_new = zeros(1, MO);
            dit_new = zeros(1, MO);
            prob_new = zeros(1, MO);

            for dst = 1:keep_k
                best_k = 1;
                best_prob = -1;
                for k = 1:n_out
                    if ~used(k) && model.prob(a, it, k) > best_prob
                        best_prob = model.prob(a, it, k);
                        best_k = k;
                    end
                end
                used(best_k) = true;
                dix_new(dst) = model.dix(a, it, best_k);
                diy_new(dst) = model.diy(a, it, best_k);
                dit_new(dst) = model.dit(a, it, best_k);
                prob_new(dst) = model.prob(a, it, best_k);
            end

            model.n_outcomes(a, it) = keep_k;
            for k = 1:MO
                model.dix(a, it, k) = dix_new(k);
                model.diy(a, it, k) = diy_new(k);
                model.dit(a, it, k) = dit_new(k);
                model.prob(a, it, k) = prob_new(k);
            end
        end
    end
end

function v_new = vi_frontier_bellman_norm(value_table, penalty_table, ...
    trans_model, ix, iy, it, map_x, map_y)

    p = vi_params();
    MV = double(p.MAX_VALUE);
    NT = p.N_THETA;
    NA = p.N_ACTIONS;

    v_new = MV;
    for a = 1:NA
        accum = 0;
        prob_sum = 0;
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

            prob = trans_model.prob(a, it, k);
            accum = accum + step_cost * prob;
            prob_sum = prob_sum + prob;
        end

        if valid && prob_sum > 0
            c = floor(accum / prob_sum);
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
