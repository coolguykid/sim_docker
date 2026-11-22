"""Sparse convex optimization for the digital-twin guidance policy."""
import numpy as np
from scipy import sparse


def solve_policy(network, counts, q, initial, density, penalty, previous, limits, config):
    """Optimize occupied rows. Density and its mean are auxiliary variables."""
    import osqp

    sources, targets = np.nonzero(network.legal & (counts > 0)[:, None])
    n_cells, n_vars = len(counts), len(sources)
    columns = np.arange(n_vars)
    occupied, row_ids = np.unique(sources, return_inverse=True)
    region_targets = network.cell_regions[targets]
    group_ids = row_ids * q.shape[1] + region_targets
    rows = sparse.csc_matrix((np.ones(n_vars), (row_ids, columns)), shape=(len(occupied), n_vars))
    groups = sparse.csc_matrix((np.ones(n_vars), (group_ids, columns)),
                               shape=(len(occupied) * q.shape[1], n_vars))
    incoming = sparse.csc_matrix(
        (np.where(sources != targets, counts[sources] / network.capacities[targets], 0.),
         (targets, columns)), shape=(n_cells, n_vars))

    center = bool(np.all(penalty == 1.))
    identity = sparse.eye(n_cells, format="csc")
    density_score = (sparse.hstack((identity, -np.ones((n_cells, 1))), format="csc")
                     if center else sparse.diags(np.sqrt(penalty), format="csc"))
    density_variables = density_score.shape[1]
    objective_scale = max(config.intention_weight, config.balance_weight * penalty.max())
    intention_weight = config.intention_weight / objective_scale
    balance_weight = config.balance_weight / objective_scale
    quadratic = sparse.block_diag(
        (2 * intention_weight * (groups.T @ groups),
         2 * balance_weight * (density_score.T @ density_score)), format="csc")
    linear = np.r_[-2 * intention_weight * (groups.T @ q[occupied].ravel()),
                   np.zeros(density_variables)]
    predict = sparse.hstack((-incoming, identity, sparse.csc_matrix((n_cells, int(center)))))
    row_sum = sparse.hstack((rows, sparse.csc_matrix((len(occupied), density_variables))))
    blocks = [row_sum, predict, sparse.eye(n_vars + density_variables, format="csc")]
    lower = np.zeros(n_vars)
    upper = np.minimum(1., limits[sources, targets] / counts[sources])
    if previous is not None:
        old = previous[sources, targets]
        lower = np.maximum(lower, old - config.max_probability_change)
        upper = np.minimum(upper, old + config.max_probability_change)
    if np.any(lower > upper) or np.any(density > config.density_limit + 1.e-8):
        return None, "infeasible"
    lb = np.r_[np.ones(len(occupied)), density, lower, np.zeros(density_variables)]
    ub = np.r_[np.ones(len(occupied)), density, upper,
               np.full(density_variables, config.density_limit)]
    if center:
        mean = sparse.csc_matrix(np.r_[np.zeros(n_vars), np.full(n_cells, 1. / n_cells), -1.][None, :])
        blocks.append(mean)
        lb, ub = np.r_[lb, 0.], np.r_[ub, 0.]
    solver = osqp.OSQP()
    solver.setup(P=sparse.triu(quadratic, format="csc"), q=linear,
                 A=sparse.vstack(blocks, format="csc"), l=lb, u=ub,
                 eps_abs=1.e-8, eps_rel=1.e-8, max_iter=10000, verbose=False)
    result = solver.solve(raise_error=False)
    if result.info.status_val not in (1, 2) or not np.all(np.isfinite(result.x)):
        return None, result.info.status

    # Clean up numerical probabilities and check the cleaned policy itself.
    values = np.clip(result.x[:n_vars], lower, upper)
    totals = rows @ values
    if np.any(totals <= 0.):
        return None, "invalid row probabilities"
    values /= totals[row_ids]
    eps = 1.e-6
    if (np.any(values < lower - eps) or np.any(values > upper + eps)
            or np.any(density + incoming @ values > config.density_limit + eps)):
        return None, "constraint residual too large"
    policy = initial.copy()
    policy[sources, targets] = values
    return policy, "solved"
