import control as ct
import numpy as np


def compute_stability_margins(L_tf: ct.TransferFunction):
    """Computes gain margin and phase margin of loop transfer function."""
    gm, pm, sm, wcg, wcp, wcs = ct.stability_margins(L_tf)
    gm_db = 20.0 * np.log10(gm) if gm is not None and gm > 0 and not np.isinf(gm) else float('inf')
    return {
        'gm_db': gm_db,
        'pm_deg': pm,
        'wcp': wcp
    }
