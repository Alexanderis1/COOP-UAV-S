"""Mission-computer side software (PHY-UAV-010 split).

P4-2 ships ``fcu_client.py`` — the MC endpoint of the FCU coop-link;
the tactical apps move here from ``interceptors/`` in P4-3/P4-5.
Import discipline: like ``coopfc/``, nothing here may reach into the
sim/world truth (boundary test lands with P4-4 per plan).
"""
