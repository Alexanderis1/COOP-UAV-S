"""P3-5 cascade flight control (PX4-style structure, plain-float).

Pipeline (rates per the sched groups; wiring at the FCU level, P3-6)::

    velocity (50 Hz)  ->  attitude (400 Hz)  ->  body rate (400 Hz)
        VelCtl.update         AttCtl.update        RateCtl.update
        -> q_sp, thrust       -> rate_sp           -> torque [-1,1]^3
                                                        |
                                          mixer.QuadXMixer.mix (400 Hz)
                                          -> 4 motor commands [0,1] + flags

Truth never enters: every input is an estimate (EKF NavState) or a
setpoint. All hot paths are plain-float (import fence). Equation sources
in docs/RESEARCH.md "P3 CoopFC flight stack".
"""

from coopuavs.coopfc.control.attitude import AttCtl, AttParams
from coopuavs.coopfc.control.mixer import MixFlags, QuadXMixer
from coopuavs.coopfc.control.position import PosCtl, PosParams
from coopuavs.coopfc.control.rate import RateCtl, RateParams
from coopuavs.coopfc.control.velocity import VelCtl, VelParams

__all__ = [
    "AttCtl", "AttParams", "MixFlags", "PosCtl", "PosParams", "QuadXMixer",
    "RateCtl", "RateParams", "VelCtl", "VelParams",
]
