"""State estimation: static alignment + Sola error-state 15-state EKF.

The one coopfc subpackage where numpy is permitted (50 Hz task rate; the
import fence bans it everywhere else). Error-state layout (frozen):

    [0:3]   δp   position error, world ENU (m)
    [3:6]   δv   velocity error, world ENU (m/s)
    [6:9]   δθ   attitude error, body frame (rad)  — local/right perturbation
    [9:12]  δb_g gyro bias error, body (rad/s)
    [12:15] δb_a accel bias error, body (m/s^2)

Citations per equation in docs/RESEARCH.md, "P3 CoopFC flight stack"
(Sola 2017 for the error-state algebra; PX4-EKF2 for the delayed-horizon
fusion architecture).
"""
