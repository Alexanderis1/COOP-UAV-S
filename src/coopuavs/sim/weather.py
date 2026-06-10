"""Weather and lighting state (SIM-ENV-002/003, SIM-PHX-003, SIM-SEN-003).

One :class:`WeatherState` per world. It owns:

* a mean wind vector (speed + meteorological direction from the scenario)
  with a seeded Ornstein-Uhlenbeck gust perturbation and a simple
  power-law altitude shear — applied by the world to every airborne truth
  object (enemies and interceptors);
* scalar fog / precipitation / daylight states in [0, 1];
* the sensor-degradation multipliers that couple the environment into the
  sensor models (SIM-SEN-003).

EO/IR multiplier model (documented per SIM-SEN-003)
---------------------------------------------------
The tower carries a co-boresighted EO + LWIR pair, so its effective range
is the better of the two channels under the current illumination:

* EO channel quality scales with ``daylight``;
* the IR channel has a 0.9 floor and *gains* up to +0.1 at night (better
  thermal contrast, no solar clutter) — night favours IR;

giving ``lighting = max(daylight, 0.9 + 0.1 * (1 - daylight))`` — exactly
1.0 on a clear night or clear day, with a shallow dusk/dawn crossover dip.
Fog and precipitation then attenuate the optical path:
``range_factor = lighting * (1 - 0.65 fog) * (1 - 0.35 precip)``.

Acoustic pickets lose range to wind masking noise and rain; radar is only
mildly degraded by precipitation. The calm-clear-night default makes every
multiplier exactly 1.0 and draws nothing from the RNG, so legacy scenarios
without a ``weather:`` block reproduce their v0.1 trajectories bit-for-bit.
"""

from __future__ import annotations

import numpy as np


class WeatherState:
    """Environmental truth: wind field, fog, precipitation, daylight."""

    def __init__(
        self,
        rng: np.random.Generator,
        wind_speed: float = 0.0,        # m/s mean at the 10 m reference height
        wind_dir_deg: float = 0.0,      # meteorological: bearing the wind blows FROM
        fog: float = 0.0,               # 0 clear .. 1 dense
        precip: float = 0.0,            # 0 dry .. 1 heavy rain/snow
        daylight: float = 0.0,          # 0 night .. 1 full day
        gust_std: float | None = None,  # OU gust σ, default 25% of mean speed
        gust_tau: float = 8.0,          # OU correlation time, s
    ):
        self.rng = rng
        self.wind_speed = float(wind_speed)
        self.wind_dir_deg = float(wind_dir_deg)
        self.fog = float(np.clip(fog, 0.0, 1.0))
        self.precip = float(np.clip(precip, 0.0, 1.0))
        self.daylight = float(np.clip(daylight, 0.0, 1.0))
        self.gust_std = 0.25 * self.wind_speed if gust_std is None else float(gust_std)
        self.gust_tau = float(gust_tau)

        # Wind blows FROM wind_dir_deg (compass) — the air-mass velocity
        # vector points the opposite way, expressed in ENU.
        bearing = np.deg2rad(self.wind_dir_deg)
        self.mean_wind = -self.wind_speed * np.array(
            [np.sin(bearing), np.cos(bearing), 0.0]
        )
        self._gust = np.zeros(2)

    @classmethod
    def from_config(cls, cfg: dict | None, rng: np.random.Generator) -> "WeatherState":
        """Build from a scenario ``weather:`` block (missing = calm clear night)."""
        cfg = dict(cfg or {})
        return cls(rng, **cfg)

    # -- dynamics --------------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance the OU gust process one world step.

        Calm air (``gust_std == 0``) deliberately skips the RNG so legacy
        scenarios keep their exact v0.1 random stream (SIM-003).
        """
        if self.gust_std <= 0.0:
            return
        tau = max(self.gust_tau, 1e-3)
        self._gust += (-self._gust / tau) * dt + self.gust_std * np.sqrt(
            2.0 * dt / tau
        ) * self.rng.normal(0.0, 1.0, 2)

    # -- wind field --------------------------------------------------------------

    @property
    def wind(self) -> np.ndarray:
        """Instantaneous wind vector (mean + gust) at the reference height."""
        return self.mean_wind + np.array([self._gust[0], self._gust[1], 0.0])

    def wind_at(self, alt: float) -> np.ndarray:
        """Wind at altitude: power-law shear on the mean, gusts unsheared."""
        shear = float(np.clip((max(alt, 2.0) / 10.0) ** 0.14, 0.6, 1.6))
        return self.mean_wind * shear + np.array([self._gust[0], self._gust[1], 0.0])

    # -- sensor coupling (SIM-SEN-003) -------------------------------------------

    def eo_ir_range_factor(self) -> float:
        """Effective-range multiplier for EO/IR sensors (model in module doc)."""
        lighting = max(self.daylight, 0.9 + 0.1 * (1.0 - self.daylight))
        atten = (1.0 - 0.65 * self.fog) * (1.0 - 0.35 * self.precip)
        return float(np.clip(lighting * atten, 0.05, 1.0))

    def acoustic_range_factor(self) -> float:
        """Wind masking noise and rain hiss shrink the audible range."""
        wind_loss = 0.5 * min(np.linalg.norm(self.mean_wind) / 20.0, 1.0)
        return float(np.clip(1.0 - wind_loss - 0.3 * self.precip, 0.2, 1.0))

    def radar_range_factor(self) -> float:
        """Mild rain attenuation; fog and darkness are transparent to radar."""
        return float(np.clip(1.0 - 0.15 * self.precip, 0.5, 1.0))

    # -- reporting ---------------------------------------------------------------

    def as_dict(self) -> dict:
        w = self.wind
        return {
            "wind": [round(float(x), 2) for x in w],
            "fog": round(self.fog, 2),
            "precip": round(self.precip, 2),
            "daylight": round(self.daylight, 2),
        }
