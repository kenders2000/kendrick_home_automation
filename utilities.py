import numpy as np


class ExponentialAverager:
    def __init__(self, alpha=0.1):
        """
        alpha: smoothing factor, 0 < alpha <= 1
               higher alpha = more weight to new data
        """
        self.alpha = alpha
        self.avg = None  # Will hold the current average

    def reset(self):
        self.avg = None  # Will hold the current average

    def update(self, new_frame: np.ndarray):
        """
        Update the moving average with a new frame.
        Returns the updated average.
        """
        if self.avg is None:
            self.avg = new_frame.astype(np.float32)  # Initialize with first frame
        else:
            self.avg = self.alpha * new_frame + (1 - self.alpha) * self.avg
        return self.avg


def asymmetric_gaussian(x, mu=0, sigma_left=0.5, sigma_right=2.0):
    """
    Compute an asymmetric Gaussian function.

    The function returns a Gaussian-shaped curve centered at `mu`, but with different
    standard deviations (`sigma_left` and `sigma_right`) on the left and right sides.

    Args:
        x (array-like): Input values where the function is evaluated.
        mu (float): Center (mean) of the Gaussian.
        sigma_left (float): Standard deviation for values less than `mu`.
        sigma_right (float): Standard deviation for values greater than or equal to `mu`.

    Returns:
        np.ndarray: Array of the same shape as `x` containing the asymmetric Gaussian values.
    """
    return np.where(
        x < mu,
        np.exp(-0.5 * ((x - mu) / sigma_left) ** 2),
        np.exp(-0.5 * ((x - mu) / sigma_right) ** 2),
    )
class KalmanFilter1D_CV:
    """
    1D constant-velocity Kalman filter with state x = [position, velocity]^T.
    Measurement is position only. Supports variable dt and outlier gating.

    Model:
        x_k   = F(dt) x_{k-1} + B(dt) * a_k       (a_k optional control: accel)
        z_k   = H x_k + v_k
        F(dt) = [[1, dt],
                 [0,  1]]
        H     = [1, 0]
        Q(dt) = q * [[dt^3/3, dt^2/2],
                     [dt^2/2,     dt]]   (white-noise acceleration spectral density q)
        R     = [r]  (measurement variance)

    Args:
        x0: initial position
        v0: initial velocity
        P0: 2x2 covariance matrix OR tuple/list (p_pos, p_vel)
        q:  process noise spectral density (acceleration^2)
        r:  measurement variance
        outlier_threshold: Mahalanobis distance gate (None to disable)
        vel_clip: optional absolute cap on |velocity| after predict
    """
    def __init__(self, x0, v0,
                 P0=(1.0, 1.0),
                 q=1.0,
                 r=0.25,
                 outlier_threshold=3.0,
                 vel_clip=None):
        self.x = np.array([[float(x0)], [float(v0)]], dtype=float)      # state
        if isinstance(P0, (list, tuple)) and len(P0) == 2:
            self.P = np.diag([float(P0[0]), float(P0[1])])
        else:
            self.P = np.array(P0, dtype=float)
            assert self.P.shape == (2, 2), "P0 must be 2x2 or (p_pos, p_vel)"
        self.q = float(q)
        self.r = float(r)
        self.H = np.array([[1.0, 0.0]])   # position-only measurement
        self.R = np.array([[self.r]])
        self.I = np.eye(2)
        self.outlier_threshold = outlier_threshold
        self.vel_clip = vel_clip
        self.last_dt = None
        self.last_z = None

    @staticmethod
    def _F(dt):
        return np.array([[1.0, dt],
                         [0.0, 1.0]])

    @staticmethod
    def _B(dt):
        # control matrix for constant acceleration input (a)
        return np.array([[0.5 * dt * dt],
                         [dt]])

    def _Q(self, dt):
        dt2 = dt * dt
        dt3 = dt2 * dt
        return self.q * np.array([[dt3 / 3.0, dt2 / 2.0],
                                  [dt2 / 2.0, dt]])

    def predict(self, dt, accel_u: float = 0.0):
        """
        Time update with optional acceleration input (units of m/s^2).
        """
        dt = float(dt)
        F = self._F(dt)
        B = self._B(dt)
        self.x = F @ self.x + B * float(accel_u)
        if self.vel_clip is not None:
            self.x[1, 0] = float(np.clip(self.x[1, 0], -self.vel_clip, self.vel_clip))
        self.P = F @ self.P @ F.T + self._Q(dt)
        self.last_dt = dt
        return float(self.x[0, 0])  # predicted position

    def update(self, z, run_smoothing=True):
        """
        Measurement update with scalar position z.
        If run_smoothing is False, we *skip* the correction and just return the
        predicted position (useful when you want to coast without using z).
        """
        self.last_z = float(z)

        if run_smoothing is False:
            return float(self.x[0, 0])  # keep the predicted state

        z_vec = np.array([[self.last_z]])
        y = z_vec - (self.H @ self.x)                          # innovation
        S = self.H @ self.P @ self.H.T + self.R                # innovation cov (1x1)
        K = (self.P @ self.H.T) / S                            # Kalman gain (2x1)

        # Outlier gating (Mahalanobis distance)
        if self.outlier_threshold is not None:
            maha = float(np.abs(y) / np.sqrt(S))
            if maha > self.outlier_threshold:
                # reject this update
                return float(self.x[0, 0])

        # Correct
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P
        return float(self.x[0, 0])

    # Convenience accessors
    def get_state(self):
        """Returns (position, velocity)."""
        return float(self.x[0, 0]), float(self.x[1, 0])

    def get_covariance(self):
        return self.P.copy()

    @property
    def position(self):
        return float(self.x[0, 0])

    @property
    def velocity(self):
        return float(self.x[1, 0])


class KalmanFilter1D:
    def __init__(
        self,
        initial_state,
        initial_uncertainty,
        process_variance,
        measurement_variance,
        outlier_threshold=3.0,
        dt: float = 1.0,          # <-- add a default time step
    ):
        self.x = float(initial_state)
        self.measurement = 0.0
        self.P_estimate_variance = float(initial_uncertainty)
        self.Q_process_variance = float(process_variance)
        self.R_measurement_variance = float(measurement_variance)
        self.outlier_threshold = outlier_threshold
        self.dt = float(dt)

    def predict(self, expected_velocity: float = 0.0, dt: float | None = None):
        """
        Advance the state using a simple constant-velocity model:
            x_pred = x + v*dt
            P_pred = P + Q*dt
        Pass a negative velocity to 'move backward'.
        """
        dt = self.dt if dt is None else float(dt)
        self.x += float(expected_velocity) * dt
        self.P_estimate_variance += self.Q_process_variance * dt
        return self.x

    def update(self, z, run_smoothing=True):
        self.measurement = float(z)
        if run_smoothing is False:
            return self.measurement

        # Innovation
        measurement_residual = self.measurement - self.x
        S_innovation_covariance = self.P_estimate_variance + self.R_measurement_variance
        K_kalman_gain = self.P_estimate_variance / S_innovation_covariance

        # Outlier check (Mahalanobis distance)
        mahalanobis_distance = abs(measurement_residual) / np.sqrt(S_innovation_covariance)
        if self.outlier_threshold is not None and mahalanobis_distance > self.outlier_threshold:
            # reject outlier, keep prior
            return self.x

        # Update
        self.x += K_kalman_gain * measurement_residual
        self.P_estimate_variance = (1 - K_kalman_gain) * self.P_estimate_variance
        return self.x

    def get_state(self):
        return self.x

    def get_uncertainty(self):
        return self.P_estimate_variance   # <-- was referencing undefined self.P
