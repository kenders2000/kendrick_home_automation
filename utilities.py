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


class KalmanFilter1D:
    def __init__(
        self,
        initial_state,
        initial_uncertainty,
        process_variance,
        measurement_variance,
        outlier_threshold=3.0,
    ):
        """
        A Kalman Filter is a recursive algorithm that estimates the state of a dynamic system
        from a series of noisy measurements. It's often used in tracking, robotics, signal smoothing, etc.

        Goal: Estimate a hidden value (e.g., position, distance) over time
        Problem: Each measurement is noisy; we don’t want to trust any one point
        Solution: Combine past state and new measurement based on uncertainty

        Predict Step:
            P = P + Q                # Increase uncertainty over time

        Update Step:
            y = z - x                # Difference between estimate and measurement
            S = P + R                # Total uncertainty
            K = P / (P + R)          # How much to trust the measurement

            if not outlier:
                x = x + K * y        # Adjust estimate
                P = (1 - K) * P      # Reduce uncertainty

        Args:
            initial_state: Initial estimate position (e.g., distance)
            initial_uncertainty: Initial estimate uncertainty, how much we trust the initial position
            process_variance: Variance of the process noise (Q)
            measurement_variance: Variance of the measurement noise (R)
            outlier_threshold: Number of standard deviations to tolerate before rejecting a measurement
        """
        self.x = initial_state
        self.P_estimate_variance = initial_uncertainty
        self.Q_process_variance = process_variance
        self.R_measurement_variance = measurement_variance
        self.outlier_threshold = outlier_threshold

    def predict(self):
        """
        Each time you call predict(), the filter assumes the system might have changed a little due to process noise 
        ande the passage of time.
        (but doesn’t apply motion directly)

        """
        # Prediction step for a constant system (no control input)
        # increase the measurement uncertainty a little as this is a function of the process noise and we have just passed time
        self.P_estimate_variance += self.Q_process_variance
        return self.x

    def update(self, z):
        """
        Args:
            z: new measurement
        Returns:
            Updated state estimate, or None if outlier
        """
        # Innovation
        measurement_residual = z - self.x              # Measurement residual (difference between actual measurement and predicted state)
        S_innovation_covariance = self.P_estimate_variance + self.R_measurement_variance  # Innovation covariance
        K_kalman_gain = self.P_estimate_variance / S_innovation_covariance  # Kalman gain
        # The Kalman Gain is the weight the filter gives to the new measurement (z) compared to the current internal estimate (x).

        # Outlier detection (using Mahalanobis distance)
        mahalanobis_distance = abs(measurement_residual) / np.sqrt(S_innovation_covariance)
        if mahalanobis_distance > self.outlier_threshold:
            print(
                f"Outlier rejected: z={z}, predicted={self.x}, distance={mahalanobis_distance:.2f}"
            )
            return self.x

        # Update step
        # If measurement_residual is small and P_estimate_variance is low, the update is small.
        self.x += K_kalman_gain * measurement_residual                              # move estimate toward measurement
        self.P_estimate_variance = (1 - K_kalman_gain) * self.P_estimate_variance  # reduce uncertainty after measurement
        return self.x

    def get_state(self):
        return self.x

    def get_uncertainty(self):
        return self.P

