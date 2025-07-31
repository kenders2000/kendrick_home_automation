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
    return np.where(
        x < mu,
        np.exp(-0.5 * ((x - mu) / sigma_left) ** 2),
        np.exp(-0.5 * ((x - mu) / sigma_right) ** 2)
    )


class KalmanFilter1D:
    def __init__(self, initial_state, initial_uncertainty,
                 process_variance, measurement_variance,
                 outlier_threshold=3.0):
        """
        Args:
            initial_state: Initial estimate of the state
            initial_uncertainty: Initial estimate uncertainty
            process_variance: Variance of the process noise (Q)
            measurement_variance: Variance of the measurement noise (R)
            outlier_threshold: Number of standard deviations to tolerate before rejecting a measurement
        """
        self.x = initial_state
        self.P = initial_uncertainty
        self.Q = process_variance
        self.R = measurement_variance
        self.outlier_threshold = outlier_threshold

    def predict(self):
        # Prediction step for a constant system (no control input)
        self.P += self.Q
        return self.x

    def update(self, z):
        """
        Args:
            z: new measurement
        Returns:
            Updated state estimate, or None if outlier
        """
        # Innovation
        y = z - self.x
        S = self.P + self.R  # Innovation covariance
        K = self.P / S       # Kalman gain

        # Outlier detection (using Mahalanobis distance)
        mahalanobis_distance = abs(y) / np.sqrt(S)
        if mahalanobis_distance > self.outlier_threshold:
            print(f"Outlier rejected: z={z}, predicted={self.x}, distance={mahalanobis_distance:.2f}")
            return None

        # Update step
        self.x += K * y
        self.P = (1 - K) * self.P
        return self.x

    def get_state(self):
        return self.x

    def get_uncertainty(self):
        return self.P
    

# kf = KalmanFilter1D(
#     initial_state=0.0,
#     initial_uncertainty=1.0,
#     process_variance=0.1,
#     measurement_variance=1.0,
#     outlier_threshold=3.0
# )

# measurements = [1.0, 1.1, 5.0, 1.2, 1.3]  # 5.0 is an outlier
# for z in measurements:
#     kf.predict()
#     state = kf.update(z)
#     if state is not None:
#         print(f"Updated state: {state:.2f}")
#     else:
#         print("Measurement rejected")
