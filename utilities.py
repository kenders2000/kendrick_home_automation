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

