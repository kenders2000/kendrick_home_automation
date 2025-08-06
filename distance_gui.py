import sys
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton, QDoubleSpinBox
from PySide6.QtCore import Qt, QTimer

# Replace this with your actual controller instance
from home_cinema_control import controller  # ← adjust import as needed

class DistanceTestUI(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.setWindowTitle("Manual Distance Slider")

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.label = QLabel("Distance: 2.00 m")
        self.label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(600)  # Represents 0.00m to 6.00m
        self.slider.setValue(200)    # Start at 2.00m
        self.slider.valueChanged.connect(self.update_label)
        self.layout.addWidget(self.slider)

        # Live update every 100 ms
        self.timer = QTimer()
        self.timer.timeout.connect(self.send_distance_to_controller)
        self.timer.start(100)

    def update_label(self):
        distance = self.slider.value() / 100.0
        self.label.setText(f"Distance: {distance:.2f} m")

    def send_distance_to_controller(self):
        distance = self.slider.value() / 100.0
        self.controller.set_manual_distance(distance)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = DistanceTestUI(controller)
    window.show()
    sys.exit(app.exec())
