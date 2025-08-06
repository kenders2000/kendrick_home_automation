import threading
import sys
from PySide6.QtWidgets import QApplication
import uvicorn

from home_cinema_control import app, controller  # your real controller instance
from distance_gui import DistanceTestUI

def start_uvicorn():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    # Start FastAPI in the background
    threading.Thread(target=start_uvicorn, daemon=True).start()

    # Start GUI in the main thread
    app = QApplication(sys.argv)
    window = DistanceTestUI(controller)
    window.show()
    sys.exit(app.exec())