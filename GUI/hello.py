import sys

# from PyQt6.QtWidgets import QApplication, QLabel, QWidget
from PyQt5.QtWidgets import QApplication, QLabel, QWidget

# Create an instance of QApplication. Create before creating any GUI object.
app = QApplication([])

window = QWidget()
window.setWindowTitle("Hello, World!")
window.setGeometry(100, 100, 280, 80)
helloMsg = QLabel("<h1>Hello, World!</h1>", parent=window)
helloMsg.move(60, 15)

window.show()

sys.exit(app.exec())