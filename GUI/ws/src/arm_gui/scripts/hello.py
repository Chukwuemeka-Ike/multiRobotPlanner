'''hello.py

Hello, World! example with PyQt5.
'''
import sys

from PyQt5.QtWidgets import *

# Create an instance of QApplication. Create before creating any GUI object.
app = QApplication([])

window = QWidget()
window.setWindowTitle("Hello, World!")
# window.setGeometry(100, 100, 280, 80)

helloMsg = QLabel("<h1>Hello, World!</h1>", parent=window)
# helloMsg.move(60, 15)

jobIDBox = QComboBox()
# jobIDBox.addItem("")
jobIDBox.addItems(["1","2","3"])
jobIDBox.setEditable(True)

formLayout = QFormLayout()
formLayout.addRow("Job ID", jobIDBox)

layout = QVBoxLayout()
layout.addWidget(helloMsg)
layout.addLayout(formLayout)

window.setLayout(layout)
window.show()

sys.exit(app.exec())