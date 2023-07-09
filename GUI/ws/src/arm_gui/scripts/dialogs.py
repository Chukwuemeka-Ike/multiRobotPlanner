import time
from PyQt5 import QtCore
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
from constants.stations_old import *


class NewTicketDialog(QDialog):
    '''.'''
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent=parent)
        self.setWindowTitle("New Ticket")

        self.dialogLayout = QVBoxLayout()

        self.jobIDComboBox = QComboBox()
        self.jobIDComboBox.addItems(["1","2","3"])
        # jobIDComboBox.isEditable

        self.machineTypeComboBox = QComboBox()
        self.machineTypeComboBox.addItems(station_type_names)

        # Allow the user pick up to 3 parents. 
        # We don't envision a task having more than 2 parents.
        self.parentsLayout = QHBoxLayout()
        self.parentsComboBox1 = QComboBox()
        self.parentsComboBox1.addItems(["1","2","3","4","5","6","7"])
        self.parentsComboBox2 = QComboBox()
        self.parentsComboBox2.addItems(["1","2","3","4","5","6","7"])
        self.parentsComboBox3 = QComboBox()
        self.parentsComboBox3.addItems(["1","2","3","4","5","6","7"])
        self.parentsLayout.addWidget(self.parentsComboBox1)
        self.parentsLayout.addWidget(self.parentsComboBox2)
        self.parentsLayout.addWidget(self.parentsComboBox3)

        self.ticketID = str(15)
        self.duration = QLineEdit()

        self.formLayout = QFormLayout()
        self.formLayout.addRow("Job ID", self.jobIDComboBox)
        self.formLayout.addRow("Duration", self.duration)
        self.formLayout.addRow("Machine Type", self.machineTypeComboBox)
        self.formLayout.addRow("Parents", self.parentsLayout)
        self.formLayout.addRow("Ticket ID", QLabel(self.ticketID))
        self.dialogLayout.addLayout(self.formLayout)

        self.buttons = QDialogButtonBox()
        addButton = self.buttons.addButton("Add", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("Cancel", QDialogButtonBox.RejectRole)
        
        addButton.clicked.connect(self._addNewTicket)
        cancelButton.clicked.connect(self._cancelAddNewTicket)

        self.dialogLayout.addWidget(self.buttons)
        self.setLayout(self.dialogLayout)

    def _addNewTicket(self):
        '''Add the new ticket.'''
        try:
            message = f"\nJob ID: {self.jobIDComboBox.currentText()}\n" +\
                f"Ticket ID: {self.ticketID}\n" + \
                f"Duration: {self.duration.text()}\n"
            QMessageBox.information(self, "Success", message)
            print(message)
            self.close()
        except:
            message = f"Error adding ticket."
            QMessageBox.information(self, "Failed", message)
            print(message)

    def _cancelAddNewTicket(self):
        '''Cancel adding the new ticket.'''
        self.close()

    def _generateNewTicketID(self):
        '''.'''
        # Get the current list of ticket IDs.

        # Add 1 to the highest number.
        pass


class EditTicketDialog(QDialog):
    '''.'''
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent=parent)
        self.setWindowTitle("Edit Ticket")
        
        self.dialogLayout = QVBoxLayout()

        self.ticketIDBox = QComboBox()
        self.ticketIDBox.addItems(["1","2","3"])
        self.ticketIDBox.setEditable(False)

        self.formLayout = QFormLayout()
        self.formLayout.addRow("Ticket ID", self.ticketIDBox)
        self.dialogLayout.addLayout(self.formLayout)

        self.buttons = QDialogButtonBox()
        applyButton = self.buttons.addButton("Apply", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("Cancel", QDialogButtonBox.RejectRole)
        
        applyButton.clicked.connect(self._applyEdit)
        cancelButton.clicked.connect(self._cancelEditTicket)

        self.dialogLayout.addWidget(self.buttons)
        self.setLayout(self.dialogLayout)

    def _applyEdit(self):
        '''Apply the edits.'''
        try:
            message = f"Applying edit to ticket {self.ticketIDBox.currentText()}."
            QMessageBox.information(self, "Success", message)
            print(message)
            self.close()
        except:
            message = f"Error editing ticket {self.ticketIDBox.currentText()}."
            QMessageBox.information(self, "Failed", message)
            print(message)

    def _cancelEditTicket(self):
        '''Cancel the edits.'''
        self.close()


class RemoveTicketDialog(QDialog):
    '''.'''
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent=parent)
        self.setWindowTitle("Remove Ticket")
        
        self.dialogLayout = QVBoxLayout()

        self.ticketIDBox = QComboBox()
        self.ticketIDBox.addItems(["1","2","3"])
        self.ticketIDBox.setEditable(False)

        self.formLayout = QFormLayout()
        self.formLayout.addRow("Ticket ID", self.ticketIDBox)
        self.dialogLayout.addLayout(self.formLayout)

        self.buttons = QDialogButtonBox()
        removeButton = self.buttons.addButton("Remove", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("Cancel", QDialogButtonBox.RejectRole)

        removeButton.clicked.connect(self._removeTicket)
        cancelButton.clicked.connect(self._cancelRemoveTicket)

        self.dialogLayout.addWidget(self.buttons)
        self.setLayout(self.dialogLayout)

    def _removeTicket(self):
        '''Remove the specified ticket.'''
        try:
            message = f"Removing ticket {self.ticketIDBox.currentText()}."
            QMessageBox.information(self, "Success", message)
            print(message)
            self.close()
        except:
            message = f"Error removing ticket {self.ticketIDBox.currentText()}."
            QMessageBox.information(self, "Failed", message)
            print(message)

    def _cancelRemoveTicket(self):
        '''Cancel the remove operation.'''
        self.close()


class TicketInfoDialog(QDialog):
    '''.'''
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent=parent)
        self.setWindowTitle("Remove Ticket")
        
        self.dialogLayout = QHBoxLayout()

        self.ticketInfoLayout = QVBoxLayout()
        self.ticketInfoLayout.addWidget(QLabel(f"Job ID: {1}"))
        self.ticketInfoLayout.addWidget(QLabel(f"Ticket ID: {4}"))
        self.ticketInfoLayout.addWidget(QLabel(f"Parents: [{2},{3}]"))
        self.ticketInfoLayout.addWidget(QLabel(f"Machine Type: RF"))

        self.jobDrawing = QLabel()
        # icon=QIcon()
        # icon.addPixmap(QPixmap(self.plus))
        jobDrawing = QPixmap('dd.jpg')
        # jobDrawing.res
        self.jobDrawing.setPixmap(jobDrawing)

        self.dialogLayout.addLayout(self.ticketInfoLayout)
        self.dialogLayout.addWidget(self.jobDrawing)
        self.setLayout(self.dialogLayout)
