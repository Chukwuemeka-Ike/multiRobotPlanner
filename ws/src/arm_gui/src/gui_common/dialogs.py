import time
import typing

# from PyQt5 import QtCore
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import *

from arm_constants.stations_old import *


class ImportTicketsDialog(QDialog):
    '''Class for importing multiple tickets at once.'''
    dataEntered = pyqtSignal(list)

    def __init__(self, parent: QWidget, min_ticket_number: int) -> None:
        super().__init__(parent)
        self.setWindowTitle("Import Tickets")

        self.dialogLayout = QGridLayout()
        self.minTicketNumber = min_ticket_number

        ticket_id_label = QLabel("Ticket ID")
        ticket_id_label.setAlignment(Qt.AlignCenter)
        parents_label = QLabel("Parents")
        parents_label.setAlignment(Qt.AlignCenter)
        duration_label = QLabel("Duration")
        duration_label.setAlignment(Qt.AlignCenter)
        machine_label = QLabel("Machine Type")
        machine_label.setAlignment(Qt.AlignCenter)

        self.dialogLayout.addWidget(ticket_id_label, 0, 0)
        self.dialogLayout.addWidget(parents_label, 0, 1)
        self.dialogLayout.addWidget(duration_label, 0, 2)
        self.dialogLayout.addWidget(machine_label, 0, 3)

        # Create the widget to hold the form layout
        # Create the form layout for tickets.
        self.formLayout = QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        self.dialogLayout.addLayout(self.formLayout, 1, 0, 1, 5)

        # Create the buttons.
        # Button to add a new ticket row.
        self.addButton = QPushButton("Add Ticket")
        self.addButton.clicked.connect(self.add_ticket)
        self.dialogLayout.addWidget(self.addButton, 2, 1, 1, 3)

        # Button to submit the tickets.
        self.submitButton = QPushButton("Submit Tickets")
        self.submitButton.clicked.connect(self.submit_tickets)
        self.dialogLayout.addWidget(self.submitButton, 3, 0, 1, 3)

        # Cancel button.
        self.cancelButton = QPushButton("Cancel")
        self.cancelButton.clicked.connect(self.close)
        self.dialogLayout.addWidget(self.cancelButton, 3, 3, 1, 2)

        self.resize(650, 100)
        # self.setFixedSize(650, 100)
        # self.setMinimumSize(650, 100)
        self.setLayout(self.dialogLayout)
        self.setSizeGripEnabled(False)
        
        self.setModal(True)
        self.show()
        self.square_width = self.dialogLayout.geometry().width()//5
        print(self.dialogLayout.geometry().width())
        print(self.square_width)

        # Add one ticket row for aesthetics.
        self.add_ticket()

    def add_ticket(self):
        # Create the input fields for each ticket row.
        ticket_id_label = QLabel(str(self.minTicketNumber))
        ticket_id_label.setAlignment(Qt.AlignCenter)
        ticket_id_label.setFixedWidth(self.square_width)
        parents_edit = QLineEdit()
        parents_edit.setFixedWidth(self.square_width)
        duration_edit = QLineEdit()
        duration_edit.setFixedWidth(self.square_width)
        station_type_combo = QComboBox()
        station_type_combo.addItems(station_type_names)
        station_type_combo.setFixedWidth(self.square_width)

        # Create a layout to hold the input fields and remove button.
        row_layout = QHBoxLayout()
        row_layout.addWidget(ticket_id_label)
        row_layout.addWidget(parents_edit)
        row_layout.addWidget(duration_edit)
        row_layout.addWidget(station_type_combo)

        # Add a button to remove the row if needed.
        remove_button = QPushButton("Remove")
        remove_button.setFixedWidth(self.square_width)
        remove_button.clicked.connect(lambda: self.remove_ticket(row_layout))
        row_layout.addWidget(remove_button)
        # row_layout.addStretch()

        # Add the input fields and remove button to the form layout.
        self.formLayout.addRow(row_layout)
        self.resize(self.width(), self.height() + 35)
        self.minTicketNumber += 1

    def remove_ticket(self, row_layout):
        '''Remove the ticket row from the form layout.'''
        self.formLayout.removeRow(row_layout)

    def submit_tickets(self):
        '''Submit all the tickets.'''
        ticket_count = self.formLayout.rowCount()
        entered_tickets = []
        print(f"{ticket_count} ticket rows submitted.")

        try:
            for row in range(ticket_count):
                layout = self.formLayout.itemAt(row)

                ticket_id = layout.itemAt(0).widget().text()
                parents = layout.itemAt(1).widget().text()
                duration = layout.itemAt(2).widget().text()
                station_name = layout.itemAt(3).widget().currentText()
                station_num = station_type_names.index(station_name)

                # Append the ticket row if it is valid.
                entered_tickets.append([ticket_id, station_num, parents, duration])

            self.dataEntered.emit(entered_tickets)
            message = f"Imported tickets successfully."
            QMessageBox.information(self, "Success", message)
            self.accept()
        except:
            # TODO: errors due to empty fields are not tolerable. Make
            # the user fix the problem.
            print("Incomplete ticket information")
            message = f"Incomplete ticket information. Please fix the missing fields."
            QMessageBox.information(self, "Import Failure", message)

    def resizeEvent(self, event):
        # Keep the dialog at its current position when resized.
        self.move(self.pos())

        # Call the base class method.
        super().resizeEvent(event)

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
            self.accept()
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
        self.setWindowTitle("Ticket Info")
        
        self.dialogLayout = QHBoxLayout()

        self.ticketInfoLayout = QVBoxLayout()
        self.ticketInfoLayout.addWidget(QLabel(f"Job ID: {1}"))
        self.ticketInfoLayout.addWidget(QLabel(f"Ticket ID: {4}"))
        self.ticketInfoLayout.addWidget(QLabel(f"Parents: [{2},{3}]"))
        self.ticketInfoLayout.addWidget(QLabel(f"Machine Type: RF Weld"))
        self.ticketInfoLayout.addStretch()

        self.jobDrawing = QLabel(self)
        # icon=QIcon()
        # icon.addPixmap(QPixmap(self.plus))
        dd = QPixmap("./dd.jpg")
        dd.scaled(dd.width()//5, dd.height()//5)
        self.jobDrawing.setPixmap(dd)
        # self.jobDrawing.resize(dd.width(), dd.height())

        self.dialogLayout.addLayout(self.ticketInfoLayout)
        self.dialogLayout.addWidget(self.jobDrawing)
        self.setLayout(self.dialogLayout)