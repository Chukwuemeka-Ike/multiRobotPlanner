import time
import typing

# from PyQt5 import QtCore
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator, QPixmap
from PyQt5.QtWidgets import *

from arm_constants.stations import *
from gui_common.gui_elements import FixedWidthLineEdit, FixedWidthLabel


class BasicDialog(QDialog):
    '''Template class for common dialog setups.'''
    dataEntered = pyqtSignal(list)
    
    def __init__(self, parent: QWidget) -> None:
        '''.'''
        super().__init__(parent)

        # self.dialogLayout = QVBoxLayout()
        # self.setLayout(self.dialogLayout)

        # Validator for ensuring line edits are kept as floats.
        self.double_validator = QDoubleValidator()

        self.setSizeGripEnabled(False)
        self.center_on_parent()
        self.setModal(True)
        self.show()

    def center_on_parent(self):
        # Get the screen geometry.
        parent = self.parent()

        if parent is not None:
            parent_rect = parent.geometry()
            self_rect = self.geometry()

            x = parent_rect.x() + (parent_rect.width() - self_rect.width()) // 2
            y = parent_rect.y() + (parent_rect.height() - self_rect.height()) // 2

            self.move(x, y)
    
    def resizeEvent(self, event):
        # Keep the dialog at its current position when resized.
        self.center_on_parent()

        # Call the base class method.
        super().resizeEvent(event)


class ImportTicketsDialog(BasicDialog):
    '''Class for importing multiple tickets at once.'''
    def __init__(self, parent: QWidget, min_ticket_number: int) -> None:
        super().__init__(parent)
        self.setWindowTitle("Bulk Add Tickets")

        self.minTicketNumber = min_ticket_number

        # This particular dialog uses a QGridLayout where the others
        # all use a QVBoxLayout, so delete and create a new one.
        self.dialogLayout = QGridLayout()
        self.setLayout(self.dialogLayout)
        
        # Height to add when adding a ticket row.
        self.row_increment = 25

        self.resize(650, 100)
        self.square_width = self.dialogLayout.geometry().width()//5
        self.create_ui()

        # Add one ticket row for aesthetics.
        self.add_ticket_row()

    def create_ui(self):
        '''Create the UI.'''
        ticket_id_label = FixedWidthLabel("Ticket ID", self.square_width)
        parents_label = FixedWidthLabel("Parents", self.square_width)
        duration_label = FixedWidthLabel("Duration", self.square_width)
        machine_label = FixedWidthLabel("Machine Type", self.square_width)

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
        self.addButton = QPushButton("Add Row")
        self.addButton.clicked.connect(self.add_ticket_row)
        self.dialogLayout.addWidget(self.addButton, 2, 1, 1, 3)

        # Button to submit the tickets.
        self.submitButton = QPushButton("Submit Tickets")
        self.submitButton.clicked.connect(self.submit_tickets)
        self.dialogLayout.addWidget(self.submitButton, 3, 0, 1, 3)

        # Cancel button.
        self.cancelButton = QPushButton("Cancel")
        self.cancelButton.clicked.connect(self.close)
        self.dialogLayout.addWidget(self.cancelButton, 3, 3, 1, 2)

    def add_ticket_row(self):
        # Create the input fields for each ticket row.
        ticket_id_label = FixedWidthLabel(str(self.minTicketNumber), self.square_width)

        parents_edit = FixedWidthLineEdit(self.square_width)
        parents_edit.setPlaceholderText("Enter parents separated by commas")
        parents_edit.setToolTip("Enter parents separated by commas")

        # Duration can only be floats. No alphabets.
        duration_edit = FixedWidthLineEdit(self.square_width)
        duration_edit.setValidator(self.double_validator)

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
        self.resize(self.width(), self.height() + self.row_increment)
        self.minTicketNumber += 1

    def remove_ticket(self, row_layout):
        '''Remove the ticket row from the form layout.'''
        self.formLayout.removeRow(row_layout)
        self.resize(self.width(), self.height() - self.row_increment)

    def submit_tickets(self):
        '''Submit all the tickets.'''
        ticket_count = self.formLayout.rowCount()
        entered_tickets = []

        if ticket_count == 0:
            message = f"No tickets imported."
            QMessageBox.information(self, "Cancelled", message)
            self.close()
            return

        try:
            for row in range(ticket_count):
                layout = self.formLayout.itemAt(row)

                ticket_id = int(layout.itemAt(0).widget().text())
                
                parents = layout.itemAt(1).widget().text()
                parents_string_list = parents.split(',')
                parents_list = []
                for a in parents_string_list:
                    if a != "":
                        parents_list.append(int(a))

                # If any duration is empty, raise an error, which will cause
                # the incomplete info message.
                duration = layout.itemAt(2).widget().text()
                if len(duration) == 0:
                    raise ValueError("No duration specified.")
                else:
                    duration = float(duration)
                station_name = layout.itemAt(3).widget().currentText()
                station_num = station_type_names.index(station_name)

                # Append the ticket row if it is valid.
                entered_tickets.append([ticket_id, parents_list, duration, station_num])

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


class NewTicketDialog(BasicDialog):
    '''Class for entering a single new ticket.'''
    def __init__(self, parent: QWidget, min_ticket_number: int, job_info: dict) -> None:
        '''.
        
        Args:
            job_info: {job_id: [ticket_id, ...], ...}
        '''
        super().__init__(parent)
        self.setWindowTitle("New Ticket")

        self.minTicketNumber = min_ticket_number
        self.jobInfo = job_info

        self.dialogLayout = QVBoxLayout()
        self.setLayout(self.dialogLayout)

        self.create_ui()

    def create_ui(self):
        '''Create the ui.'''

        # For picking an existing job ID.
        # If left on default, there will be no parents allowed.
        self.jobIDComboBox = QComboBox()
        self.jobIDComboBox.addItem("Select Job ID")

        # Convert to list of strings for the function using list comprehension.
        self.jobIDComboBox.addItems([str(key) for key in self.jobInfo.keys()])
        self.jobIDComboBox.currentIndexChanged.connect(self.change_allowed_parents)

        self.ticketID = str(self.minTicketNumber)

        # Allow the user pick up to 2 parents. 
        # We don't envision a task having more than 2 parents.
        self.parentsLayout = QHBoxLayout()
        self.parentsComboBox1 = QComboBox()
        self.parentsComboBox2 = QComboBox()
        self.parentsLayout.addWidget(self.parentsComboBox1)
        self.parentsLayout.addWidget(self.parentsComboBox2)

        # Duration can only be floats. No alphabets.
        self.duration = QLineEdit()
        self.duration.setValidator(self.double_validator)

        self.machineTypeComboBox = QComboBox()
        self.machineTypeComboBox.addItems(station_type_names)

        self.formLayout = QFormLayout()
        self.formLayout.addRow("Job ID", self.jobIDComboBox)
        self.formLayout.addRow("Ticket ID", QLabel(self.ticketID))
        self.formLayout.addRow("Parents", self.parentsLayout)
        self.formLayout.addRow("Duration", self.duration)
        self.formLayout.addRow("Machine Type", self.machineTypeComboBox)

        self.dialogLayout.addLayout(self.formLayout)

        # Add and Cancel buttons.
        self.buttons = QDialogButtonBox()
        addButton = self.buttons.addButton("Add", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("Cancel", QDialogButtonBox.RejectRole)
        
        addButton.clicked.connect(self.submit_ticket)
        cancelButton.clicked.connect(self.close)

        self.dialogLayout.addWidget(self.buttons)

    def change_allowed_parents(self):
        '''Change the parents combo boxes based on the selected job ID.

        The choices for ticket parents are narrowed by the job ID chosen.
        Helps to keep the drop-down list small.
        '''
        self.parentsComboBox1.clear()
        self.parentsComboBox2.clear()

        # If no selected job ID, there are no parent options.
        if self.jobIDComboBox.currentText() == "Select Job ID":
            self.parentsComboBox1.setEnabled(False)
            self.parentsComboBox2.setEnabled(False)
        else:
            self.parentsComboBox1.setEnabled(True)
            self.parentsComboBox2.setEnabled(True)

            self.parentsComboBox1.addItem("")
            self.parentsComboBox2.addItem("")

            # Convert both to lists of strings using list comprehension.
            self.parentsComboBox1.addItems(
                [str(a) for a in self.jobInfo[int(self.jobIDComboBox.currentText())] ]
            )
            self.parentsComboBox2.addItems(
                [str(a) for a in self.jobInfo[int(self.jobIDComboBox.currentText())] ]
            )

    def submit_ticket(self):
        '''Submit the new ticket.'''
        if self.duration.text() == "":
            message = f"Incomplete ticket information. Please fix the missing fields."
            print("Incomplete ticket information")
            QMessageBox.information(self, "Add Failed", message)
            return
        try:
            ticket_id = int(self.ticketID)
            parents = []
            if self.parentsComboBox1.currentText() != "":
                parents.append(int(self.parentsComboBox1.currentText()))
            if self.parentsComboBox2.currentText() != "":
                parents.append(int(self.parentsComboBox2.currentText()))
            duration = float(self.duration.text())
            station_name = self.machineTypeComboBox.currentText()
            station_num = station_type_names.index(station_name)

            self.dataEntered.emit([ticket_id, parents, duration, station_num])
            message = "Added ticket successfully."
            QMessageBox.information(self, "Success", message)
            print(message)
            self.accept()
        except:
            message = f"Error adding ticket."
            QMessageBox.information(self, "Failed", message)
            print(message)


class EditTicketDialog(BasicDialog):
    '''.'''
    def __init__(self, parent: QWidget, job_info: dict, all_tickets: dict, ongoing: dict) -> None:
        '''.
        
        Args:
            job_info: {job_id: [ticket_id, ...], ...}
        '''
        super().__init__(parent)
        self.setWindowTitle("Edit Ticket")

        self.jobInfo = job_info
        self.ongoing = ongoing
        self.allTickets = all_tickets
        
        self.dialogLayout = QVBoxLayout()
        self.setLayout(self.dialogLayout)
        self.create_ui()

    def create_ui(self):
        '''Create the UI.'''

        # For picking an existing job ID.
        # If left on default, there will be no ticket to pick.
        self.jobIDComboBox = QComboBox()
        self.jobIDComboBox.addItem("Select Job ID")

        # Convert to list of strings for the function using list comprehension.
        self.jobIDComboBox.addItems([str(key) for key in self.jobInfo.keys()])
        self.jobIDComboBox.currentIndexChanged.connect(self.change_allowed_ticket_id)

        self.ticketIDBox = QComboBox()
        self.ticketIDBox.currentIndexChanged.connect(self.fill_in_ticket_info)

        # Duration can only be floats. No alphabets.
        self.duration = QLineEdit()
        self.duration.setValidator(self.double_validator)

        self.machineTypeComboBox = QComboBox()
        self.machineTypeComboBox.addItems(station_type_names)

        self.formLayout = QFormLayout()
        self.formLayout.addRow("Job ID", self.jobIDComboBox)
        self.formLayout.addRow("Ticket ID", self.ticketIDBox)
        self.formLayout.addRow("Duration", self.duration)
        self.formLayout.addRow("Machine Type", self.machineTypeComboBox)
        self.dialogLayout.addLayout(self.formLayout)

        self.buttons = QDialogButtonBox()
        applyButton = self.buttons.addButton("Apply", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("Cancel", QDialogButtonBox.RejectRole)
        
        applyButton.clicked.connect(self.submit_edits)
        cancelButton.clicked.connect(self.close)

        self.dialogLayout.addWidget(self.buttons)

    def change_allowed_ticket_id(self):
        '''Change the ticket ID combo box based on the selected job ID.

        The choices for ticket IDs are narrowed by the job ID chosen.
        Helps to keep the drop-down list small.
        '''
        self.ticketIDBox.clear()

        # If no selected job ID, there are no ticket ID options.
        if self.jobIDComboBox.currentText() == "Select Job ID":
            self.ticketIDBox.setEnabled(False)
        else:
            self.ticketIDBox.setEnabled(True)

            # Convert to list of strings using list comprehension.
            self.ticketIDBox.addItems(
                [str(a) for a in self.jobInfo[int(self.jobIDComboBox.currentText())] ]
            )

    def fill_in_ticket_info(self):
        '''Fills in the .'''
        ticket_id = int(self.ticketIDBox.currentText())
        duration = str(self.allTickets[ticket_id]["duration"])
        station_num = self.allTickets[ticket_id]["station_type"]

        self.duration.setText(duration)
        self.machineTypeComboBox.setCurrentIndex(station_num)

    def submit_edits(self):
        '''Apply the edits.'''
        if self.duration.text() == "":
            message = f"Please fill in the duration."
            print("Incomplete ticket information")
            QMessageBox.information(self, "Add Failed", message)
            return
        try:
            ticket_id = int(self.ticketIDBox.currentText())
            duration = float(self.duration.text())
            station_name = self.machineTypeComboBox.currentText()
            station_num = station_type_names.index(station_name)
            
            # If the ticket is ongoing, double check with the user that they
            # still want to edit it.
            complete_edit = True
            if ticket_id in self.ongoing:
                result = self.openPopupDialog(ticket_id)
                if result == QDialog.Accepted:
                    complete_edit = True
                else:
                    complete_edit = False

            if complete_edit == True:
                self.dataEntered.emit([ticket_id, duration, station_num])
                message = f"Edited ticket {self.ticketIDBox.currentText()} successfully."
                QMessageBox.information(self, "Success", message)
                print(message)
                self.accept()
        except:
            message = f"Error editing ticket {self.ticketIDBox.currentText()}."
            QMessageBox.information(self, "Failed", message)
            print(message)

    def openPopupDialog(self, ticket_id):
        popup_dialog = ConfirmEditOngoingTicketDialog(self, ticket_id)
        print("here")
        x_offset = (self.width() - popup_dialog.width()) // 2
        y_offset = (self.height() - popup_dialog.height()) // 2
        popup_dialog.move(self.geometry().x() + x_offset, self.geometry().y() + y_offset)
        result = popup_dialog.exec_()
        return result        


class ConfirmEditOngoingTicketDialog(QDialog):
    def __init__(self, parent: QWidget, ticket_number: int):
        super().__init__(parent)

        self.setWindowTitle("Confirm Edit")
        self.setGeometry(0, 0, 200, 100)

        self.dialogLayout = QVBoxLayout()
        self.setLayout(self.dialogLayout)

        print("here D")
        self.label = QLabel(f"Ticket {ticket_number} is currently ongoing. Edit?")
        print("here b")
        self.buttons = QDialogButtonBox()
        applyButton = self.buttons.addButton("Apply", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("Cancel", QDialogButtonBox.RejectRole)
        
        applyButton.clicked.connect(self.accept)
        cancelButton.clicked.connect(self.close)

        self.dialogLayout.addWidget(self.label)
        self.dialogLayout.addWidget(self.buttons)
        print("here A")

        self.setModal(True)
        self.show()


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

# class Edit

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