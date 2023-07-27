#!/usr/bin/env python3
'''
Rensselaer Polytechnic Institute - Julius Lab
ARM Project
Author - Chukwuemeka Osaretin Ike

Description:
    Dialog classes for the GUIs to use.
'''
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QDoubleValidator, QPixmap
from PyQt5.QtWidgets import *

from arm_constants.machines import *
from gui_common.gui_elements import FixedWidthLineEdit, FixedWidthLabel


class BasicDialog(QDialog):
    '''Template class for common dialog setups.'''
    dataEntered = pyqtSignal(list)
    
    def __init__(self, parent: QWidget) -> None:
        '''.'''
        super().__init__(parent)

        # Validator for ensuring line edits are kept as floats.
        self.double_validator = QDoubleValidator()

        self.setSizeGripEnabled(False)
        self.setModal(True)

    def center_on_parent(self):
        '''Centers the dialog on its parent.'''
        # Get the parent's geometry.
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
    
    def showEvent(self, event):
        '''Called when the dialog is about to be shown.'''
        self.center_on_parent()
        super().showEvent(event)

class BasicConfirmDialog(BasicDialog):
    '''Confirmation sub-dialog for ensuring the user wants to do something.'''
    def __init__(self, parent: QWidget, window_title: str, confirm_text: str):
        super().__init__(parent)

        self.setWindowTitle(window_title)
        self.setGeometry(0, 0, 200, 100)

        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.label = QLabel(confirm_text)

        self.buttons = QDialogButtonBox()
        applyButton = self.buttons.addButton("Yes", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("No", QDialogButtonBox.RejectRole)
        
        applyButton.clicked.connect(self.accept)
        cancelButton.clicked.connect(self.close)

        self.mainLayout.addWidget(self.label)
        self.mainLayout.addWidget(self.buttons)

        self.show()

class ImportTicketsDialog(BasicDialog):
    '''Class for importing multiple tickets at once.'''
    def __init__(self, parent: QWidget, min_ticket_number: int) -> None:
        super().__init__(parent)
        self.setWindowTitle("Bulk Add Tickets")

        self.minTicketNumber = min_ticket_number

        # This particular dialog uses a QGridLayout where the others
        # all use a QVBoxLayout, so delete and create a new one.
        self.mainLayout = QGridLayout()
        self.setLayout(self.mainLayout)
        
        # Height to add when adding a ticket row.
        self.row_increment = 25

        self.resize(650, 100)
        self.show()
        self.square_width = self.mainLayout.geometry().width()//5
        self.create_ui()

        # Add one ticket row for aesthetics.
        self.add_ticket_row()

    def create_ui(self):
        '''Create the UI.'''
        ticket_id_label = FixedWidthLabel("Ticket ID", self.square_width)
        parents_label = FixedWidthLabel("Parents", self.square_width)
        duration_label = FixedWidthLabel("Duration", self.square_width)
        machine_label = FixedWidthLabel("Machine Type", self.square_width)

        self.mainLayout.addWidget(ticket_id_label, 0, 0)
        self.mainLayout.addWidget(parents_label, 0, 1)
        self.mainLayout.addWidget(duration_label, 0, 2)
        self.mainLayout.addWidget(machine_label, 0, 3)

        # Create the widget to hold the form layout
        # Create the form layout for tickets.
        self.formLayout = QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        self.mainLayout.addLayout(self.formLayout, 1, 0, 1, 5)

        # Create the buttons.
        # Button to add a new ticket row.
        self.addButton = QPushButton("Add Row")
        self.addButton.clicked.connect(self.add_ticket_row)
        self.mainLayout.addWidget(self.addButton, 2, 1, 1, 3)

        # Button to submit the tickets.
        self.submitButton = QPushButton("Submit Tickets")
        self.submitButton.clicked.connect(self.submit_tickets)
        self.mainLayout.addWidget(self.submitButton, 3, 0, 1, 3)

        # Cancel button.
        self.cancelButton = QPushButton("Cancel")
        self.cancelButton.clicked.connect(self.close)
        self.mainLayout.addWidget(self.cancelButton, 3, 3, 1, 2)

    def add_ticket_row(self):
        # Create the input fields for each ticket row.
        ticket_id_label = FixedWidthLabel(str(self.minTicketNumber), self.square_width)

        parents_edit = FixedWidthLineEdit(self.square_width)
        parents_edit.setPlaceholderText("Enter parents separated by commas")
        parents_edit.setToolTip("Enter parents separated by commas")

        # Duration can only be floats. No alphabets.
        duration_edit = FixedWidthLineEdit(self.square_width)
        duration_edit.setValidator(self.double_validator)

        machine_type_combo = QComboBox()
        machine_type_combo.addItems(machine_type_names)
        machine_type_combo.setFixedWidth(self.square_width)

        # Create a layout to hold the input fields and remove button.
        row_layout = QHBoxLayout()
        row_layout.addWidget(ticket_id_label)
        row_layout.addWidget(parents_edit)
        row_layout.addWidget(duration_edit)
        row_layout.addWidget(machine_type_combo)

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
                machine_name = layout.itemAt(3).widget().currentText()
                machine_type_num = machine_type_names.index(machine_name)

                # Append the ticket row if it is valid.
                entered_tickets.append([ticket_id, parents_list, duration, machine_type_num])

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
        self.job_ticket_ids = job_info

        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.create_ui()
        self.show()

    def create_ui(self):
        '''Create the ui.'''

        # For picking an existing job ID.
        # If left on default, there will be no parents allowed.
        self.jobIDComboBox = QComboBox()
        self.jobIDComboBox.addItem("Select Job ID")

        # Convert to list of strings for the function using list comprehension.
        self.jobIDComboBox.addItems([str(key) for key in self.job_ticket_ids.keys()])
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
        self.machineTypeComboBox.addItems(machine_type_names)

        self.formLayout = QFormLayout()
        self.formLayout.addRow("Job ID", self.jobIDComboBox)
        self.formLayout.addRow("Ticket ID", QLabel(self.ticketID))
        self.formLayout.addRow("Parents", self.parentsLayout)
        self.formLayout.addRow("Duration", self.duration)
        self.formLayout.addRow("Machine Type", self.machineTypeComboBox)

        self.mainLayout.addLayout(self.formLayout)

        # Add and Cancel buttons.
        self.buttons = QDialogButtonBox()
        addButton = self.buttons.addButton("Add", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("Cancel", QDialogButtonBox.RejectRole)
        
        addButton.clicked.connect(self.submit_ticket)
        cancelButton.clicked.connect(self.close)

        self.mainLayout.addWidget(self.buttons)

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
                [str(a) for a in self.job_ticket_ids[int(self.jobIDComboBox.currentText())] ]
            )
            self.parentsComboBox2.addItems(
                [str(a) for a in self.job_ticket_ids[int(self.jobIDComboBox.currentText())] ]
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
            machine_name = self.machineTypeComboBox.currentText()
            machine_type_num = machine_type_names.index(machine_name)

            self.dataEntered.emit([ticket_id, parents, duration, machine_type_num])
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
    def __init__(self, parent: QWidget, job_info: dict, all_tickets: dict, ongoing: list) -> None:
        '''.
        
        Args:
            job_info: {job_id: [ticket_id, ...], ...}
        '''
        super().__init__(parent)
        self.setWindowTitle("Edit Ticket")

        self.job_ticket_ids = job_info
        self.ongoing = ongoing
        self.all_tickets = all_tickets
        
        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)
        self.create_ui()
        self.show()

    def create_ui(self):
        '''Create the UI.'''

        # For picking an existing job ID.
        # If left on default, there will be no ticket to pick.
        self.jobIDComboBox = QComboBox()
        self.jobIDComboBox.addItem("Select Job ID")

        # Convert to list of strings for the function using list comprehension.
        self.jobIDComboBox.addItems([str(key) for key in self.job_ticket_ids.keys()])
        self.jobIDComboBox.currentIndexChanged.connect(self.change_allowed_ticket_id)

        self.ticketIDBox = QComboBox()
        self.ticketIDBox.currentIndexChanged.connect(self.fill_in_ticket_info)

        # Duration can only be floats. No alphabets.
        self.duration = QLineEdit()
        self.duration.setValidator(self.double_validator)

        self.machineTypeComboBox = QComboBox()
        self.machineTypeComboBox.addItems(machine_type_names)

        self.formLayout = QFormLayout()
        self.formLayout.addRow("Job ID", self.jobIDComboBox)
        self.formLayout.addRow("Ticket ID", self.ticketIDBox)
        self.formLayout.addRow("Duration", self.duration)
        self.formLayout.addRow("Machine Type", self.machineTypeComboBox)
        self.mainLayout.addLayout(self.formLayout)

        self.buttons = QDialogButtonBox()
        applyButton = self.buttons.addButton("Apply", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("Cancel", QDialogButtonBox.RejectRole)
        
        applyButton.clicked.connect(self.submit_edits)
        cancelButton.clicked.connect(self.close)

        self.mainLayout.addWidget(self.buttons)

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
                [str(a) for a in self.job_ticket_ids[int(self.jobIDComboBox.currentText())] ]
            )

    def fill_in_ticket_info(self):
        '''Fills in the information for the currently selected ticket.'''
        print(self.ticketIDBox.currentText())
        ticket_id = int(self.ticketIDBox.currentText())
        duration = str(self.all_tickets[ticket_id]["duration"])
        machine_type_num = self.all_tickets[ticket_id]["machine_type"]

        self.duration.setText(duration)
        self.machineTypeComboBox.setCurrentIndex(machine_type_num)

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
            machine_name = self.machineTypeComboBox.currentText()
            machine_type_num = machine_type_names.index(machine_name)
            
            # If the ticket is ongoing, double check with the user that they
            # still want to edit it.
            confirm_edit = True
            if ticket_id in self.ongoing:
                result = self.openPopupDialog(ticket_id)
                if result == QDialog.Accepted:
                    confirm_edit = True
                else:
                    confirm_edit = False

            if confirm_edit == True:
                self.dataEntered.emit([ticket_id, duration, machine_type_num])
                message = f"Edited ticket {self.ticketIDBox.currentText()} successfully."
                QMessageBox.information(self, "Success", message)
                print(message)
                self.accept()
        except:
            message = f"Error editing ticket {self.ticketIDBox.currentText()}."
            QMessageBox.information(self, "Failed", message)
            print(message)

    def openPopupDialog(self, ticket_id):
        '''Open the popup confirming ticket edit.'''
        popup_dialog = BasicConfirmDialog(
            self,
            "Confirm Ticket Edit",
            f"Ticket {ticket_id} is currently ongoing. Edit?"
        )
        result = popup_dialog.exec_()
        return result

class EditJobDialog(BasicDialog):
    '''Dialog for editing an entire job at a time.'''
    deleteJobID = pyqtSignal(int)

    def __init__(
            self, parent: QWidget, job_info: dict, all_tickets: dict,
            ongoing: list
    ) -> None:
        '''Constructor that creates the UI and sets member variables.
        
        Args:
            job_info: {job_id: [ticket_id, ...], ...}
        '''
        super().__init__(parent)
        self.setWindowTitle("Job Editor")

        self.job_ticket_ids = job_info
        self.all_tickets = all_tickets
        self.ongoing = ongoing

        self.mainLayout = QGridLayout()
        self.setLayout(self.mainLayout)

        self.setFixedSize(750, 400)
        self.show()
        self.square_width = self.layout().geometry().width()//7
        self.create_ui()

    def create_ui(self):
        '''Create the ui.'''

        # For picking an existing job ID.
        # If left on default, there will be no tickets shown.
        self.jobIDLabel = FixedWidthLabel("Job ID", self.square_width)
        self.jobIDComboBox = QComboBox()
        self.jobIDComboBox.addItem("Select Job ID")

        # Convert to list of strings for the function using list comprehension.
        self.jobIDComboBox.addItems([str(key) for key in self.job_ticket_ids.keys()])
        self.jobIDComboBox.currentIndexChanged.connect(self.update_ticket_list)
        
        self.mainLayout.addWidget(self.jobIDLabel, 0, 0)
        self.mainLayout.addWidget(self.jobIDComboBox, 0, 1, 1, 2)

        # Delete job button.
        self.deleteJobButton = QPushButton("Delete Job")
        self.deleteJobButton.clicked.connect(self.delete_job)
        self.deleteJobButton.setEnabled(False)
        self.deleteJobButton.setStyleSheet("background-color : lightGrey")
        self.mainLayout.addWidget(self.deleteJobButton, 0, 4, 1, 2)
        
        # Create the layout for the title.
        self.titleLayout = QHBoxLayout()
        self.titleLayout.addWidget(FixedWidthLabel("Ticket", self.square_width))
        self.titleLayout.addWidget(FixedWidthLabel("Parents", self.square_width))
        self.titleLayout.addWidget(FixedWidthLabel("Duration", self.square_width))
        self.titleLayout.addWidget(FixedWidthLabel("Machine Type", self.square_width))
        self.titleLayout.addWidget(FixedWidthLabel("Status", self.square_width))
        self.titleLayout.addWidget(FixedWidthLabel("Remove?", self.square_width))

        # self.ticketListLayout.addLayout(self.titleLayout)
        self.mainLayout.addLayout(self.titleLayout, 1, 0, 1, 6)

        # Create the job list display.
        # Create a layout to hold the scroll area and the scroll widget.
        self.jobScrollLayout = QVBoxLayout()
        self.jobScrollArea = QScrollArea()
        self.jobScrollWidget = QWidget()

        # The ticket list layout holds the tickets and gets updated
        # when a job ID is chosen.
        self.ticketListLayout = QVBoxLayout()

        self.jobScrollWidget.setLayout(self.ticketListLayout)

        self.jobScrollArea.setWidget(self.jobScrollWidget)
        self.jobScrollArea.setWidgetResizable(True)
        self.jobScrollArea.setMaximumHeight(self.height())

        self.jobScrollLayout.addWidget(self.jobScrollArea)

        # Add the scroll layout to the main layout.
        self.mainLayout.addLayout(self.jobScrollLayout, 2, 0, 1, 6)

        # Button to submit the tickets.
        self.submitButton = QPushButton("Submit Edits")
        self.submitButton.setEnabled(False)
        self.submitButton.clicked.connect(self.submit_edits)
        self.mainLayout.addWidget(self.submitButton, 3, 0, 1, 3)

        # Cancel button.
        self.cancelButton = QPushButton("Cancel")
        self.cancelButton.clicked.connect(self.close)
        self.mainLayout.addWidget(self.cancelButton, 3, 3, 1, 3)

    def update_ticket_list(self):
        '''Updates the list of tickets for the selected job.'''
        # Clear the layout first.
        self.clear_layout(self.ticketListLayout)
        
        # Get the chosen job ID.
        if self.jobIDComboBox.currentText() == "Select Job ID":
            self.submitButton.setEnabled(False)
            self.deleteJobButton.setEnabled(False)
            self.deleteJobButton.setStyleSheet("background-color : lightGrey")
            return
        else:
            job_id = int(self.jobIDComboBox.currentText())
            self.submitButton.setEnabled(True)
            self.deleteJobButton.setEnabled(True)
            self.deleteJobButton.setStyleSheet("background-color : red")

        # Add all the tickets' information in horizontal layouts.
        for ticket_id in self.job_ticket_ids[job_id]:
            ticket = self.all_tickets[ticket_id]

            ticketLayout = QHBoxLayout()

            ticketLayout.addWidget(
                FixedWidthLabel(f"{ticket['ticket_id']}", self.square_width)
            )

            parents_string = self.convert_parents_list_to_string(ticket['parents'])
            parentsEdit = FixedWidthLineEdit(self.square_width)
            parentsEdit.setText(parents_string)
            ticketLayout.addWidget(parentsEdit)

            durationEdit = FixedWidthLineEdit(self.square_width)
            durationEdit.setValidator(self.double_validator)
            durationEdit.setText(f"{ticket['duration']: .2f}".replace(' ', ''))
            ticketLayout.addWidget(durationEdit)

            machineType = QComboBox()
            machineType.addItems(machine_type_names)
            machineType.setFixedWidth(self.square_width)

            machine_type_num = self.all_tickets[ticket_id]["machine_type"]
            machineType.setCurrentIndex(machine_type_num)
            ticketLayout.addWidget(machineType)

            ticketLayout.addWidget(FixedWidthLabel(f"{ticket['status']}", self.square_width))

            removeCheck = QCheckBox()
            removeCheck.setChecked(False)
            removeCheck.setFixedWidth(self.square_width)
            ticketLayout.addWidget(removeCheck)

            self.ticketListLayout.addLayout(ticketLayout)
        self.ticketListLayout.addStretch()

    def convert_parents_list_to_string(self, parents_list: list):
        '''Converts a list of parents to comma separated integers.'''
        return str(parents_list).replace('(', '').replace(')', '')

    def clear_layout(self, layout):
        '''Clears the specified layout.

        Need to do so recursively to clean everything properly.
        '''
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
            elif isinstance(item, QLayout):
                self.clear_layout(item)
            else:
                layout.removeItem(item)

    def delete_job(self):
        '''Deletes the selected job.'''
        job_id = int(self.jobIDComboBox.currentText())
        confirm_delete = False
        result = self.openPopupDialog(job_id)
        if result == QDialog.Accepted:
            confirm_delete = True
        else:
            confirm_delete = False

        if confirm_delete == True:
            self.deleteJobID.emit(job_id)
            message = f"Deleting job {job_id}."
            QMessageBox.information(self, "Success", message)
            print(message)
            self.accept()

    def openPopupDialog(self, job_id):
        '''Open the popup confirming job deletion.'''
        # popup_dialog = ConfirmJobDeletionDialog(self, job_id)
        popup_dialog = BasicConfirmDialog(
            self,
            "Confirm Job Deletion",
            f"Are you sure you want to delete job {job_id}?"
        )
        result = popup_dialog.exec_()
        return result

    def submit_edits(self):
        '''Submit the edits made to the tickets.'''
        # Only change the tickets that were edited. This is important because
        # if an ongoing ticket is changed, the ticket manager will reset it
        # and re-schedule it. If we send all tickets in the job over the topic,
        # a ticket might be incorrectly reset.
        job_id = int(self.jobIDComboBox.currentText())
        edited_tickets = []
        for row in range(len(self.job_ticket_ids[job_id])):
            # ticket = self.all_tickets[row]
            # Only 
            ticket_layout = self.ticketListLayout.itemAt(row)

            if self.check_ticket_for_edits(ticket_layout):
                ticket_id = int(ticket_layout.itemAt(0).widget().text())
                parents = ticket_layout.itemAt(1).widget().text()
                parents_string_list = parents.split(',')
                parents_list = []
                for a in parents_string_list:
                    if a != "":
                        parents_list.append(int(a))

                # If any duration is empty, raise an error, which will cause
                # the incomplete info message.
                duration = ticket_layout.itemAt(2).widget().text()
                if len(duration) == 0:
                    raise ValueError("No duration specified.")
                else:
                    duration = float(duration)
                machine_name = ticket_layout.itemAt(3).widget().currentText()
                machine_type_num = machine_type_names.index(machine_name)

                # Append the ticket row if it is valid.
                edited_tickets.append([ticket_id, parents_list, duration, machine_type_num])

        if len(edited_tickets) > 0:
            self.dataEntered.emit(edited_tickets)
            message = f"Edited job successfully."
            QMessageBox.information(self, "Success", message)
            self.accept()
        else:
            message = f"No changes made."
            QMessageBox.information(self, "Success", message)
            self.accept()

    def check_ticket_for_edits(self, row_layout):
        '''Checks if the ticket was edited against its original values.'''
        ticket_id = int(row_layout.itemAt(0).widget().text())

        # If any duration is empty, return True. The ticket was edited.
        # The calling function will take care of the empty duration issue.
        duration = row_layout.itemAt(2).widget().text()
        if len(duration) == 0:
            return True
        else:
            duration = float(duration)

        # Round to 2 decimal places because ticket has ~14 decimal places.
        # TODO: Find the source and remove the rounding.
        if duration != round(self.all_tickets[ticket_id]["duration"], 2):
            print(duration)
            print(self.all_tickets[ticket_id]["duration"])
            print("Duration")
            return True

        # Check if machine type was edited.
        machine_name = row_layout.itemAt(3).widget().currentText()
        machine_type_num = machine_type_names.index(machine_name)

        if machine_type_num != self.all_tickets[ticket_id]["machine_type"]:
            print("Machine type")
            return True

        # Check if parents were edited.
        parents = row_layout.itemAt(1).widget().text()
        parents_string_list = parents.split(',')
        parents_list = []
        for a in parents_string_list:
            if a != "":
                parents_list.append(int(a))

        if not self.lists_have_same_values(
            parents_list, self.all_tickets[ticket_id]["parents"]):
            print("Parents")
            return True

        # If nothing has changed, return false.
        return False

    def lists_have_same_values(self, list1, list2):
        '''
        Checks if two lists have all the same values unordered.

        Args:
            list1: The first list.
            list2: The second list.

        Returns:
            True if the two lists have all the same values, False otherwise.
        '''
        set1 = set(list1)
        set2 = set(list2)

        # If the lengths are different, they can't be same.
        if len(set1) != len(set2):
            return False

        # With same lengths, iterate through one, and if it has a value not in
        # the other, they aren't same.
        for value in set1:
            if value not in set2:
                return False

        return True

# TODO: Delete this after the next commit.
class ConfirmJobDeletionDialog(BasicDialog):
    '''Quick.'''
    def __init__(self, parent: QWidget, job_id: int):
        super().__init__(parent)

        self.setWindowTitle("Confirm Job Delete")
        self.setGeometry(0, 0, 200, 100)

        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.label = FixedWidthLabel(f"Are you sure you want to delete job {job_id}?")

        self.buttons = QDialogButtonBox()
        applyButton = self.buttons.addButton("Yes", QDialogButtonBox.AcceptRole)
        cancelButton = self.buttons.addButton("No", QDialogButtonBox.RejectRole)

        applyButton.clicked.connect(self.accept)
        cancelButton.clicked.connect(self.close)

        self.mainLayout.addWidget(self.label)
        self.mainLayout.addWidget(self.buttons)

        self.show()


class TicketInfoDialog(QDialog):
    '''.'''
    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent=parent)
        self.setWindowTitle("Ticket Info")
        
        self.mainLayout = QHBoxLayout()

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

        self.mainLayout.addLayout(self.ticketInfoLayout)
        self.mainLayout.addWidget(self.jobDrawing)
        self.setLayout(self.mainLayout)
        self.show()