import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QLineEdit, QComboBox, QWidget, 
                             QTextEdit, QGroupBox)
from PyQt5.QtCore import QTimer

class MotorControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.init_ui()
        self.init_serial()
        
    def init_ui(self):
        self.setWindowTitle("Arduino Motor Control")
        self.setGeometry(100, 100, 600, 400)
        
        # Main layout
        main_widget = QWidget()
        main_layout = QVBoxLayout()
        
        # Serial port selection
        serial_group = QGroupBox("Serial Connection")
        serial_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        serial_layout.addWidget(QLabel("Port:"))
        serial_layout.addWidget(self.port_combo)
        serial_layout.addWidget(self.connect_btn)
        serial_group.setLayout(serial_layout)
        
        # Control buttons
        control_group = QGroupBox("Motor Control")
        control_layout = QVBoxLayout()
        
        # Home command
        home_btn = QPushButton("Home (H)")
        home_btn.clicked.connect(lambda: self.send_command("H"))
        
        # Stop command
        stop_btn = QPushButton("Stop (S)")
        stop_btn.clicked.connect(lambda: self.send_command("S"))
        
        # Move up
        up_group = QHBoxLayout()
        self.up_steps = QLineEdit("100000")
        up_btn = QPushButton("Move Up (U)")
        up_btn.clicked.connect(lambda: self.send_command(f"U{self.up_steps.text()}"))
        up_group.addWidget(QLabel("Steps:"))
        up_group.addWidget(self.up_steps)
        up_group.addWidget(up_btn)
        
        # Move down
        down_group = QHBoxLayout()
        self.down_steps = QLineEdit("100000")
        down_btn = QPushButton("Move Down (D)")
        down_btn.clicked.connect(lambda: self.send_command(f"D{self.down_steps.text()}"))
        down_group.addWidget(QLabel("Steps:"))
        down_group.addWidget(self.down_steps)
        down_group.addWidget(down_btn)
        
        # Move to position
        pos_group = QHBoxLayout()
        self.target_pos = QLineEdit("100000")
        pos_btn = QPushButton("Move To Position (M)")
        pos_btn.clicked.connect(lambda: self.send_command(f"M{self.target_pos.text()}"))
        pos_group.addWidget(QLabel("Position:"))
        pos_group.addWidget(self.target_pos)
        pos_group.addWidget(pos_btn)
        
        # Status buttons
        status_group = QHBoxLayout()
        pos_query_btn = QPushButton("Query Position (P)")
        pos_query_btn.clicked.connect(lambda: self.send_command("P"))
        lower_limit_btn = QPushButton("Lower Limit (LL)")
        lower_limit_btn.clicked.connect(lambda: self.send_command("LL"))
        upper_limit_btn = QPushButton("Upper Limit (LU)")
        upper_limit_btn.clicked.connect(lambda: self.send_command("LU"))
        
        status_group.addWidget(pos_query_btn)
        status_group.addWidget(lower_limit_btn)
        status_group.addWidget(upper_limit_btn)
        
        # Add to control layout
        control_layout.addWidget(home_btn)
        control_layout.addWidget(stop_btn)
        control_layout.addLayout(up_group)
        control_layout.addLayout(down_group)
        control_layout.addLayout(pos_group)
        control_layout.addLayout(status_group)
        control_group.setLayout(control_layout)
        
        # Status display
        self.status_display = QTextEdit()
        self.status_display.setReadOnly(True)
        
        # Current position display
        self.position_label = QLabel("Current Position: Unknown")
        
        # Add to main layout
        main_layout.addWidget(serial_group)
        main_layout.addWidget(control_group)
        main_layout.addWidget(self.position_label)
        main_layout.addWidget(self.status_display)
        
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)
        
        # Serial read timer
        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self.read_serial)
        self.read_timer.start(100)  # Check every 100ms
        
    def init_serial(self):
        self.serial_port = None
        
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)
            
    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.connect_btn.setText("Connect")
            self.status_display.append("Disconnected from serial port")
        else:
            port_name = self.port_combo.currentText()
            if not port_name:
                self.status_display.append("No port selected!")
                return
                
            try:
                self.serial_port = serial.Serial(
                    port=port_name,
                    baudrate=115200,
                    timeout=1
                )
                self.connect_btn.setText("Disconnect")
                self.status_display.append(f"Connected to {port_name}")
            except Exception as e:
                self.status_display.append(f"Connection failed: {str(e)}")
                
    def send_command(self, command):
        if not self.serial_port or not self.serial_port.is_open:
            self.status_display.append("Not connected to serial port!")
            return
            
        try:
            self.serial_port.write(f"{command}\n".encode('utf-8'))
            self.status_display.append(f"Sent: {command}")
        except Exception as e:
            self.status_display.append(f"Error sending command: {str(e)}")
            
    def read_serial(self):
        if not self.serial_port or not self.serial_port.is_open:
            return
            
        try:
            while self.serial_port.in_waiting > 0:
                data = self.serial_port.read(12)
                if len(data) == 12:
                    self.process_response(data)
                    
        except Exception as e:
            self.status_display.append(f"Error reading serial: {str(e)}")
            
    def process_response(self, data):
        status_string_byte = data[0]
        position_string_bytes = data[1:]

        position = int(position_string_bytes)
        
        # Update position display
        self.position_label.setText(f"Current Position: {position}")
        
        # Decode status
        status_messages = {
            48: "Unknown Command",
            49: "Command Error",
            50: "Operation Pending",
            51: "Operation Cancelled",
            52: "Over Upper Limit",
            53: "Over Lower Limit",
            54: "Operation Ended",
            55: "TRUE",
            56: "FALSE"
        }
        
        status_msg = status_messages.get(status_string_byte, f"Unknown Status ({chr(status_string_byte)})")
        self.status_display.append(f"Received: {status_msg}, Position: {position}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorControlGUI()
    window.show()
    sys.exit(app.exec_())