from collections import deque
from serial_node.constants import START_BYTE, FEEDBACK_HEADER

class Serial:
    def __init__(self, port, baudrate, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.buffer = deque()  # Use deque for efficient pops from the left
        self.reset_input_buffer()
    
    def reset_input_buffer(self):
        length = 5

        self.buffer.clear()
        for x in [START_BYTE, FEEDBACK_HEADER, length] + [0x7f] * length:
            self.buffer.append(x)

        self.in_waiting = len(self.buffer)

    def flush(self):
        self.in_waiting = 0
        self.reset_input_buffer()  # Refill the buffer for testing

    def read(self, size):
        data = bytearray()
        
        # Read 'size' bytes safely by popping from the front and rotating to the back
        for _ in range(min(size, len(self.buffer))):
            byte = self.buffer.popleft()
            data.append(byte)
            self.buffer.append(byte) # Simulates continuous circular hardware stream
            
        self.in_waiting = len(self.buffer)
        return bytes(data)

    def write(self, data):
        # Simulate writing data to the serial port (for testing)
        print(f"Mock Serial Write: {data}")

    def close(self):
        print("Mock Serial Port Closed")
