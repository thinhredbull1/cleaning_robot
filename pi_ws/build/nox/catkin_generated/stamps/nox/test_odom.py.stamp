import serial
import time

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Change this to your port
BAUD_RATE = 57600             # Match your device's baud rate (from your C++ code)
TIMEOUT = 1                   # Timeout in seconds (adjust as needed)

def initialize_serial(port, baud_rate, timeout):
    """Initialize and return a serial connection."""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=timeout
        )
        print(f"Connected to {port} at {baud_rate} baud")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to {port}: {e}")
        return None

def read_serial_data(ser):
    """Read and process serial data."""
    try:
        # Read a line from the serial port (assumes data ends with \n)
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(f"Received: {line}")
            return line
        return None
    except serial.SerialException as e:
        print(f"Error reading from serial port: {e}")
        return None
    except UnicodeDecodeError:
        print("Received invalid UTF-8 data")
        return None

def parse_data(data):
    """Parse the received serial data (customize based on your format)."""
    if data:
        # Example: If data is "left/right;" like "12/34;" from your C++ sendSerial
        if '/' in data and data.endswith(';'):
            try:
                left, right = data.rstrip(';').split('/')
                left = int(left)
                right = int(right)
                print(f"Parsed - Left: {left}, Right: {right}")
                return left, right
            except ValueError as e:
                print(f"Failed to parse data '{data}': {e}")
                return None, None
        else:
            print(f"Unrecognized data format: {data}")
            return None, None
    return None, None

def main():
    # Initialize serial connection
    ser = initialize_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)
    if not ser:
        return

    try:
        # Main loop to continuously read data
        while True:
            # Read data from serial port
            data = read_serial_data(ser)
            
            # Parse the data (customize this function as needed)
            left, right = parse_data(data)
            
            # Optional: Add your processing logic here
            if left is not None and right is not None:
                # Example: Calculate something with left and right values
                print(f"Processing - Left: {left}, Right: {right}")
            
            # Small delay to avoid overwhelming the CPU
            time.sleep(0.01)  # Adjust as needed

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        # Clean up: Close the serial port
        if ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()