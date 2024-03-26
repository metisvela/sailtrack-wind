import serial
import datetime

# Configure the serial connection settings
serial_port = '/dev/ttyACM0'  # Linux example, Windows: COM3, Mac: /dev/tty.usbmodem14101
baud_rate = 9600  # Match the baud rate to your Arduino's
output_file_path = 'serial_data_with_timestamps.txt'

# Open the serial port
with serial.Serial(serial_port, baud_rate) as ser, open(output_file_path, 'a') as outfile:
    print(f"Started reading from {serial_port} at {baud_rate} baud rate.")
    
    try:
        while True:
            # Read a line from the serial port
            line = ser.readline().decode('utf-8').strip()
            
            # Get the current timestamp
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # Format the data with the timestamp
            data_with_timestamp = f"{timestamp}: {line}"
            
            # Print to console (optional)
            print(data_with_timestamp)
            
            # Append to the file
            outfile.write(data_with_timestamp + '\n')
            outfile.flush()  # Ensure data is written to file immediately
            
    except KeyboardInterrupt:
        print("Stopped by user.")
