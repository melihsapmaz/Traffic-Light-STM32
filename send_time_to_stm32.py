import serial
import time
import threading
import sys
import datetime

# --- Platform-specific function to get a single character ---
try:
    # Windows
    import msvcrt
    def get_char():
        """Gets a single character from the standard input on Windows."""
        return msvcrt.getch().decode()
except ImportError:
    # POSIX (Linux, macOS)
    import tty
    import termios
    def get_char():
        """Gets a single character from the standard input on POSIX systems."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# --- Configuration ---
SERIAL_PORT = 'COM4'
BAUDRATE = 115200

# --- Global variable to signal threads to exit ---
exit_event = threading.Event()

def read_from_port(ser):
    """Continuously reads and prints data from the serial port."""
    while not exit_event.is_set():
        try:
            line = ser.readline()
            if line:
                # Print incoming messages on a new line to not interfere with user typing
                print(f"\rSTM32: {line.decode().strip()}")
        except serial.SerialException:
            break
        except Exception:
            break

def main():
    """Main function to auto-sync time and then handle single-character input."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"--- Opened {SERIAL_PORT}. Waiting for STM32 to boot... ---")
        time.sleep(2)
    except serial.SerialException as e:
        print(f"Error: Could not open port {SERIAL_PORT}. {e}")
        return

    # --- Step 1: Automatically send the time on startup ---
    print("--- Automatically syncing RTC... ---")
    now = datetime.datetime.now()
    # Format: T<Hour><Minute><Second><Day><Month><Year>
    # Using the current time: Monday, June 23, 2025 at 12:13:56 PM
    time_str = now.strftime("T%H%M%S%d%m%y\r\n") 
    ser.write(time_str.encode())
    print(f"--- Time sync command sent: {time_str.strip()} ---")
    time.sleep(1) # Give STM32 a moment to process the command

    # --- Step 2: Start the interactive terminal ---
    reader_thread = threading.Thread(target=read_from_port, args=(ser,))
    reader_thread.daemon = True
    reader_thread.start()

    print("--- Serial terminal ready. Press keys to send commands. ---")
    print("--- Press 'q' to quit. ---")
    print("-" * 40)

    try:
        while not exit_event.is_set():
            # Get a single character from the user without waiting for Enter
            char_to_send = get_char()

            if char_to_send.lower() == 'q':
                print("\r\n--- Exiting on 'q' command... ---")
                break
            
            print(f"\rSent '{char_to_send}'") # Optional: Give user feedback
            
            # Encode and send the character
            ser.write((char_to_send + '\r\n').encode())

    except KeyboardInterrupt:
        print("\r\n--- Interrupted by user. Exiting... ---")
    finally:
        exit_event.set()
        reader_thread.join(timeout=2)
        ser.close()
        print("--- Serial port closed. ---")

if __name__ == "__main__":
    main()