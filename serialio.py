import serial
import sys
import glob

# rate at which information is transferred over serial port
BAUDRATE = 115200

def get_available_serial_ports() -> list[str]:
    """
    gets a list of available serial port names

    Raises
    ------
    EnvironmentError:
        on unsupported or unknown platforms

    Returns
    -------
    list[str]
        a list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform.')

    available_ports = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            available_ports.append(port)
        except (OSError, serial.SerialException):
            pass

    return available_ports

def init_serial() -> serial.Serial:
    '''
    initialize serial i/o
    '''
    serial_port = ''
    try:
        # assume pico's serial port is the first one available
        serial_port = get_available_serial_ports()[0]
        return serial.Serial(port=serial_port, baudrate=BAUDRATE)
    except serial.SerialException:
        print(f'Serial port {serial_port} not detected. Check connection and try again.')
        exit(1)
    except IndexError:
        print('No serial port detected.')
        exit(1)
    except EnvironmentError as e:
        print(e)
        exit(1)

def readline(serial_connection: serial.Serial) -> str:
    '''
    reads until a '\n' is encountered
    
    Parameters
    ----------
    serial_connection : serial.Serial
        serial connection where data will be read from

    Returns
    -------
    str
        the data from the serial connection
    '''
    return serial_connection.readline().decode('utf-8')

def write_string(string: str, serial_connection: serial.Serial):
    '''
    writes a string to serial connection

    Parameters
    ----------
    string : str
        the string to write
    serial_connection : serial.Serial
        serial connection to write to
    '''
    serial_connection.write(f'{string}\n'.encode('utf-8'))

def main():
    serial_connection = init_serial()
    serial_connection.reset_input_buffer()
    serial_connection.reset_output_buffer()
    write_string('1000', serial_connection)
    while serial_connection.in_waiting == 0: pass
    #while serial_connection.in_waiting > 0:
    #    print(serial_connection.read().decode('utf-8'))
    print(readline(serial_connection))
    serial_connection.close()
