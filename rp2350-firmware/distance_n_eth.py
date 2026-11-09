from machine import UART, Pin
from utime import sleep_ms
import time
import _thread
import neopixel

try:
    from ch9120 import CH9120
except ImportError:
    CH9120 = None

# Status bits
LIDAR_SENSOR_OK = False 
TCP_SEND_OK = False
KEEP_LOOPING = True

def ch9120_configure(eth_uart):
    if CH9120 is None:
        print("CH9120 module is not available")
        return

    # CH9120 Ethernet bridge configuration.
    MODE = 0  # 0: TCP Server
    GATEWAY = (192, 168, 0, 1)
    TARGET_IP = (192, 168, 0, 251)
    LOCAL_IP = (192, 168, 0, 100)
    SUBNET_MASK = (255, 255, 255, 0)
    LOCAL_PORT = 2000
    TARGET_PORT = 2000
    BAUD_RATE = 115200

    ch9120 = CH9120(eth_uart)
    ch9120.enter_config()
    ch9120.set_mode(MODE)
    ch9120.set_localIP(LOCAL_IP)
    ch9120.set_subnetMask(SUBNET_MASK)
    ch9120.set_gateway(GATEWAY)
    ch9120.set_localPort(LOCAL_PORT)
    ch9120.set_targetIP(TARGET_IP)
    ch9120.set_targetPort(TARGET_PORT)
    ch9120.set_baudRate(BAUD_RATE)
    ch9120.exit_config()

    time.sleep(0.5)
    eth_uart.read(eth_uart.any())
    return UART(1, baudrate=115200, tx=Pin(20), rx=Pin(21))


def led_thread():
    global LIDAR_SENSOR_OK, TCP_SEND_OK, KEEP_LOOPING


    GREEN = (20,0,0)
    RED = (0,20,0)
    BLUE = (0,0,20)
    OFF = (0,0,0)

    # status-LED
    pixel = neopixel.NeoPixel(Pin(25), 1)

    pixel[0] = BLUE
    pixel.write()
    time.sleep(1)
    pixel[0] = OFF
    pixel.write()
    time.sleep(1)


    time_w_no_lidar = 0.0
    led_color = RED

    while KEEP_LOOPING:

        if time_w_no_lidar > 10:
            KEEP_LOOPING = False

        if LIDAR_SENSOR_OK:
            sleep_time = 1
            led_color = GREEN
        else:
            led_color = RED
            sleep_time = 0.3

        if not LIDAR_SENSOR_OK:
            time_w_no_lidar += sleep_time*2

        pixel[0] = led_color
        pixel.write()
        time.sleep(sleep_time)
        pixel[0] = OFF
        pixel.write()
        time.sleep(sleep_time)


def main():
    global LIDAR_SENSOR_OK, TCP_SEND_OK, KEEP_LOOPING

    _thread.start_new_thread(led_thread, ())

    # UART1 connects to the RP2350-ETH board's CH9120 module.
    eth_uart = UART(1, baudrate=9600, tx=Pin(20), rx=Pin(21))

    # TF-Luna UART0 wiring: sensor TX -> GP1 (UART0 RX), sensor RX -> GP0.
    uart = UART(0, baudrate=115200, bits=8, parity=None, stop=1,
                tx=Pin(0), rx=Pin(1))

    print("Configuring CH9120 Ethernet bridge in TCP server mode...")
    eth_uart = ch9120_configure(eth_uart) or eth_uart
    print("TF-Luna reader started on UART0 at 115200 baud")
    print("Waiting for sensor frames...")

    print_counter = 100

    frame = bytearray(9)
    while KEEP_LOOPING:
        if uart.any():
            first_byte = uart.read(1)
            if first_byte != b"Y":
                continue

            second_byte = uart.read(1)
            if second_byte != b"Y":
                continue

            payload = uart.readinto(memoryview(frame)[2:])
            if payload != 7:
                continue

            frame[0] = 0x59
            frame[1] = 0x59
            if sum(frame[:8]) & 0xFF != frame[8]:
                print("Invalid TF-Luna frame")
                LIDAR_SENSOR_OK = False
                continue

            LIDAR_SENSOR_OK = True

            distance_cm = frame[2] | (frame[3] << 8)
            signal_strength = frame[4] | (frame[5] << 8)
            temperature_c = ((frame[6] | (frame[7] << 8)) / 8) - 256
            message = "distance_cm={};signal_strength={};temperature_c={:.1f}\n".format(
                distance_cm, signal_strength, temperature_c)
            #print(message, end="")

            try:
                bytearr = bytearray([distance_cm, signal_strength])
                eth_uart.write(bytearr)
                if print_counter < 0:
                    print(f"Sent data over TCP: {distance_cm}")
                    print_counter = 100
            except Exception:
                pass

            print_counter -=1
        else:
            sleep_ms(10)



main()

