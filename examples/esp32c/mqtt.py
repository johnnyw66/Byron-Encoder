"""
 ESP32C MicroPython with OLED display - Subscribe to MQTT Broker (topic bellring/HEXID) and ring bells
 HEXID is a 32 or 24 bit string. eg. 0xAABBCCDD 0xAABBCC
 Example secret.py (which includes Broker and WiFi credentials)

WIFI_SSID = 'MYWIFI-SSID'
WIFI_PASS = 'MYWIFI-PASSWORD'

MQTT_BROKER = 'example-mqtt.com'
MQTT_PORT = 1883
MQTT_USER = 'MQTT_USER'
MQTT_PASS = 'MQTT_PASSWORD'

"""

import network
import time
from umqtt.simple import MQTTClient
import ubinascii
import machine
from esp32 import RMT
from machine import Pin, I2C
import ssd1306

from secret import WIFI_SSID
from secret import WIFI_PASS
from secret import MQTT_BROKER
from secret import MQTT_PORT
from secret import MQTT_USER
from secret import MQTT_PASS

BYRON_BELL_ID_1 = 0x26587A2C
BYRON_BELL_ID_2 = 0x6edd2a6c
TEK_ID_1 = 0x8e00f8
INITIAL_TEST = False

BELLS = [{"type": "ELRO", "id":BYRON_BELL_ID_1},
         {"type": "ELRO", "id":BYRON_BELL_ID_2},
         {"type": "PRINCETON", "id":TEK_ID_1},
        ]

# Define I2C interface using your specified GPIOs
i2c = I2C(0, scl=Pin(6), sda=Pin(5), freq=400000)

switch = Pin(0, Pin.IN, Pin.PULL_UP)
led = Pin(8, Pin.OUT)


# Scan to confirm the OLED is found
print("I2C devices found:", i2c.scan())  # Should return [0x3C] or similar

# Create SSD1306 display object (128x64 or 128x32)
#oled = ssd1306.SSD1306_I2C(64, 48, i2c)
oled = ssd1306.SSD1306_I2C(128, 48, i2c)

# MQTT topic wildcard
TOPIC_WILDCARD = b'bellring/#'

CLIENT_ID = b'esp32_' + ubinascii.hexlify(machine.unique_id())

def waiting_for_bell():
    oled.fill(0)
    oled.text("WAITING", 28, 8)
    oled.text("FOR", 28, 16)
    oled.text("BELL CMD", 28, 24)
    oled.show()

# ELRO 
FS1000A_PIN = 2	# ESP32's GPIO DATA PIN CONNECTED TO FS1000A.

# RMT clock constants - probably only needs looking out if for some day in the future
# these can be changed.
CLOCK_DIV = 80
SOURCE_FREQ = 80_000_000

# Example Elro timings (change to your actual protocol):
# Assume each bit is 496 µs short or 1488 µs long, gap between 32-bit ID is 6980 us
NUM_ELRO_BITS = 32

ELRO_P_LONG_US = 1488
ELRO_P_SHORT_US = 496
ELRO_GAP_US = 6980

# Princeton Timings
NUM_PRINCETON_BITS = 24

# Example Princeton PT2262 / EV1527  timings (change to your actual protocol):
# TE_US was taken from a Flipper Zero Capture.
PRINCETON_TE_US = 518
PRINCETON_P_LONG_US = 3*PRINCETON_TE_US
PRINCETON_P_SHORT_US = PRINCETON_TE_US

PRINCETON_GAP_US = 29*PRINCETON_TE_US


tick_s = CLOCK_DIV / SOURCE_FREQ  # tick duration in seconds

def us_to_ticks(us: float) -> int:
    """Convert microseconds to RMT ticks."""
    return round(us * 1e-6 / tick_s)

# Configure RMT on GPIO2 with 1 µs resolution
rmt = RMT(0, pin=Pin(FS1000A_PIN, Pin.OUT), clock_div=CLOCK_DIV)

def elro_send(code: int, repeats: int = 16) -> None:
    """Send a 32-bit Elro code using RMT."""
    print("elro_send", code)
    
    pulses = []
    
    # Convert uSec timings to clock ticks - Note for the default RMT we should
    # find that the default 80Mhz clock and CLOCK_DIV of 8 will make calling
    # the 'us_to_ticks' function, rather moot.
    T_LONG = us_to_ticks(ELRO_P_LONG_US)
    T_SHORT = us_to_ticks(ELRO_P_SHORT_US)
    GAP = us_to_ticks(ELRO_GAP_US)
    
    #print(T_LONG, T_SHORT, GAP)
    
    bit0 = [T_LONG, T_SHORT]
    bit1 = [T_SHORT, T_LONG]

    # Sync bit (33rd bit, but for the last and first)
    
    pulses.extend([T_SHORT,GAP])

    # 32-bit ID + 1 tail bit + GAP bit
    for i in range(NUM_ELRO_BITS):
        bit = (code >> ((NUM_ELRO_BITS - 1) - i)) & 1

        if bit:
            pulses.extend(bit1)  
        else:
            pulses.extend(bit0)
            
    #print(pulses)

    for _ in range(repeats):
        rmt.write_pulses(pulses)
        rmt.wait_done()
        #print(_)

def princeton_send(code: int, repeats: int = 8) -> None:
    """Send a 32-bit Princeton code using RMT."""

    print("princeton_send", code)
    
    pulses = []
    # Convert uSec timings to clock ticks - Note for the default RMT we should
    # find that the default 80Mhz clock and CLOCK_DIV of 8 will make calling
    # the 'us_to_ticks' function, rather moot.
    T_LONG = us_to_ticks(PRINCETON_P_LONG_US)
    T_SHORT = us_to_ticks(PRINCETON_P_SHORT_US)
    GAP = us_to_ticks(PRINCETON_GAP_US)
        
    bit1 = [T_LONG, T_SHORT]
    bit0 = [T_SHORT, T_LONG]
    

    # 24-bit ID
    for i in range(NUM_PRINCETON_BITS):
        bit = (code >> ((NUM_PRINCETON_BITS-1) - i)) & 1

        if bit:
            pulses.extend(bit1)  
        else:
            pulses.extend(bit0)

    pulses.extend([T_SHORT,GAP])
    
    for _ in range(repeats):
        rmt.write_pulses(pulses)
        rmt.wait_done()

# ===== Wi-Fi =====
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('Connecting to Wi-Fi...')
        wlan.connect(WIFI_SSID, WIFI_PASS)
        while not wlan.isconnected():
            time.sleep(0.5)
    print('Wi-Fi connected:', wlan.ifconfig())
    return wlan

# ===== MQTT =====
def mqtt_callback(topic, msg):
    topic_str = topic.decode()
    print('Received topic:', topic_str, '->', msg)

    # Extract BELLID from topic
    parts = topic_str.split('/')
    if len(parts) >= 2:
        bell_id_hex_str = parts[1]  # assumes topic format bellring/BELLID
        print('BELLID:', bell_id_hex_str)
        bell_id = int(bell_id_hex_str,16) & 0xFFFFFFFF

        oled.fill(0)
        # Write text
        oled.text("RING BELL", 28, 8)
        oled.text(bell_id_hex_str, 28, 16)
        oled.show()
        
        bells = [bell for bell in BELLS if bell_id == bell["id"]]
        print(bells)        
        for _ in bells:
            bell_id = _['id']
            func =  princeton_send if 'PRINCETON' in _['type'] else elro_send if 'ELRO' in _['type'] else None
            if (func):
                func(bell_id)
                
        waiting_for_bell()
        
        
        
        
def mqtt_connect():
    client = MQTTClient(client_id=CLIENT_ID, server=MQTT_BROKER, port=MQTT_PORT,
                        user=MQTT_USER, password=MQTT_PASS, keepalive=60)
    client.set_callback(mqtt_callback)
    client.connect()
    client.subscribe(TOPIC_WILDCARD)
    print('Subscribed to', TOPIC_WILDCARD)
    return client

def ring_bells():
    for bell in BELLS:
        bell_id = bell["id"]
        bell_type = bell["type"]
        print("RING ", hex(bell_id), bell_type)
        if ("ELRO" in bell_type):
            elro_send(bell_id)
        if ("PRINCETON" in bell_type):
            princeton_send(bell_id)
            
        time.sleep(0.5)
    
# ===== Main loop =====
def main_loop():
    #wlan = connect_wifi()
    client = None
    oled.fill(0)
    # Write text
    oled.text("MQTT-TO-", 28, 8)
    oled.text("-ELRO-", 28, 16)
    oled.text("-GATEWAY", 28, 24)
    
    oled.show()
    time.sleep(4)
    
    oled.fill(0)
    # Write text
    oled.text("RING TEST", 28, 8)
    oled.text("RING TEST", 28, 16)
    oled.text("RING TEST", 28, 24)
    oled.text(hex(BYRON_BELL_ID_1), 28, 32)
    
      
    oled.show()

    if (INITIAL_TEST):
        ring_bells()
    

    wlan = connect_wifi()

    waiting_for_bell()
    led_status = 0
    while True:
        try:
            if client is None:
                client = mqtt_connect()
            client.check_msg()  # non-blocking
            if switch.value() == 0:   # 0 means pressed (GND)
                print("Switch PRESSED")
                ring_bells()
                
            #else:
            #    print("Switch released")
            
        except OSError as e:
            print('MQTT connection lost. Reconnecting...', e)
            client = None
            # reconnect Wi-Fi if needed
            if not wlan.isconnected():
                connect_wifi()
        
        time.sleep(0.1)
        led.value(led_status)   # ON
        led_status = not led_status
        
# ===== Run =====
#if __name__ == '__main__':
main_loop()

