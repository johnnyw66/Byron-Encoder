# Byron-Encoder Pico and ESP32C versions.
MicroPython (Raspberry Pi Pico) routine to encode Elro DB286A 32-bit.

Even though - the codes needed to trigger a door bell are 32 bits - it appears that the bottom 16 bits are one of two constants - 0x7A2C and 0x2A6C. 

I've also spotted that there are two possible codes for each doorbell (0xXXXX7A2C and 0xYYYY2A6C) which alternate 
after every bell press. 





```
import time
from machine import Pin
import rp2

# MicroPython routine to ring a Byron DB301 Doorbell with a FS1000A 433.920Mhz transmitter.
# Tested 5v supply from Pico to power FS1000A 

TX_433_PIN = 16  # FS1000A Data Pin
CLOCK_FREQ = 64000 # CLOCK FREQ



# Byron DB301-V2 Encoder (rtl_433 -R 62 -A)

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def elro_db286A():
    pull(block)

    set(pins,1) [31]
    set(pins,1) [31]
    set(pins,1) [31]
    set(pins,0) [31]
    set(y, 13)        # 14-1
    label("delay_loop")
    nop() [31]    # 32 cycles
    jmp(y_dec, "delay_loop")

    label("top_bit")
    out(x,1)   #top most bit of osr into x
    jmp(not_x,"not_zero")
    
    # Output zero
    set(pins,1) [31]
    set(pins,0) [31]
    set(pins,0) [31]
    set(pins,0) [30]

    jmp("check_osre")
    
    label("not_zero")
    
    # Output 1
    set(pins,1) [31]
    set(pins,1) [31]
    set(pins,1) [31]
    set(pins,0) [30]
    
    label("check_osre")    
    jmp(not_osre,"top_bit")
    
    mov(isr, x)




def bell_ringer_generator(clock=CLOCK_FREQ, repeats=8):
    
    sm = rp2.StateMachine(0, elro_db286A, freq=clock, set_base=Pin(TX_433_PIN))

    def ring_bell(bell_id):
        sm.active(1)
        for i in range(repeats):
            sm.put(bell_id)
            #time.sleep_ms(6)  # wait for the SM to finish
            
        sm.active(0)
        
    return ring_bell

def ring_continuously(ringer, code, sleep_time = 5):
    print("ring_continously")
    while True:
        ringer(code)
        time.sleep(sleep_time)
    
def seek_code(ringer, lower_bits, sleep_time = 0.5, start = 0xFFFF):
    print("seek_code")
    for upper in range(start, 0x0000, -1):
        code = (upper << 16) | lower_bits
        print(hex(code))
        ringer(code)
        time.sleep(sleep_time)
  


ringer = bell_ringer_generator(clock=CLOCK_FREQ, repeats=4)

seek = False  # Annoy
if not seek:
    code = 0x26587A2c
    ring_continuously(ringer, code, sleep_time = 5)    
else:
    lower = 0x7A2C  # known LOW bits
    seek_code(ringer, lower, sleep_time = 0.125, start=0x2660)    
    

```

Example ESP32C source using RMT module for Elro 32-bit and Princeton PT2262 / EV1527 style 24-bit

```
from esp32 import RMT
from machine import Pin
import time


FS1000A_PIN = 2	# ESP32's GPIO DATA PIN CONNECTED TO FS1000A.

# RMT clock constants - probably only needs looking out if for some day in the future
# these can be changed.
CLOCK_DIV = 80
SOURCE_FREQ = 80_000_000

# Example Elro timings (change to your actual protocol):
# Assume each bit is 496 µs short or 1488 µs long, gap between 32-bit ID is 6980 us
P_LONG_US = 1488
P_SHORT_US = 496
GAP_US = 6980

tick_s = CLOCK_DIV / SOURCE_FREQ  # tick duration in seconds

def us_to_ticks(us: float) -> int:
    """Convert microseconds to RMT ticks."""
    return round(us * 1e-6 / tick_s)

# Configure RMT on GPIO2 with 1 µs resolution
rmt = RMT(0, pin=Pin(FS1000A_PIN, Pin.OUT), clock_div=CLOCK_DIV)

def elro_send(code: int, repeats: int = 16) -> None:
    """Send a 32-bit Elro code using RMT."""
    pulses = []
    
    # Convert uSec timings to clock ticks - Note for the default RMT we should
    # find that the default 80Mhz clock and CLOCK_DIV of 8 will make calling
    # the 'us_to_ticks' function, rather moot.
    T_LONG = us_to_ticks(P_LONG_US)
    T_SHORT = us_to_ticks(P_SHORT_US)
    GAP = us_to_ticks(GAP_US)
    
    #print(T_LONG, T_SHORT, GAP)
    
    bit0 = [T_LONG, T_SHORT]
    bit1 = [T_SHORT, T_LONG]

    # Sync bit (33rd bit, but for the last and first)
    
    pulses.extend([T_SHORT,GAP])

    # 32-bit ID + 1 tail bit + GAP bit
    for i in range(32):
        bit = (code >> (31 - i)) & 1

        if bit:
            pulses.extend(bit1)  
        else:
            pulses.extend(bit0)
            
    #print(pulses)

    for _ in range(repeats):
        rmt.write_pulses(pulses)
        rmt.wait_done()
        #print(_)

# Example usage
ID = 0x6EDD2A6C
ID = 0x26587A2C
#ID = 0x2E1E7A2C
while True:
 elro_send(ID)
 time.sleep(10)
 

```


Below is a version for Princeton PT2262/EV1527 24-bit format.
Tested on a cheap Nestling Doorbell
```
from esp32 import RMT
from machine import Pin
import time


FS1000A_PIN = 2	# ESP32's GPIO DATA PIN CONNECTED TO FS1000A.

# RMT clock constants - probably only needs looking out if for some day in the future
# these can be changed.
CLOCK_DIV = 80
SOURCE_FREQ = 80_000_000
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

def princeton_send(code: int, repeats: int = 8) -> None:
    """Send a 24-bit Princeton code using RMT."""
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

ID = 0x8e00f8
while True:
 print("send", hex(ID))
 princeton_send(ID)
 time.sleep(5)

```
