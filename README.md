# Byron-Encoder Pico and ESP32C versions.
MicroPython (Raspberry Pi Pico) routine to encode Elro DB286A 

Even though - the codes needed to trigger a door bell are 32 bits - it appears that the bottom 16 bits are one of two constants - 0x7A2C and 0x2A6C. 

I've also spotted that there are two possible codes for each doorbell (0xXXXX7A2C and 0xYYYY2A6C) which alternate 
after every bell press. 





```
import time
from machine import Pin
import rp2

# MicroPython routine to ring a Byron DB301 Doorbell with a FS1000A 433.920Mhz transmitter.
# Tested 5v supply from Pico to power FS1000A 



# Byron DB301-V2 Encoder (rtl_433 -R 62 -A)

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def elro_db286A():
    pull(block)

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

    set(pins,1) [31]
    set(pins,1) [31]
    set(pins,1) [31]
    set(pins,0) [31]
    
    mov(isr, x)
    push()




TX_433_PIN = 16  # FS1000A Data Pin
# BELL_ID = 0x26587A2C
BELL_ID = 0x6edd2a6c  #Bell ID! (Obtained from running rtl_433 -R 62 and pressing the bell)
CLOCK = 64000

def bell_ringer_generator(clock=64000):
    
    sm = rp2.StateMachine(0, elro_db286A, freq=CLOCK, set_base=Pin(TX_433_PIN))

    def ring_bell(bell_id):
        sm.active(1)
        for i in range(16):
            sm.put(bell_id)
            sm.get()		  # Wait until all bits exhausted.
            #print(sm.get())
            time.sleep_ms(6)  # wait for the SM to finish
            
        sm.active(0)
        
    return ring_bell


ring = bell_ringer_generator(clock=CLOCK)

while True:
    ring(BELL_ID)
    time.sleep(5)
    

```

Example ESP32C source using RMT module

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
