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
    
    
