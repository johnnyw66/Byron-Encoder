# Byron-Encoder
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
