#  Blink an LED with the LGPIO library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import time
import lgpio
import signal
import sys
import atexit

DROPWEIGHT = 11
STROBE = 12

boot_time = time.time()
weight_dropped = False
# open the gpio chip and set the LED pin as output
h = lgpio.gpiochip_open(4)
lgpio.gpio_claim_output(h, DROPWEIGHT,0)
# lgpio.gpio_claim_output(h, STROBE,0)

def cleanup():
    lgpio.gpio_write(h, DROPWEIGHT, 0)
    lgpio.gpio_write(h, STROBE, 0)
    print(f"Cleaned up GPIO")
    lgpio.gpiochip_close(h)
    print(f"Closed the GPIO Chip")

def handle_sigterm(signum, frame):
    print(f"Received signal : {signum}. Cleaning up GPIO")
    cleanup()
    sys.exit(0)

# Catch shutdown / stop signals
signal.signal(signal.SIGTERM, handle_sigterm)  # systemd, shutdown
signal.signal(signal.SIGINT, handle_sigterm)   # Ctrl+C

while True:
    if time.time() - boot_time > 5 * 60 * 60:
        # 5 hours have passed. Drop weight to surface.
        lgpio.gpio_write(h, DROPWEIGHT, 0)
        print(f"Timer ran out. Dropping Weight. Hope to see you in surface")
        weight_dropped = True

    if not weight_dropped:
        lgpio.gpio_write(h, DROPWEIGHT, 1)
    lgpio.gpio_write(h, STROBE, 1)
    time.sleep(1)

    lgpio.gpio_write(h, DROPWEIGHT, 0)
    lgpio.gpio_write(h, STROBE, 0)
    for i in range(5):
        time.sleep(1)
