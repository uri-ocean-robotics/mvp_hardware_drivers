#  Blink an LED with the LGPIO library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import time
import lgpio

DROPWEIGHT = 11
STROBE = 12

boot_time = time.time()

# open the gpio chip and set the LED pin as output
h = lgpio.gpiochip_open(4)
lgpio.gpio_claim_output(h, DROPWEIGHT)
lgpio.gpio_claim_output(h, STROBE)

try:
    while True:
        if (time.time() - boot_time > 5 * 60 * 60):
            # 5 Hours have passed. Drop weight to surface.
            lgpio.gpio_write(h, DROPWEIGHT, 0)

        # Turn the GPIO pin on
        lgpio.gpio_write(h, DROPWEIGHT, 1)
        lgpio.gpio_write(h, STROBE, 1)
        time.sleep(1)

        # Turn the GPIO pin off
        lgpio.gpio_write(h, DROPWEIGHT, 0)
        lgpio.gpio_write(h, STROBE, 0)
        time.sleep(5)

except KeyboardInterrupt:
    lgpio.gpio_write(h, DROPWEIGHT, 0)
    lgpio.gpio_write(h, STROBE, 0)
    lgpio.gpiochip_close(h)
