# UselessBox
https://www.youtube.com/watch?v=giH2SSml5ok&t=4s

Automated Useless Box, responds to Alexa command, opens box, flips switch, closes the box, then plays a tune from SD card

Demo here: https://youtu.be/ujBDMIQfqQ0

Parts:
Box: https://www.amazon.com/Egemwy-Useless-Assembled-MachineTurns-Children/dp/B07DDH6QX8/ref=sr_1_3?crid=39MEG2DG098N6&dchild=1&keywords=useless+box&qid=1608239009&sprefix=useless%2Caps%2C206&sr=8-3

DOIT ESP32 DEVKIT V1 https://www.amazon.com/gp/product/B086MJGFVV/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1

SparkFun Qwiic Motor Driver  https://www.sparkfun.com/products/15451

DFRobot DFPlayerMini https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299

Adafruit Mini Lipo w/Mini-B USB Jack - USB LiIon/LiPoly charger - v1 https://www.adafruit.com/product/1905

Adafruit Lithium Ion Polymer Battery - 3.7v 1200mAh https://www.adafruit.com/product/258

Motor Driver Alternatives:

I wanted to keep everything at 3.3V for the ESP32 be able to use a flat 3.7V Lithium Ion Polymer battery. I tried using a simple L293D H bridge chip, but it did not work well at 3.3V (specification is 5V). I had the Sparkfun Qwiic Motor Driver laying around and it functioned well, but is expensive ($15) and a pain to use because of the Qwicc connector. A cheaper alternative to the Sparkfun Qwiic Motor Driver would probably be to use any DRV8835 (same as in the Sparkfun module) based motor driver, such as a Pololu DRV8835 Dual Motor Driver Carrier, https://www.pololu.com/product/2135 for under $5.


