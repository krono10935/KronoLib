import neopixel
import time
from neopixel import *
from ntcore import *


#dummy numbers
AMOUNT_OF_LEDS = 30
BRIGHTNESS =10


#TODO: set actual boards
strip1 = neopixel.NeoPixel(board.D18, AMOUNT_OF_LEDS, brightness=BRIGHTNESS)
strip2= neopixel.NeoPixel(board.D18,AMOUNT_OF_LEDS,brightness=BRIGHTNESS)
strip3= neopixel.NeoPixel(board.D18,AMOUNT_OF_LEDS,brightness=BRIGHTNESS)
strip4= neopixel.NeoPixel(board.D18,AMOUNT_OF_LEDS,brightness=BRIGHTNESS)

strips = [strip1, strip2, strip3, strip4]


NetworkTableInstance.setServerTeam(10935)
nt = NetworkTableInstance.getDefault()

table = nt.getTable("Led")
ledLineIDEntry = table.getEntry("id")
patternEntry = table.getEntry("pattern")
mainColorEntry = table.getEntry("mainColor")
secondaryColorEntry = table.getEntry("secondaryColor")
hzEntry = table.getEntry("hz")
rangeEntry = table.getEntry("range")
hasChangeEntry = table.getEntry("hasChange")


def convertColorToTuple(color: NetworkTableEntry):
    color = (color.getDoubleArray())
    return (color[0], color[1], color[2])

def handleLeds():
    strip = strips[ledLineIDEntry.getDouble()]
    hz = hzEntry.getDouble()
    pattern = patternEntry.getString()
    ledsStart = int(rangeEntry.getDoubleArray()[0])
    ledsEnd = int(rangeEntry.getDoubleArray()[1])
    leds_range = range(ledsStart, ledsEnd)
    mainColor = convertColorToTuple(mainColorEntry)
    secondaryColor = convertColorToTuple(secondaryColorEntry)

    if pattern == "SOLID_COLOR":
        for i in leds_range:
            strip[i] = mainColor

    elif pattern == "BLINKING_COLOR":
        period = 1 / hz
        on = time.time() % period < period / 2
        color = mainColor if on else (0, 0, 0)
        for i in leds_range:
            strip[i] = color

    elif pattern == "SWITCHING_COLOR":
        period = 1 / hz
        on = time.time() % period < period / 2
        color = mainColor if on else secondaryColor
        for i in leds_range:
            strip[i] = color


    elif pattern == "RAINBOW":

        length = ledsEnd - ledsStart

        # Offset to animate the gradient over time
        offset = int(time.time() * hz * 256) % 256

        for i in leds_range:
            # Calculate a "position" from 0 → 1 across the strip, then shift by offset
            pos = ((i - ledsStart) * 256 // length + offset) & 255
            t = pos / 255  # Convert to 0 → 1

            # Interpolate between mainColor and secondaryColor
            color = tuple(
                int(mainColor[j] + (secondaryColor[j] - mainColor[j]) * t)
                for j in range(3)
            )

            strip[i] = color

    hasChangeEntry.setBoolean(False)
    strip.show()

if __name__ == '__main__':
    FREQUENCY = 0.02 #seconds
    while True:
        handleLeds()
        time.sleep(FREQUENCY)



