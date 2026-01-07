import typing

import neopixel
import time
from neopixel import *
from ntcore import *
from typing import List
from typing import Tuple
from pattern import removeOverlapAndAddPattern
import pattern

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

def handleLeds(patterns: List[pattern.Pattern]):


    for p in patterns:
        rng = range(p.startingIndex,p.endingIndex)
        chosenStrip = strips[p.strip]
        if p.name == "SOLID_COLOR":
            for i in rng:
                chosenStrip[i] = p.mainColor

        elif p.name == "BLINKING_COLOR":
            period = 1 / p.hz
            on = time.time() % period < period / 2
            color = p.mainColor if on else (0, 0, 0)
            for i in rng:
                chosenStrip[i] = color

        elif p.name == "SWITCHING_COLOR":
            period = 1 / p.hz
            on = time.time() % period < period / 2
            color = p.mainColor if on else p.secondaryColor
            for i in rng:
                chosenStrip[i] = color


        elif p.name == "RAINBOW":

            length = p.endingIndex - p .startingIndex

            # Offset to animate the gradient over time
            offset = int(time.time() * p.hz * 256) % 256

            for i in rng:
                # Calculate a "position" from 0 → 1 across the strip, then shift by offset
                pos = ((i - p.startingIndex) * 256 // length + offset) & 255
                t = pos / 255  # Convert to 0 → 1

                # Interpolate between mainColor and secondaryColor
                color = tuple(
                    int(p.mainColor[j] + (p.secondaryColor[j] - p.mainColor[j]) * t)
                    for j in range(3)
                )

                chosenStrip[i] = color

        chosenStrip.show()


def getNewPattern() -> pattern.Pattern:
    strip = strips[ledLineIDEntry.getDouble()]
    hz = hzEntry.getDouble()
    patternName = patternEntry.getString()
    ledsStart = int(rangeEntry.getDoubleArray()[0])
    ledsEnd = int(rangeEntry.getDoubleArray()[1])
    leds_range = range(ledsStart, ledsEnd)
    mainColor = convertColorToTuple(mainColorEntry)
    secondaryColor = convertColorToTuple(secondaryColorEntry)

    return pattern.Pattern(
        patternName, strip, ledsStart, ledsEnd, hz, convertColorToTuple(mainColor), convertColorToTuple(secondaryColor))


if __name__ == '__main__':
    patterns = []
    FREQUENCY = 0.02 #seconds
    while True:
        removeOverlapAndAddPattern(getNewPattern(), patterns)
        handleLeds(patterns)
        time.sleep(FREQUENCY)



