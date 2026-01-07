from typing import List
from typing import Tuple
class Pattern:

    def __init__(self, name: str, strip: int, startingIndex: int, endingIndex: int,
                 hz: int, mainColor: Tuple[float, float, float], secondaryColor: Tuple[float,float, float]):
        self.name = name
        self.strip = strip
        self.startingIndex = startingIndex
        self.endingIndex = endingIndex
        self.hz = hz
        self.mainColor = mainColor
        self.secondaryColor = secondaryColor

    def __repr__(self):
        return f"Pattern({self.name}, {self.startingIndex}-{self.endingIndex})"

# Function to check if two patterns overlap
def overlap(p1: Pattern, p2: Pattern) -> bool:
    # Two patterns overlap if their ranges intersect
    return not (p1.endingIndex <= p2.startingIndex or p1.startingIndex >= p2.endingIndex)

# Function to remove overlapping patterns from a list
def removeOverlapAndAddPattern(newPattern: Pattern, patterns: List[Pattern]) -> List[Pattern]:
    # Keep only patterns that do NOT overlap with newPattern
    new = [p for p in patterns if not overlap(newPattern, p)]
    new.append(newPattern)
    return new


