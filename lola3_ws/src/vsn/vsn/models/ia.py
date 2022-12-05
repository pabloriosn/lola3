import random


class Ia:
    def random(self):
        a = random.randrange(0, 5, 1)
        angle = random.randrange(-360, 360, 1)
        distance = random.randrange(1, 10, 1)

        if a == 0:
            movement = "left"
            return movement, angle

        elif a == 1:
            movement = "right"
            return movement, angle

        elif a == 2:
            movement = "forward"
            return movement, distance

        elif a == 3:
            movement = "backward"
            return movement, distance

        elif a == 4:
            movement = "stop"
            return movement, 0
