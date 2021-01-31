import numpy as np

colours = {
    'black':  0,
    'white':  int('0xffffff', 16),
    'red':    int('0xff0000', 16),
    'blue':   int('0x5555ff', 16),
}

class MapDisplay:
    def __init__(self, display_device):
        self.display = display_device
        self.w = self.display.getWidth()
        self.display.setFont('Lucida Console', 20, True)

        self.clear()

        self.legend = {}

    def normalise(self, x):
        """Normalise an input vector x to be displayed"""
        norm = ((x + 1.2) / 2.4) * self.w
        clipped = np.clip(norm, 0, self.w)
        # convert to list to use base numpy type int
        return clipped.astype(int).tolist()

    def clear(self):
        self.display.setColor(colours['black'])
        self.display.fillPolygon([0, 0, self.w, self.w], [0, self.w, self.w, 0])

        red_home = np.array([[0.8, 1.2, 1.2, 0.8], [0.8, 0.8, 1.2, 1.2]])
        self.display.setColor(colours['red'])
        self.display.fillPolygon(*self.normalise(red_home))

        blue_home = (red_home.T + np.array([0., -2., ])).T
        self.display.setColor(colours['blue'])
        self.display.fillPolygon(*self.normalise(blue_home))


    def drawPoint(self, pos, size, colour='white', name=None):
        x, y = self.normalise(pos)
        self.display.setColor(colours[colour])
        self.display.fillOval(x, y, size, size)
        if name is not None: self.legend[name] = colour

    def drawLine(self, p1, p2, colour='white', name=None):
        x1, y1 = self.normalise(p1)
        x2, y2 = self.normalise(p2)
        self.display.setColor(colours[colour])
        self.display.drawLine(x1, y1, x2, y2)
        if name is not None: self.legend[name] = colour

    def drawPolygon(self, pts, colour='white', fill=True, name=None):
        points = self.normalise(pts)
        self.display.setColor(colours[colour])
        self.display.fillPolygon(*points)
        if name is not None: self.legend[name] = colour

    def drawText(self, text, x, y, colour='white'):
        self.display.setColor(colours[colour])
        self.display.drawText(text, x, y)
        
    def drawLegend(self):
        for n, (name, colour) in enumerate(self.legend.items()):
            x = 10
            y = 10 + 30*n
            self.drawText(name, x, y, colour)