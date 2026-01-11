import math

class TrajectoryGenerator:
    def __init__(self):
        self.mode = "sin"  # "sin", "circle", "line", "lissajous", "step"
        self.amplitude = 500    #振幅
        self.period = 200       #周期

    def generate(self, t):
       # t = frame / 50.0

        if self.mode == "sin":
            val = int(self.amplitude * math.sin(2 * math.pi * t / self.period))
            return [val] * 5
        if self.mode == "triangle":
            val = self.triangle(t, self.amplitude, self.period)
            return [val] * 5
        
        if self.mode == "circle":
            val = int(self.amplitude * math.sin(2 * math.pi * t / self.period))
            return [val] * 5

        if self.mode == "line":
            v = int((t % 4 - 2) * 500)
            return [v] * 5

        if self.mode == "lissajous":
            x = int(1000 * math.sin(t))
            y = int(1000 * math.sin(2*t))
            return [x, y, 0, 0, 0]

        if self.mode == "step":
            step = 1000 if (t // 100) % 2 == 0 else -1000
            return [step]  * 5

        return [0] * 5
    
    def triangle(self, t, amplitude=500, period=200):
        # 0〜1 の sawtooth を作る
        saw = (t % period) / period  # 0〜1

        # 三角波に変換
        tri = 2 * abs(2 * saw - 1) - 1  # -1〜1

        return int(tri * amplitude)
