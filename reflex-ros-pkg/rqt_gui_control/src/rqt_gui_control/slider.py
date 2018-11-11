
class Slider:
    def __init__(self,slider,label_val,tick):
        self.s = slider
        self.l = label_val
        self.t = tick

    def getTickCheck(self):
        return self.t
    def setSlider(self,s):
        self.s.setValue(s)
    def setLabel(self,label):
        self.l.setText(label)
    def getVal(self):
        return self.s.value()
