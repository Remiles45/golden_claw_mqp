
class Slider:
    def __init__(self,slider,label_val,ref,tick):
        self.s = slider
        self.l = label_val
        self.r = ref
        self.t = tick

    def getSlider(self):
        return self.s
    def getLabel(self):
        return self.l
    def getRef(self):
        return self.r
    def getTickCheck(self):
        return self.t
    def setSlider(self,s):
        self.s.setValue(s)
    def setLabel(self,label):
        self.l.setText(label)
    def setRef(self,r):
        self.r = r
    def setTick(self, t):
        self.t = t
    def getVal(self):
        return self.s.value()
