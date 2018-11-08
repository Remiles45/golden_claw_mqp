
import keyboard
import thread

class KeyboardControl():
    def __init__(self):
        #Enabling Keyboard Control
        thread.start_new_thread(self.keyboardControl, ())
    def keyboardControl(self):
        while True:
            if keyboard.is_pressed('d'):
                pose0 = PoseCommand(f1=0.5,f2=0.0,f3=0.0,k1=0.0,k2=1.0)
                velocity0 = VelocityCommand(f1=3,f2=0,f3=0,k1=0,k2=0)
                self.moveHandtoPose(Command(pose=pose0,velocity=velocity0))#pose0, velocity0)
                self.rate.sleep()
                pose0 = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=1.0)
                velocity0 = VelocityCommand(f1=3,f2=0,f3=0,k1=0,k2=0)
                self.moveHandtoPose(Command(pose=pose0,velocity=velocity0))#pose0, velocity0)
            if keyboard.is_pressed('f'):
                pose0 = PoseCommand(f1=0.0,f2=0.5,f3=0.0,k1=0.0,k2=1.0)
                velocity0 = VelocityCommand(f1=0,f2=3,f3=0,k1=0,k2=0)
                self.moveHandtoPose(Command(pose=pose0,velocity=velocity0))#pose0, velocity0)
                self.rate.sleep()
                pose0 = PoseCommand(f1=0.0,f2=0.0,f3=0.0,k1=0.0,k2=1.0)
                velocity0 = VelocityCommand(f1=0,f2=3,f3=0,k1=0,k2=0)
                self.moveHandtoPose(Command(pose=pose0,velocity=velocity0))#pose0, velocity0)
