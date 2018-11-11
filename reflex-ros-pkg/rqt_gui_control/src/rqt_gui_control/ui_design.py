from python_qt_binding.QtWidgets import *
from manual_cntrl_widget import ManualHandControlWidget
from glove_cntrl_widget import GloveWidget


class SetUpUI(QMdiArea):
    def __init__(self):
        super(SetUpUI, self).__init__()
        self.manual = ManualHandControlWidget()
        self.auto = GloveWidget()

        self.leftlist = QListWidget ()
        self.leftlist.insertItem (0, 'Manual Control' )
        self.leftlist.insertItem (1, 'Glove Control' )

        self.stack = QStackedWidget()
        self.stack.addWidget(self.manual)
        self.stack.addWidget(self.auto)

        hbox = QHBoxLayout(self)
        hbox.addWidget(self.leftlist)
        hbox.addWidget(self.stack)

        self.setLayout(hbox)
        self.leftlist.currentRowChanged.connect(self.display)
        self.show()

    def display(self,i):
        self.stack.setCurrentIndex(i)
