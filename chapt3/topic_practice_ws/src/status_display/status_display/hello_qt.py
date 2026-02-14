from PyQt5.QtWidgets import QApplication , QWidget , QLabel,QVBoxLayout,QPushButton
import sys

def main():
    #创建QApplication对象
    app = QApplication(sys.argv)
    #创建QLable对象并显示
    label = QLabel()
    label.setText("Hello Qt!")
    label.show()

    sys.exit(app.exec())