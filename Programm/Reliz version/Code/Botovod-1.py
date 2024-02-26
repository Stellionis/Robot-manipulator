import os, msvcrt, sys, serial.tools.list_ports
from PyQt5 import QtWidgets, uic, QtGui, QtCore 
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem, QAbstractItemView
from PyQt5.QtCore import  Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPixmap, QTransform
from scservo_sdk import *
import numpy as np
import time
import threading
import pandas as pd
import datetime
import random
import cv2
import testcv2
from tkinter import Tk, filedialog

# Control table address
ADDR_SCS_TORQUE_ENABLE     = 40
ADDR_SCS_GOAL_ACC          = 41
ADDR_SCS_GOAL_POSITION     = 42
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_PRESENT_POSITION  = 56
ADDR_SCS_PRESENT_CURRENCY  = 69
ADDR_SCS_PRESENT_VOLTAGE   = 62
ADDR_SCS_PRESENT_TEMPERATURE = 63


# Default setting
SCS_ID                      = 1                 # SCServo ID : 1
SCS_MOVING_STATUS_THRESHOLD = 20          # SCServo moving status threshold
SCS_MOVING_SPEED            = 40          # SCServo moving speed
SCS_MOVING_ACC              = 0           # SCServo moving acc
protocol_end                = 0           # SCServo bit end(STS/SMS=0, SCS=1)
START_POSITION              = 2048
START_POSITION_TWO_CL         = 169
START_POSITION_SIX_CL          = 941

QApplication.setAttribute(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        uic.loadUi('main (30).ui', self)
        self.setWindowFlag(QtCore.Qt.WindowTitleHint, False)

        self.thread = None
        self.fill_serial_port_combobox()

        self.ButtonOPEN.clicked.connect(self.open_port)
        self.ButtonOPEN.clicked.connect(self.ping)
        
        self.ButtonCLOSE.clicked.connect(self.close_port)
        
        
        self.ButtonOPEN_file.clicked.connect(self.open_csv)
        self.ButtonSave.clicked.connect(self.save_csv)
        self.ButtonCHOOSE.clicked.connect(self.choose_file_jpeg)

        

        self.ButtonInit.clicked.connect(self.calibration_thr)
        self.ButtonPlay.clicked.connect(self.play_points_thr)
        self.servPos.cellDoubleClicked.connect(self.move_to_cell_thr)
        self.ButtonGRAB.clicked.connect(self.compression_thr)
        self.ButtonUNGRAB.clicked.connect(self.ungrub_thr)
        self.ButtonSTOP.clicked.connect(self.stop)

        self.servPos.setColumnWidth(0, 230)
        self.servPos.setColumnWidth(1, 20)
        self.servPos.setEditTriggers(QAbstractItemView.NoEditTriggers) 
        
        self.servPos.cellClicked.connect(self.cell_clicked)
        self.servPos.cellChanged.connect(self.add_time)



        self.ButtonRemove.clicked.connect(self.deleteRow)
        self.ButtonAdd.clicked.connect(self.add_a_point)
        self.ButtonPlus.clicked.connect(self.add_duration)
        self.ButtonMinus.clicked.connect(self.detract_duration)

        self.Goal_one.editingFinished.connect(self.handEditPos_one)
        self.Goal_two.editingFinished.connect(self.handEditPos_two)
        self.Goal_three.editingFinished.connect(self.handEditPos_three)
        self.Goal_four.editingFinished.connect(self.handEditPos_four)
        self.Goal_five.editingFinished.connect(self.handEditPos_five)
        self.Goal_six.editingFinished.connect(self.handEditPos_six)
        
        
        self.Slider_one.setMinimum(0)
        self.Slider_one.setMaximum(4096)
        self.Slider_two.setValue(int(2048))
        self.Slider_one.valueChanged.connect(self.slider_value_changed_one)
        
        self.Slider_two.setMinimum(1200)
        self.Slider_two.setMaximum(2750)
        self.Slider_two.setValue(int(1300))
        self.Slider_two.valueChanged.connect(self.slider_value_changed_two)

        self.Slider_three.setMinimum(350)
        self.Slider_three.setMaximum(3350)
        self.Slider_three.setValue(int(2300))
        self.Slider_three.valueChanged.connect(self.slider_value_changed_three)

        self.Slider_four.setMinimum(750)
        self.Slider_four.setMaximum(3450)
        self.Slider_four.setValue(int(1000))
        self.Slider_four.valueChanged.connect(self.slider_value_changed_four)

        self.Slider_five.setMinimum(70)
        self.Slider_five.setMaximum(2120)
        self.Slider_five.setValue(int(1100))
        self.Slider_five.valueChanged.connect(self.slider_value_changed_five)

        self.Slider_six.setMinimum(450)
        self.Slider_six.setMaximum(1320)
        self.Slider_six.setValue(int(500))
        self.Slider_six.valueChanged.connect(self.slider_value_changed_six)

        self.terminal = self.findChild(QtWidgets.QListView, 'Terminal')
        self.model = QtGui.QStandardItemModel(self.terminal)
        self.terminal.setModel(self.model)

        self.points      = []
        self.points_two  = []
        self.points_three= []
        self.points_four = []
        self.points_five = []
        self.points_six  = []
        self.voltage      = [0] * 6
        self.current      = [0] * 6
        self.temperature  = [0] * 6
        self.timedata     = [0] * 100
        self.current_row  = -1
        self.numrow       = -1
       
        self.stop_thread_flag = False
        self.grab_flag = False


    
    def fill_serial_port_combobox(self):
        """Заполняет выпадающий список доступных портов USB на Windows"""
        ports = []
        for port in serial.tools.list_ports.comports():
            if port[2] != 'n/a' and 'USB' in port[2]:
                ports.append(port[0])
        self.COM.addItems(ports)

    def open_port(self):
        """Открывает последовательный порт и выводит данные в консоль"""
        try:
            port_name = self.COM.currentText()
            self.portHandler = PortHandler(port_name)
            self.packetHandler = PacketHandler(protocol_end)
            if self.portHandler.openPort():
                print("Succeeded to open the port")
                item1 = QtGui.QStandardItem("Succeeded to open the port")
                self.model.appendRow(item1)
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 1 , ADDR_SCS_GOAL_SPEED, 50)
                time.sleep(0.01)
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_SPEED, 50)
                time.sleep(0.01)
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_SPEED, 50)
                time.sleep(0.01)
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_SPEED, 50)
                time.sleep(0.01)
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_SPEED, 50)
                time.sleep(0.01)
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_SPEED, 50)
                time.sleep(0.01)
                _, _ =self.packetHandler.write1ByteTxRx(self.portHandler, 1 , ADDR_SCS_GOAL_ACC, 60)
                time.sleep(0.01)
                _, _ =self.packetHandler.write1ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_ACC, 60)
                time.sleep(0.01)
                _, _ =self.packetHandler.write1ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_ACC, 60)
                time.sleep(0.01)
                _, _ =self.packetHandler.write1ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_ACC, 60)
                time.sleep(0.01)
                _, _ =self.packetHandler.write1ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_ACC, 60)
                time.sleep(0.01)
                _, _ =self.packetHandler.write1ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_ACC, 60)
    
            else:
                print("Failed to open the port")
                item2 = QtGui.QStandardItem("Failed to open the port")
                self.model.appendRow(item2)
                msvcrt.getch().decode()
                quit()
        except:
            pass
    def calibration_thr(self):
        thr_calibration = threading.Thread(target=self.calibration, daemon=True, name="Thread_calibration" )
        thr_calibration.start()
    def calibration(self):
        """калибровка"""
        try:
            
            start_position_one = self.packetHandler.read2ByteTxRx(self.portHandler, 1, ADDR_SCS_PRESENT_POSITION)[0]
            start_position_two = self.packetHandler.read2ByteTxRx(self.portHandler, 2, ADDR_SCS_PRESENT_POSITION)[0]
            start_position_three = self.packetHandler.read2ByteTxRx(self.portHandler, 3, ADDR_SCS_PRESENT_POSITION)[0]
            start_position_four = self.packetHandler.read2ByteTxRx(self.portHandler, 4, ADDR_SCS_PRESENT_POSITION)[0]
            start_position_five = self.packetHandler.read2ByteTxRx(self.portHandler, 5, ADDR_SCS_PRESENT_POSITION)[0]
            start_position_six = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]

            end_position_one = 2048
            end_position_two = 1300
            end_position_three = 2300
            end_position_four = 1000
            end_position_five = 1100
            end_position_six = 500

            position_delta_one = end_position_one - start_position_one
            position_delta_two = end_position_two - start_position_two
            position_delta_three = end_position_three - start_position_three
            position_delta_four = end_position_four - start_position_four
            position_delta_five = end_position_five - start_position_five  
            position_delta_six = end_position_six - start_position_six

            start_time = time.time()
            duration = 3

            while True and not (self.stop_thread_flag):
                # вычислите прошедшее время с начала движения
                elapsed_time = time.time() - start_time
                # если прошло достаточно времени, выйдите из цикла
                if elapsed_time >= duration:
                    print("ДОКРУТКА")
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 1, ADDR_SCS_GOAL_POSITION, end_position_one)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_POSITION, end_position_two)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_POSITION, end_position_three)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_POSITION, end_position_four)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_POSITION, end_position_five)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, end_position_six)
                    time.sleep(0.005)
                    end_one = self.packetHandler.read2ByteTxRx(self.portHandler, 1, ADDR_SCS_PRESENT_POSITION)[0]
                    end_two = self.packetHandler.read2ByteTxRx(self.portHandler, 2, ADDR_SCS_PRESENT_POSITION)[0]
                    end_three = self.packetHandler.read2ByteTxRx(self.portHandler, 3, ADDR_SCS_PRESENT_POSITION)[0]
                    end_four = self.packetHandler.read2ByteTxRx(self.portHandler, 4, ADDR_SCS_PRESENT_POSITION)[0]
                    end_five = self.packetHandler.read2ByteTxRx(self.portHandler, 5, ADDR_SCS_PRESENT_POSITION)[0]
                    end_six = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]
                    print(end_one, end_two, end_three, end_four, end_five, end_six)

                   
                    if ( (2038 < end_one < 2058)  and (790 < end_two < 810) and (1540 < end_three < 1560) and (890 < end_four < 910) and (2090 < end_five < 2110) and (1580 < end_six < 1600) ):
                        gotov = QtGui.QStandardItem("Сервоприводы готовы к управлению")
                    # if ((end_one not in (2038,2058))  and (end_two not in(795,805)) and (end_three not in(1545,1555)) and (end_four not in(895,905)) and (end_five not in(2095,2105)) and (end_six not in(1585,1595))):
                    else:
                        gotov = QtGui.QStandardItem("Сервоприводы не готовы к управлению")
                    self.model.appendRow(gotov)
                    self.Slider_one.setValue(int(end_one))
                    self.Slider_two.setValue(int(end_two))
                    self.Slider_three.setValue(int(end_three))
                    self.Slider_four.setValue(int(end_four))
                    self.Slider_five.setValue(int(end_five))
                    self.Slider_six.setValue(int(end_six))
                   
                    break

                # вычислите текущую позицию сервопривода на основе времени и изменения позиции
                current_position_one = int(((elapsed_time) / duration) * position_delta_one) + start_position_one
                current_position_two = int(((elapsed_time) / duration) * position_delta_two) + start_position_two
                current_position_three = int(((elapsed_time) / duration) * position_delta_three) + start_position_three
                current_position_four = int(((elapsed_time) / duration) * position_delta_four) + start_position_four
                current_position_five = int(((elapsed_time) / duration) * position_delta_five) + start_position_five
                current_position_six = int(((elapsed_time) / duration) * position_delta_six) + start_position_six
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 1, ADDR_SCS_GOAL_POSITION, current_position_one)
                
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_POSITION, current_position_two)
                
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_POSITION, current_position_three)
               
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_POSITION, current_position_four)
                
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_POSITION, current_position_five)
                
                _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, current_position_six)
                
                print(current_position_one, current_position_two, current_position_three, current_position_four, current_position_five, current_position_six)
                
        except:
            pass
    
    def ping(self):
        """Проверяет наличие сервоприводов"""
        try:
            scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(self.portHandler, 1)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
                print("Сервопривод 1 не найден")
                itemg = QtGui.QStandardItem("Сервопривод 1 не найден") 
                self.model.appendRow(itemg)
            elif scs_error != 0:
                print("Сервопривод 1 не найден")
                itemb = QtGui.QStandardItem("Сервопривод 1 не найден") 
                self.model.appendRow(itemb)

            else:
                print("Связь с 1 сервоприводом установлена")
                itemz = QtGui.QStandardItem("Связь с 1 сервоприводом установлена") 
                self.model.appendRow(itemz)
            
            scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(self.portHandler, 2)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
                print("Сервопривод 2 не найден")
                itemq = QtGui.QStandardItem("Сервопривод 2 не найден") 
                self.model.appendRow(itemq)
            elif scs_error != 0:
                print("Сервопривод 2 не найден")
                itema = QtGui.QStandardItem("Сервопривод 2 не найден") 
                self.model.appendRow(itema)

            else:
                print("Связь со 2 сервоприводом установлена")
                items = QtGui.QStandardItem("Связь со 2 сервоприводом установлена") 
                self.model.appendRow(items)
            
            scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(self.portHandler, 3)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
                print("Сервопривод 3 не найден")
                itemd = QtGui.QStandardItem("Сервопривод 3 не найден") 
                self.model.appendRow(itemd)
            elif scs_error != 0:
                print("Сервопривод 3 не найден")
                itemc = QtGui.QStandardItem("Сервопривод 3 не найден") 
                self.model.appendRow(itemc)

            else:
                print("Связь с 3 сервоприводом установлена")
                itemt = QtGui.QStandardItem("Связь с 3 сервоприводом установлена") 
                self.model.appendRow(itemt)
        
            scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(self.portHandler, 4)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
                print("Сервопривод 4 не найден")
                itemh = QtGui.QStandardItem("Сервопривод 4 не найден") 
                self.model.appendRow(itemh)
            elif scs_error != 0:
                print("Сервопривод 4 не найден")
                itemj = QtGui.QStandardItem("Сервопривод 4 не найден") 
                self.model.appendRow(itemj)

            else:
                print("Связь с 4 сервоприводом установлена")
                itemu = QtGui.QStandardItem("Связь с 4 сервоприводом установлена") 
                self.model.appendRow(itemu)
            
            scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(self.portHandler, 5)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
                print("Сервопривод 5 не найден")
                itemo = QtGui.QStandardItem("Сервопривод 5 не найден") 
                self.model.appendRow(itemo)
            elif scs_error != 0:
                print("Сервопривод 5 не найден")
                itemo = QtGui.QStandardItem("Сервопривод 5 не найден") 
                self.model.appendRow(itemo)

            else:
                print("Связь с 5 сервоприводом установлена")
                itemm = QtGui.QStandardItem("Связь с 5 сервоприводом установлена") 
                self.model.appendRow(itemm)

            scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(self.portHandler, 6)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
                print("Сервопривод 6 не найден")
                iteme = QtGui.QStandardItem("Сервопривод 6 не найден") 
                self.model.appendRow(iteme)
            elif scs_error != 0:
                print("Сервопривод 6 не найден")
                iteme = QtGui.QStandardItem("Сервопривод 6 не найден") 
                self.model.appendRow(iteme)

            else:
                print("Связь с 6 сервоприводом установлена")
                itemy = QtGui.QStandardItem("Связь с 6 сервоприводом установлена") 
                self.model.appendRow(itemy)
        except:
            pass
           
    def close_port(self):
        """Закрывает порт"""
        try:
            self.portHandler.closePort()
            print("Port is closed")   
            item5 = QtGui.QStandardItem("Port is closed") 
            self.model.appendRow(item5)
        except:
            pass

    def slider_value_changed_one(self): 
        """Управление положением первого сервопривода через слайдер"""                                      
        try:
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, 1, ADDR_SCS_GOAL_POSITION, self.Slider_one.value())
            self.Goal_one.setText(str(self.Slider_one.value()))
        except:
            pass

    def handEditPos_one(self):
        """Устанавливает первый слайдер в положение, считываемое из приложения"""
        try:
            self.Slider_one.setValue(int(self.Goal_one.text()))
        except:
            pass

    def slider_value_changed_two(self):
        """Управление вторым сервоприводом через слайдер"""
        try:
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_POSITION, self.Slider_two.value())
            self.Goal_two.setText(str(self.Slider_two.value()))
        except:
            pass

    def handEditPos_two(self):
        """Устанавливает второй слайдер в положение, считываемое из приложения"""
        try:
            self.Slider_two.setValue(int(self.Goal_two.text()))
        except:
            pass

    def slider_value_changed_three(self):
        """Управление третьим сервоприводом через слайдер"""
        try:
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_POSITION, self.Slider_three.value())
            self.Goal_three.setText(str(self.Slider_three.value()))
        except:
            pass
    def handEditPos_three(self):
        """Устанавливает третий слайдер в положение, считываемое из приложения"""
        try:
            self.Slider_three.setValue(int(self.Goal_three.text()))
        except:
            pass
    
    def slider_value_changed_four(self):
        """Управление четвертым сервоприводом через слайдер"""
        try:
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_POSITION, self.Slider_four.value())
            self.Goal_four.setText(str(self.Slider_four.value()))
        except:
            pass
    def handEditPos_four(self):
        """Устанавливает четвертый слайдер в положение, считываемое из приложения"""
        try:
            self.Slider_four.setValue(int(self.Goal_four.text()))
        except:
            pass
    
    def slider_value_changed_five(self):
        """Управление пятым сервоприводом через слайдер"""
        try:
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_POSITION, self.Slider_five.value())
            self.Goal_five.setText(str(self.Slider_five.value()))
        except:
            pass
    def handEditPos_five(self):
        """Устанавливает пятый слайдер в положение, считываемое из приложения"""
        try:
            self.Slider_five.setValue(int(self.Goal_five.text()))
        except:
            pass

    def slider_value_changed_six(self):
        """
        Управление пятым сервоприводом через слайдер
        """
        try:
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, self.Slider_six.value())
            self.Goal_six.setText(str(self.Slider_six.value()))
        except:
            pass
    def handEditPos_six(self):
        """Устанавливает пятый слайдер в положение, считываемое из приложения"""
        try:
            self.Slider_six.setValue(int(self.Goal_six.text()))
        except:
            pass
    
    def cell_clicked(self, row, column):
        """Получает номер столбца и строки, данные из выбранной ячейки"""
        # self.celldata_str = self.servPos.item(row, column).text()
        # self.celldata = eval(self.celldata_str)
        print(row)
        try:
            self.current_row = 1
            self.numrow = row
            if column == 1 :
                self.celldata_str = self.servPos.item(row, column).text()
                self.durationValue.setText(str(self.celldata_str))
        except:
            pass
   
    def add_duration(self):
        try:    
            duration = float(self.durationValue.text())
            duration += 0.5 
            self.durationValue.setText(str(duration))
            if self.current_row == 1:
                item = QTableWidgetItem(str(duration))
                self.servPos.setItem(self.numrow, 1, item) 
        except:
            pass
    
    def detract_duration(self):
        try:
            duration = float(self.durationValue.text())
            if duration > 0:
                duration -= 0.5 
                self.durationValue.setText(str(duration))
            else: pass        
            if self.current_row == 1:
                item = QTableWidgetItem(str(duration))
                self.servPos.setItem(self.numrow, 1, item)
        except:
            pass    

    def move_to_cell_thr(self, row, column):
        self.row = row
        self.column = column
        thr = threading.Thread(target=self.move_to_cell, daemon = True, name="Thread_move_to_cell" )
        thr.start()
    def move_to_cell(self):
        try:
           
            if self.column == 0:

               

                start_position_one = self.packetHandler.read2ByteTxRx(self.portHandler, 1, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_two = self.packetHandler.read2ByteTxRx(self.portHandler, 2, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_three = self.packetHandler.read2ByteTxRx(self.portHandler, 3, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_four = self.packetHandler.read2ByteTxRx(self.portHandler, 4, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_five = self.packetHandler.read2ByteTxRx(self.portHandler, 5, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_six = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]

                celldata_str = self.servPos.item(self.row, self.column).text()
                celldata = eval(celldata_str)

                end_position_one = celldata[0]
                end_position_two = celldata[1] 
                end_position_three = celldata[2]
                end_position_four = celldata[3]
                end_position_five = celldata[4]
                end_position_six = celldata[5]
                
                print("ДВИГАЕМСЯ К ТОЧКЕ:")
                print( end_position_one,end_position_two,end_position_three,end_position_four, end_position_five, end_position_six)

                position_delta_one = end_position_one - start_position_one
                position_delta_two = end_position_two - start_position_two  
                position_delta_three = end_position_three - start_position_three
                position_delta_four = end_position_four - start_position_four
                position_delta_five = end_position_five - start_position_five  
                position_delta_six = end_position_six - start_position_six

                start_time = time.time()
                duration = 2
                
                while True and not (self.stop_thread_flag):
                    # вычислите прошедшее время с начала движения
                    elapsed_time = time.time() - start_time

                    # если прошло достаточно времени, выйдите из цикла
                    if elapsed_time >= duration:
                            
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 1, ADDR_SCS_GOAL_POSITION, end_position_one)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_POSITION, end_position_two)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_POSITION, end_position_three)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_POSITION, end_position_four)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_POSITION, end_position_five)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, end_position_six)
                        time.sleep(0.5)
                        end_one = self.packetHandler.read2ByteTxRx(self.portHandler, 1, ADDR_SCS_PRESENT_POSITION)[0]
                        end_two = self.packetHandler.read2ByteTxRx(self.portHandler, 2, ADDR_SCS_PRESENT_POSITION)[0]
                        end_three = self.packetHandler.read2ByteTxRx(self.portHandler, 3, ADDR_SCS_PRESENT_POSITION)[0]
                        end_four = self.packetHandler.read2ByteTxRx(self.portHandler, 4, ADDR_SCS_PRESENT_POSITION)[0]
                        end_five = self.packetHandler.read2ByteTxRx(self.portHandler, 5, ADDR_SCS_PRESENT_POSITION)[0]
                        end_six = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]

                        print("КОНЕЧНОЕ ПОЛОЖЕНИЕ ", end_one, end_two, end_three, end_four, end_five, end_six)
                        self.Slider_one.setValue(int(end_one))
                        self.Slider_two.setValue(int(end_two))
                        self.Slider_three.setValue(int(end_three))
                        self.Slider_four.setValue(int(end_four))
                        self.Slider_five.setValue(int(end_five))
                        self.Slider_six.setValue(int(end_six))
                        
                        break

                    # вычислите текущую позицию сервопривода на основе времени и изменения позиции
                    current_position_one = int((elapsed_time / duration) * position_delta_one) + start_position_one
                    current_position_two = int((elapsed_time / duration) * position_delta_two) + start_position_two
                    current_position_three = int(((elapsed_time) / duration) * position_delta_three) + start_position_three
                    current_position_four = int(((elapsed_time) / duration) * position_delta_four) + start_position_four
                    current_position_five = int(((elapsed_time) / duration) * position_delta_five) + start_position_five
                    current_position_six = int(((elapsed_time) / duration) * position_delta_six) + start_position_six

                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 1, ADDR_SCS_GOAL_POSITION, current_position_one)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_POSITION, current_position_two)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_POSITION, current_position_three)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_POSITION, current_position_four)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_POSITION, current_position_five)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, current_position_six)
                    print(current_position_one, current_position_two, current_position_three, current_position_four, current_position_five, current_position_six)

                    time.sleep(0.001) 
                    
            else: pass
        except:
            pass
   
    def add_a_point(self):
        
            """
            Сохранение положения сервоприводов и вывод их в таблицу
            """
            
            position_one = self.packetHandler.read2ByteTxRx(self.portHandler, 1, ADDR_SCS_PRESENT_POSITION)[0]
            position_two = self.packetHandler.read2ByteTxRx(self.portHandler, 2, ADDR_SCS_PRESENT_POSITION)[0]
            position_three = self.packetHandler.read2ByteTxRx(self.portHandler, 3, ADDR_SCS_PRESENT_POSITION)[0]
            position_four = self.packetHandler.read2ByteTxRx(self.portHandler, 4, ADDR_SCS_PRESENT_POSITION)[0]
            position_five = self.packetHandler.read2ByteTxRx(self.portHandler, 5, ADDR_SCS_PRESENT_POSITION)[0]
            position_six = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]
            if self.current_row == -1: 
                self.points.append(position_one)
                self.points_two.append(position_two)
                self.points_three.append(position_three)
                self.points_four.append(position_four)
                self.points_five.append(position_five)

                if self.grab_flag == True:
                    self.points_six.append(-1)
                else:
                    self.points_six.append(position_six)

                self.timedata.append(float(self.durationValue.text()))


                row = self.servPos.rowCount() 
                self.servPos.setRowCount(row + 1) 
                self.servPos.setItem(row, 0, QTableWidgetItem(str(row))) 
                

            elif self.current_row == 1:

                self.points.insert( self.numrow+1, position_one)
                self.points_two.insert(self.numrow+1, position_two)
                self.points_three.insert(self.numrow+1, position_three)
                self.points_four.insert(self.numrow+1, position_four)
                self.points_five.insert(self.numrow+1,position_five)
                
                if self.grab_flag == True:
                    self.points_six.insert(self.numrow+1, -1)
                else:
                    self.points_six.insert(self.numrow+1, position_six)

                self.timedata.insert(self.numrow+1, float(self.durationValue.text()))
                
                self.servPos.insertRow(self.numrow + 1)
                row = self.numrow + 1    
            if self.grab_flag == True:
                position_six = -1    
            tablePos = [position_one, position_two, position_three, position_four, position_five, position_six]
            item = QTableWidgetItem(str(tablePos))
            self.servPos.setItem(row, 0, item) 

            # self.timedata[row] = float(self.durationValue.text()) 
            item = QTableWidgetItem(self.durationValue.text()) 
            self.servPos.setItem(row, 1, item) 

            item9 = QtGui.QStandardItem("Положение сохранено")
            self.model.appendRow(item9)

        
    def deleteRow(self):
        """
        Удаление положения сервопривода из таблицы
        """
        try:
            row = self.servPos.currentRow()           
            if row != -1: 
                del self.points[row]
                del self.points_two[row]
                del self.points_three[row]
                del self.points_four[row]
                del self.points_five[row]
                del self.points_six[row]
                del self.timedata[row] 
                self.servPos.removeRow(row) 
                self.current_row = -1
                row = None
            item = QtGui.QStandardItem("Положение удалено")
            self.model.appendRow(item)         
        except:
            pass
    
    def add_time(self, row, column):
        """
        Получение времени из таблицы, запись его в список
        """
        try:
            if column == 1:
                timedata_str= self.servPos.item(row, column).text()
                timedata = float(timedata_str)
                self.timedata[row] = timedata
            else:
                pass
        except:
            pass

    def play_points_thr(self):
        thr_two = threading.Thread(target=self.play_points, daemon=True, name="Thread_play_points")
        thr_two.start()
    def play_points(self):
        """Перемещение сервоприводов в сохраненные положения с заданным временем прибытия в каждую точку"""
        try:
            
            
            itemr = QtGui.QStandardItem("ПОЕХАЛИ!")
            self.model.appendRow(itemr)
            print(self.points, self.points_two, self.points_three, self.points_four, self.points_five, self.points_six, self.timedata)
            for i in range(len(self.points)):
                flag_grab = False
                duration = self.timedata[i]
                time_one_point = duration
                
                start_position_one = self.packetHandler.read2ByteTxRx(self.portHandler, 1, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_two = self.packetHandler.read2ByteTxRx(self.portHandler, 2, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_three = self.packetHandler.read2ByteTxRx(self.portHandler, 3, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_four = self.packetHandler.read2ByteTxRx(self.portHandler, 4, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_five = self.packetHandler.read2ByteTxRx(self.portHandler, 5, ADDR_SCS_PRESENT_POSITION)[0]
                start_position_six = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]

                end_position_one = self.points[i]
                end_position_two = self.points_two[i]     
                end_position_three = self.points_three[i]
                end_position_four = self.points_four[i]
                end_position_five = self.points_five[i]
                end_position_six = self.points_six[i]
                if end_position_six == -1:

                    flag_grab = True
                    self.compression()


                position_delta_one = end_position_one - start_position_one
                position_delta_two = end_position_two - start_position_two 
                position_delta_three = end_position_three - start_position_three
                position_delta_four = end_position_four - start_position_four
                position_delta_five = end_position_five - start_position_five
                if flag_grab == False:  
                    position_delta_six = end_position_six - start_position_six
                

                start_time = time.time()
                while True and not (self.stop_thread_flag):
                    elapsed_time = time.time() - start_time
                    if elapsed_time >= time_one_point:
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 1, ADDR_SCS_GOAL_POSITION, end_position_one)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_POSITION, end_position_two)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_POSITION, end_position_three)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_POSITION, end_position_four)
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_POSITION, end_position_five)
                        if flag_grab == False:  
                            _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, end_position_six)
                        

                        time.sleep(0.01)
                    
                        
                        break
                    current_position_one = int((elapsed_time / time_one_point) * position_delta_one) + start_position_one
                    current_position_two = int((elapsed_time / time_one_point) * position_delta_two) + start_position_two
                    current_position_three = int(((elapsed_time) / duration) * position_delta_three) + start_position_three
                    current_position_four = int(((elapsed_time) / duration) * position_delta_four) + start_position_four
                    current_position_five = int(((elapsed_time) / duration) * position_delta_five) + start_position_five
                    if flag_grab == False:
                        current_position_six = int(((elapsed_time) / duration) * position_delta_six) + start_position_six
                    
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 1, ADDR_SCS_GOAL_POSITION, current_position_one)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_POSITION, current_position_two)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_POSITION, current_position_three)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_POSITION, current_position_four)
                    _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_POSITION, current_position_five)
                    if flag_grab == False:
                        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, current_position_six)
                    print(current_position_one, current_position_two, current_position_three, current_position_four, current_position_five, current_position_six)
                    time.sleep(0.01)
            time.sleep(0.1)
            end_one = self.packetHandler.read2ByteTxRx(self.portHandler, 1, ADDR_SCS_PRESENT_POSITION)[0]
            end_two = self.packetHandler.read2ByteTxRx(self.portHandler, 2, ADDR_SCS_PRESENT_POSITION)[0]
            end_three = self.packetHandler.read2ByteTxRx(self.portHandler, 3, ADDR_SCS_PRESENT_POSITION)[0]
            end_four = self.packetHandler.read2ByteTxRx(self.portHandler, 4, ADDR_SCS_PRESENT_POSITION)[0]
            end_five = self.packetHandler.read2ByteTxRx(self.portHandler, 5, ADDR_SCS_PRESENT_POSITION)[0]
            end_six = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]
            self.Slider_one.setValue(int(end_one))
            self.Slider_two.setValue(int(end_two))
            self.Slider_three.setValue(int(end_three))
            self.Slider_four.setValue(int(end_four))
            self.Slider_five.setValue(int(end_five))
            self.Slider_six.setValue(int(end_six))
            
        except:
            pass

    def compression_thr(self):
        thr = threading.Thread(target=self.compression, daemon = True, name="Thread_compression")
        thr.start()

    def compression(self):
        """Захват объекта с учетом силы нажатия по току"""
        try:  
            self.grab_flag = True  
            while True and not (self.stop_thread_flag): 
                current_valuy_six, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_CURRENCY)
                data = SCS_TOHOST(current_valuy_six, 15)
                present_curr_six = abs(data * 6.5)
                time.sleep(0.1)
                print(present_curr_six)
                a = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]
                print(a)
                print(present_curr_six)
                if (present_curr_six < 30.0) and (a > 1310) and (a < 1320) :
                    break
                if (present_curr_six > 80.0):
                    break
                time.sleep(0.1)
                _, _ = self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, a+50)
                time.sleep(0.1)
            
            itemf = QtGui.QStandardItem("Кусь!")
            self.model.appendRow(itemf)
        except:
            pass
    
    def ungrub_thr(self):
        thr = threading.Thread(target=self.ungrub, daemon = True, name="Thread_ungrub")
        thr.start()
    def ungrub(self): 
        """Разжатие клешней"""
        try:
            self.grab_flag = False
            a = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]
            while (a not in range(450,510)) and not (self.stop_thread_flag): 
                    a = self.packetHandler.read2ByteTxRx(self.portHandler, 6, ADDR_SCS_PRESENT_POSITION)[0]
                    _, _ = self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_POSITION, 500)
            
        except: pass
            
    def choose_file(self):
        try:
            root = Tk()
            root.withdraw()
            file_path = filedialog.askopenfilename(filetypes=(("JPEG files", "*.jpg;*.jpeg"), ("All files", "*.*")))
            return file_path
        except: pass

    def choose_file_jpeg(self):
        try:
            file_path2 = self.choose_file()
            file1 = QtGui.QStandardItem("Выбранный файл: ")
            file2 = QtGui.QStandardItem(file_path2)
            self.model.appendRow(file1)
            self.model.appendRow(file2)
            jpeg_foto = QPixmap(file_path2)
            jpeg_foto = jpeg_foto.scaled(261, 261)
            self.jpeg.setPixmap(jpeg_foto)
        except:
            pass

    def save_csv(self):
        try:
            df = pd.DataFrame(columns=['servo', 'servo2', 'servo3', 'servo4', 'servo5', 'servo6', 'time'])
        
            print(self.points)

            for n in range(len(self.points)):  
                df.loc[len(df.index)] = [self.points[n],self.points_two[n],self.points_three[n],self.points_four[n],self.points_five[n], self.points_six[n],self.timedata[n]]
                print(self.points_six[n])
            timestamp = datetime.datetime.now().strftime('%Y-%d-%m-%H-%M-%S')
            random_number = random.randint(1, 1000)
            file_name = f"df_{timestamp}_{random_number}.csv"
            df.to_csv(file_name, index = False)
            print(df)
            item_save = QtGui.QStandardItem(f"Файл сохранен как {file_name}")
            self.model.appendRow(item_save)   
        except: 
            pass
    def open_file_csv(self):
        try:
            roots = Tk()
            roots.withdraw()
            file_path = filedialog.askopenfilename(filetypes=(("CSV", "*.csv"), ("All files", "*.*")))
            return file_path
        except: pass
    def open_csv(self):
        try:
            self.points = []
            self.points_two = []
            self.points_three = []
            self.points_four = []
            self.points_five = []
            self.points_six = []
            self.timedata = []
            if (self.servPos.rowCount()  != 0):
                self.servPos.setRowCount(0)
                t = 0
                i = 0
            file_csv = self.open_file_csv()
            fd = pd.read_csv(file_csv)
            servo_one = fd["servo"]
            servo_two = fd["servo2"]
            servo_three = fd["servo3"]
            servo_four = fd["servo4"]
            servo_five = fd["servo5"]
            servo_six = fd["servo6"]
            time = fd["time"]
            for i in servo_one.index:
                
                
                print(servo_one.iloc[i])
                self.points.append(servo_one.iloc[i])
                self.points_two.append(servo_two.iloc[i])
                self.points_three.append(servo_three.iloc[i])
                self.points_four.append(servo_four.iloc[i])
                self.points_five.append(servo_five.iloc[i])
                self.points_six.append(servo_six.iloc[i])
                self.timedata.append(time.iloc[i])
                self.points = list(map(int, self.points))
                self.points_two = list(map(int, self.points_two))
                self.points_three = list(map(int, self.points_three))
                self.points_four = list(map(int, self.points_four))
                self.points_five = list(map(int, self.points_five))
                self.points_six = list(map(int, self.points_six))
                self.timedata = list(map(int,self.timedata))
            for t in range(len(self.points)):
                row = self.servPos.rowCount() 
                self.servPos.setRowCount(row + 1) 
                self.servPos.setItem(row, 0, QTableWidgetItem(str(row))) 
                tablePos = [self.points[t], self.points_two[t], self.points_three[t], self.points_four[t], self.points_five[t], self.points_six[t]]
                item = QTableWidgetItem(str(tablePos))
                self.servPos.setItem(t, 0, item) 
                item_two_three = QTableWidgetItem(str(self.timedata[t]))
                self.servPos.setItem(t, 1, item_two_three) 
                print(t)
            print(self.points, self.points_two, self.points_three, self.points_four, self.points_five, self.timedata)
        except: pass
    def stop(self):
        self.stop_thread_flag = True
        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 1, ADDR_SCS_GOAL_SPEED, 0)
        time.sleep(0.01)
        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_SCS_GOAL_SPEED, 0)
        time.sleep(0.01)
        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_SCS_GOAL_SPEED, 0)
        time.sleep(0.01)
        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 4, ADDR_SCS_GOAL_SPEED, 0)
        time.sleep(0.01)
        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 5, ADDR_SCS_GOAL_SPEED, 0)
        time.sleep(0.01)
        _, _ =self.packetHandler.write2ByteTxRx(self.portHandler, 6, ADDR_SCS_GOAL_SPEED, 0)
        itemаf = QtGui.QStandardItem("Манипулятор остановлен!")
        self.model.appendRow(itemаf)
        self.stop_thread_flag = False


   
if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.stdout.flush()
    sys.exit(app.exec_())



