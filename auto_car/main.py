import threading
import serial

from plotp import *
from go2point_algorithms import *
import time


whl_2R = 37.5 #in mm
whl_R = whl_2R/2 #in mm

ser = serial.Serial('/dev/ttyACM0')
global right_motor, left_motor, En, stop, posx, posy, theta

max_acc = 0.5 # in m per s2
max_vel = 1 # in m per s


right_motor = 0
left_motor = 0
En = 0
stop = False

posx, posy, theta = 0, 0, 0
def RXTHR():
  global posx, posy, theta, stop
  while ser.isOpen:
    if stop == True: return 0
    if ser.in_waiting > 0:
      msg = ser.readline().decode("ascii")
      print(f"Received: {msg}")
      msg_lst = [float(i) for i in msg.split(',')]
      posx, posy, theta = msg_lst
    else:
      time.sleep(0.05)
def TXTHR():
  global right_motor ,left_motor, En, stop
  while ser.isOpen:
    if stop == True: return 0
    msg = f"{right_motor} , {left_motor} , {En}\r\n"
    print(f"Transmited: {msg}")
    ser.write(msg.encode('ascii'))
    time.sleep(0.1)
def Stop(t1, t2):
  stop = True
  time.sleep(0.1)
  t1.join()
  t2.join()
  msg = f"0,0,0\r\n"
  ser.write(msg.encode('ascii'))
  time.sleep(0.3)
  print('SHUTDOWN')

t1 = threading.Thread(target=TXTHR)
t2 = threading.Thread(target=RXTHR)
t1.start()
t2.start()


def SINGLE_COOR(target_x, target_y):
  global right_motor ,left_motor, En, posx, posy, theta, stop
  while not stop:
    v_fwd, theta_t, dist, En = p2p(target_x, target_y, posx, posy, theta, max_acc)
    if (abs(v_fwd) > max_vel):
      v_fwd = max_vel * np.sign(v_fwd)
    left_motor = (v_fwd + 0.1*theta_t)/(np.pi*whl_2R/1000)
    right_motor = (v_fwd - 0.1*theta_t)/(np.pi*whl_2R/1000) 
    # all motors operate on rad/sec

def EXE_SHAPE(coor_list):
  global posx, posy, theta, stop, right_motor, left_motor, En
  while not stop:
    v_fwd, theta_t, dist, En= stack_p2p(coor_list, (posx, posy), theta, max_acc)
    if (abs(v_fwd) > max_vel):
      v_fwd = max_vel * np.sign(v_fwd)
    left_motor = (v_fwd + 0.2*theta_t)/(whl_R/1000) 
    right_motor = (v_fwd - 0.2*theta_t)/(whl_R/1000) 
    # all motors operate on rad/sec


def main():
  num = 40
  sqr = square_coordinates(num, False)
  crc = circle_coordinates(num,False)
  inf = infinity_coordinates(num,False)
  try:
    SINGLE_COOR(2000, 2000)
  except KeyboardInterrupt:
    Stop(t1, t2)

if __name__ == "__main__":
  main()



