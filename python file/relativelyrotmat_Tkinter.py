import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# threading 相關的部分
class SerialReader(threading.Thread):
    # 初始化threading並準備讀取sensor的值
    def __init__(self, port, lock):
        super().__init__()
        self.port = port
        self.data = np.eye(3)  # Initialize as identity matrix
        self.running = True
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.lock = lock
        time.sleep(1)

    # 讀取sensor print出的值，並將其拆解再組合成3X3的矩陣
    def run(self):
        while self.running:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                data = line.split(', ')
                if len(data) == 9:
                    matrix = tuple(map(float, data))
                    matrix = np.array(matrix).reshape(3, 3)
                    with self.lock:
                        self.data = matrix

    # 當程式停止時，threading 也要關閉
    def stop(self):
        self.running = False
        self.ser.close()

def calculate_relative_rotation(mat1, mat2):
    return mat1 @ np.linalg.inv(mat2)

def update_plot(ax, lock, reader1, reader2, rel_lines):
    with lock:
        mat1 = reader1.data
        mat2 = reader2.data

    rel_rotation = calculate_relative_rotation(mat1, mat2)

    mirror_y = np.diag([1, -1, 1])
    mirror_z = np.diag([1, 1, -1])
    rel_rotation = mirror_y @ rel_rotation @ mirror_y
    rel_rotation = mirror_z @ rel_rotation @ mirror_z

    x = np.array([1, 0, 0])
    y = np.array([0, 1, 0])
    z = np.array([0, 0, 1])

    x_rel, y_rel, z_rel = rel_rotation @ x, rel_rotation @ y, rel_rotation @ z

    for line, vec in zip(rel_lines, [x_rel, y_rel, z_rel]):
        line.set_data([0, vec[0]], [0, vec[1]])
        line.set_3d_properties([0, vec[2]])

def animate():
    update_plot(ax, lock, reader1, reader2, rel_lines)
    canvas.draw()
    root.after(40, animate)  # 每40毫秒更新一次
# main函式
if __name__ == "__main__":
    # 要讀取哪兩個port
    port1 = "COM3"
    port2 = "COM5"

    # 開啟threading的鎖
    lock = threading.Lock()

    # 將python的reader連接上Arduino的Serial port
    reader1 = SerialReader(port1, lock)
    reader2 = SerialReader(port2, lock)

    # reader開始讀取Serial port的資料
    reader1.start()
    reader2.start()

    # 創造一個空白的3D畫布
    root = tk.Tk()
    root.title("3D Rotation Visualization")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    rel_line_x, = ax.plot([0, 1], [0, 0], [0, 0], 'b', label='X-axis')
    rel_line_y, = ax.plot([0, 0], [0, 1], [0, 0], 'g', label='Y-axis')
    rel_line_z, = ax.plot([0, 0], [0, 0], [0, 1], 'r', label='Z-axis')
    rel_lines = [rel_line_x, rel_line_y, rel_line_z]

    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    animate()  # 开始动画

    try:
        root.mainloop()
    except KeyboardInterrupt:
        # 如果程式結束了會進這裡
        print("Program terminated by user.")
    finally:
        # 最後要關閉reader的連接，並且用join確保兩個reader都已經完成所有工作才關閉
        reader1.stop()
        reader2.stop()
        reader1.join()
        reader2.join()
