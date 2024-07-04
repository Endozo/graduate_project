import threading
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

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

# 將更新的數值計算並應用在三軸上
def update_plot(frame, ax, lock, reader1, reader2, rel_lines):
    # 讀出兩個sensor的DATA，並使用lock來確保兩邊都有讀取到正確的部分才下一步
    with lock:
        R1 = reader1.data
        R2 = reader2.data

    # 計算相對旋轉量
    R_rel = R1 @ R2.T

    # 做出Y, Z兩軸的鏡射算子，並應用在旋轉矩陣上
    mirror_y = np.diag([1, -1, 1])
    mirror_z = np.diag([1, 1, -1])
    R_rel = mirror_y @ R_rel @ mirror_y
    R_rel = mirror_z @ R_rel @ mirror_z

    # 做出三軸的起始狀態
    x = np.array([1, 0, 0])
    y = np.array([0, 1, 0])
    z = np.array([0, 0, 1])

    # 將旋轉矩陣應用在三軸上
    x_rel, y_rel, z_rel = R_rel @ x, R_rel @ y, R_rel @ z

    # 將結果組合成rel_lines
    for line, vec in zip(rel_lines, [x_rel, y_rel, z_rel]):
        line.set_data([0, vec[0]], [0, vec[1]])
        line.set_3d_properties([0, vec[2]])

    # 回傳rel_lines
    return rel_lines

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
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Initialize lines for relative rotation (blue)
    rel_line_x, = ax.plot([0, 1], [0, 0], [0, 0], 'b', label='X-axis')
    rel_line_y, = ax.plot([0, 0], [0, 1], [0, 0], 'g', label='Y-axis')
    rel_line_z, = ax.plot([0, 0], [0, 0], [0, 1], 'r', label='Z-axis')
    rel_lines = [rel_line_x, rel_line_y, rel_line_z]

    # 限制三軸的長度範圍
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()         # 將顏色，三軸的label應用在畫布上

    # 使用FuncAnimation來更新動畫內容
    ani = FuncAnimation(fig, update_plot, fargs=(ax, lock, reader1, reader2, rel_lines), interval=100,
                        cache_frame_data=False)

    try:
        # 將動畫圖片畫出來
        plt.show()
    except KeyboardInterrupt:
        # 如果程式結束了會進這裡
        print("Program terminated by user.")
    finally:
        # 最後要關閉reader的連接，並且用join確保兩個reader都已經完成所有工作才關閉
        reader1.stop()
        reader2.stop()
        reader1.join()
        reader2.join()
