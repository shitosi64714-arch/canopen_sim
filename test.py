import sys
import time
import can
import canopen
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from trajectory import TrajectoryGenerator
from axis import Axis

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CANopen 5-axis Simulator")
        self.resize(1200, 800)

        layout = QVBoxLayout()
        self.setLayout(layout)

        # matplotlib

# ============================
#  Axis クラス（5軸の仮想ドライブ）
# ============================
class Axis:
    def __init__(self, network, node_id, eds):
        self.network = network
        self.node = canopen.LocalNode(node_id, eds)
        self.node_id = node_id
        network.add_node(self.node)

        self.actual_position = 0
        self.statusword = 0x0000
        self.velocity = 0

        # PDO設定（TPDO1）
        #self.node.tpdo.read()
        self.node.tpdo[1].enabled = True
        self.node.tpdo[1].clear() # 念のため
        self.node.tpdo[1].add_variable(0x6064)  # Actual Position
        self.node.tpdo[1].trans_type = 1        # SYNCごと
        self.node.tpdo[1].cob_id = 0x180 + node_id # ← これが超重要
        self.node.tpdo[1].save()
        #self.node.tpdo[1].transmit()
        #self.node.tpdo[1].event_timer = 0 
        #self.node.tpdo[1].inhibit_time = 0
      

    # SYNCを受けたときに呼ばれる
    def on_sync(self):
        # 状態遷移は省略（すでに 0x2700 にしているので OK）
        self.statusword = 0x2700
        self.update_motor()

        # 6064h の値を取得
        pos = self.node.sdo[0x6064].raw

        # PDO フレームを自分で作る
        arb_id = 0x180 + self.node_id
        data = int(pos).to_bytes(4, 'little', signed=True)

        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        #print("SEND:", hex(arb_id), list(data))
        # CAN バスに送信（network.bus を使う）
        self.network.bus.send(msg)

    # CiA402 状態遷移
    def update_state(self):
        #print("axes =", axes)
        cw = self.node.sdo[0x6040].raw

        if cw == 0x06:
            self.statusword = 0x2100
        elif cw == 0x07:
            self.statusword = 0x2300
        elif cw == 0x0F:
            self.statusword = 0x2700

        self.node.sdo[0x6041].raw = self.statusword

    # モータモデル（Target に向かって 10% ずつ近づく）
    def update_motor(self):
        target = self.node.sdo[0x607A].raw
        error = target - self.actual_position

        vmax = 200
        acccel = 10

        if abs(error) > 200:
            if self.velocity < vmax:
                self.velocity += acccel
            else:
                if self.velocity > 0:
                    self.velocity -= acccel

            self.velocity = max(min(self.velocity, vmax), -vmax)

            self.actual_position += int(self.velocity)
            # PDO で送る値
            self.node.sdo[0x6064].raw = self.actual_position


        # 1次遅れ追従（Profile Position の簡易版）
        #self.actual_position += int(error * 0.1)

    
        
        #if self.statusword == 0x2700:  # Operation Enabled
        #    target = self.node.sdo[0x607A].raw
        #    self.actual_position += 10
            #self.actual_position += int((target - self.actual_position) * 0.1)
        #    self.node.sdo[0x6064].raw = self.actual_position

            # TPDO送信
         #   self.node.tpdo[1].transmit()

class TrajectoryGenerator:
    def __init__(self):
        self.mode = "sin"  # sin / circle / line / lissajous / step

    def generate(self, frame):
        t = frame / 50.0

        if self.mode == "sin":
            return [int(1000 * math.sin(t + i*0.3)) for i in range(5)]

        if self.mode == "circle":
            x = int(1000 * math.cos(t))
            y = int(1000 * math.sin(t))
            return [x, y, 0, 0, 0]

        if self.mode == "line":
            v = int((t % 4 - 2) * 500)
            return [v]*5

        if self.mode == "lissajous":
            x = int(1000 * math.sin(t))
            y = int(1000 * math.sin(2*t))
            return [x, y, 0, 0, 0]

        if self.mode == "step":
            step = 1000 if (frame//100)%2==0 else -1000
            return [step]*5
        
        return [0]*5


# ============================
#  CANopen ネットワーク初期化
# ============================
network = canopen.Network()
network.connect(bustype='virtual')
bus=network.bus

rx_bus = can.interface.Bus(bustype='virtual',receive_own_messages=True)

#print("network.bus =", network.bus)
#print("bus =", bus)
print("network.bus is bus =", network.bus is bus)
print("id(network.bus) =", id(network.bus))
print("id(bus) =", id(bus))


# 5軸作成
axes = []
for nid in range(1, 6):
    axis = Axis(network, nid, 'cia402.eds')
    axes.append(axis)

network.nmt.state = 'OPERATIONAL'


# ============================
#  マスタ側（PDO受信＋グラフ描画）
# ============================

# データバッファ
history = {nid: [] for nid in range(1, 6)}

# matplotlib 準備
fig, ax = plt.subplots()
lines = {}

for nid in range(1, 6):
    line, = ax.plot([], [], label=f"Axis {nid}")
    lines[nid] = line

ax.set_xlim(0, 300)
ax.set_ylim(-20000, 20000)
ax.legend()

# SYNC送信
def send_sync():
    msg = can.Message(arbitration_id=0x80, data=[], is_extended_id=False)
    bus.send(msg)

# アニメーション更新
def update(frame):
    # ----------------------------- 
    # ① 多軸同期ターゲット生成 
    # ----------------------------- 
    t = frame / 50.0 # 時間パラメータ（周期に合わせて調整） 
    
    # 5軸に位相をずらしたサイン波を与える 
    for i, axis in enumerate(axes, start=1):
        target = int(1000 * math.sin(t + i * 0.3)) 
        axis.node.sdo[0x607A].raw = target
    
    # ----------------------------- 
    # ② 前フレームのPDOを受信 
    # -----------------------------

 
    while True:
        msg = rx_bus.recv(timeout=0.01)

        if msg is None:
            #print("PDO:", msg.arbitration_id, actual)
            break

        print("RECV:", hex(msg.arbitration_id), list(msg.data))
        
        # PDO (0x180〜0x185)
        if 0x180 <= msg.arbitration_id <= 0x185:
            nid = msg.arbitration_id - 0x180
            actual = int.from_bytes(msg.data[:4], byteorder='little', signed=True)
            #print("PDO:", msg.arbitration_id, actual)
            history[nid].append(actual)
    
    # ----------------------------- 
    # ③ SYNC送信 
    # -----------------------------
    send_sync()
    
    # ----------------------------- 
    #  ④ 各Axisのon_sync()を呼ぶ（ここでPDO送信） 
    #  -----------------------------
    for axis in axes:
        axis.on_sync()

# ----------------------------- 
#  ⑤ グラフ更新 
#  -----------------------------
    for nid in range(1, 6):
        lines[nid].set_data(range(len(history[nid])), history[nid])

    ax.set_xlim(0, max(200, len(history[1])))
    ax.set_ylim(-1200, 1200)
    #print({nid: len(history[nid]) for nid in history})
    return list(lines.values())
    

ani = animation.FuncAnimation(fig, update, interval=10)
plt.show()
