import sys
import can
import canopen

from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox,QLabel,QGroupBox
from PyQt5.QtCore import QTimer
from matplotlib.axis import Axis
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from canopen.objectdictionary import Variable, ObjectDictionary

import math

class TrajectoryGenerator:
    def __init__(self):
        self.mode = "sin"  # "sin", "circle", "line", "lissajous", "step"
        self.amplitude = 500    #振幅
        self.period = 200       #周期

    def generate(self, t):
       # t = frame / 50.0

        if self.mode == "sin":
            val = int(self.amplitude * math.sin(2 * math.pi * t / self.period))
            return [val] * 5
        if self.mode == "triangle":
            val = self.triangle(t, self.amplitude, self.period)
            return [val] * 5
        
        if self.mode == "circle":
            val = int(self.amplitude * math.sin(2 * math.pi * t / self.period))
            return [val] * 5

        if self.mode == "line":
            v = int((t % 4 - 2) * 500)
            return [v] * 5

        if self.mode == "lissajous":
            x = int(1000 * math.sin(t))
            y = int(1000 * math.sin(2*t))
            return [x, y, 0, 0, 0]

        if self.mode == "step":
            step = 1000 if (t // 100) % 2 == 0 else -1000
            return [step]  * 5

        return [0] * 5
    
    def triangle(self, t, amplitude=500, period=200):
        # 0〜1 の sawtooth を作る
        saw = (t % period) / period  # 0〜1

        # 三角波に変換
        tri = 2 * abs(2 * saw - 1) - 1  # -1〜1

        return int(tri * amplitude)


class MainWindow(QMainWindow):
    def __init__(self):

        # --- CANopen Network ---
        self.network = canopen.Network()
        self.network.connect(bustype='virtual')

        # 受信用バス
        self.rx_bus = can.interface.Bus(bustype='virtual', receive_own_messages=True)

        # --- 5軸の履歴（グラフ用） ---
        self.history = {i: [] for i in range(1, 6)}

        #② 5軸ノード生成（OD を手動追加）
        self.axes = []

        for nid in range(1, 6):
            od = ObjectDictionary()

            # 6064h Actual Position
            var_6064 = Variable("Position actual value", 0x6064, 0)
            var_6064.data_type = 0x0007
            od[0x6064] = var_6064

            # 607Ah Target Position
            var_607A = Variable("Target position", 0x607A, 0)
            var_607A.data_type = 0x0007
            od[0x607A] = var_607A

            node = self.network.add_node(nid, od)
            self.axes.append(Axis(node, nid, self.network))

        #③ 軌道生成器を追加
        self.traj = TrajectoryGenerator()
        self.frame = 0


        super().__init__()
        self.setWindowTitle("CANopen 5-axis Simulator")
        self.resize(1200, 800)

        central = QWidget()
        self.setCentralWidget(central)

        layout = QVBoxLayout()
        central.setLayout(layout)

        # matplotlib キャンバス
        self.fig = Figure()
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_subplot(111)

        layout.addWidget(self.canvas)
        self.canvas.draw()


        # 軸ごとの色
        self.colors = {
            1: "red",
            2: "blue",
            3: "green",
            4: "orange",
            5: "purple"
        }
        # ODlist
        self.od_candidates = [
            0x6064,  # Actual Position
            0x606C,  # Actual Velocity
            0x6077,  # Actual Torque
            0x607A,  # Target Position
            0x6041,  # Statusword
            0x6040,  # Controlword
        ]

        #PD Mappinng Grupe
        self.pdo_group = QGroupBox("PDO Mapping")
        self.pdo_layout = QVBoxLayout()
        self.pdo_group.setLayout(self.pdo_layout)

    # PDO Mapping UI を構築
        for pdo_num in range(1, 5):
            row = self.create_pdo_row(self.axes[0], pdo_num)  # まずは Axis1 用
            self.pdo_layout.addLayout(row)

        # メインレイアウトに追加
        layout.addWidget(self.pdo_group)



    # --- グラフ用ラインを作成 ---
        self.lines = {}
        for nid in range(1, 6):
            line, = self.ax.plot([], [], color=self.colors[nid], label=f"Axis {nid}")
            self.lines[nid] = line

        self.ax.legend(loc="upper right")

        # グリッド追加
        self.ax.grid(True)

        # 軌道モード選択
        self.combo = QComboBox()
        self.combo.addItems(["sin", "circle", "line", "lissajous", "step", "triangle"])
        self.combo.currentTextChanged.connect(self.change_mode)
        layout.addWidget(self.combo)

        # 制御ボタン
        btn_layout = QHBoxLayout()

        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")
        self.reset_btn = QPushButton("Rset")
        self.estop_btn = QPushButton("E-Stop")
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addWidget(self.reset_btn)
        btn_layout.addWidget(self.estop_btn)

        layout.addLayout(btn_layout)

        # タイマー（10ms周期）
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_sim)
        self.timer.start(20) # 20ms周期で update_sim を呼ぶ
        self.start_btn.clicked.connect(self.start_motion)
        self.stop_btn.clicked.connect(self.stop_motion)
        self.reset_btn.clicked.connect(self.reset_motion)
        self.estop_btn.clicked.connect(self.emergency_stop)

        # --- 5軸の現在位置表示ラベル ---
        self.pos_labels = []
        pos_layout = QVBoxLayout()

        for i in range(1, 6):
            label = QLabel(f"Axis {i}: 0")
            label.setStyleSheet("font-size: 16px;")
            self.pos_labels.append(label)
            pos_layout.addWidget(label)

        layout.addLayout(pos_layout)



    def change_mode(self, mode):
        self.traj.mode = mode

    def start_motion(self):
        self.running = True

    def stop_motion(self):
        self.running = False
        for axis in self.axes:
            axis.slow_stop()

    def reset_motion(self):
        for axis in self.axes:
            axis.actual_position = 0
            axis.velocity = 0
            axis.node.object_dictionary[0x6064].value = 0
            axis.node.object_dictionary[0x607A].value = 0

    # グラフもクリア
        for nid in range(1, 6):
            self.history[nid].clear()

    def emergency_stop(self):
        self.running = False
        for axis in self.axes:
            axis.velocity = 0

    def update_motion(self):
        # 軌道生成
        targets = self.traj.generate(self.frame)

        # 各軸に目標位置を送る
        for i, axis in enumerate(self.axes, start=1):
            target = targets[i - 1]

            #---None対策
            if target is None:
                target = 0

            axis.node.object_dictionary[0x607A].raw = int(target)

            # モーター更新
            axis.update()

            # 履歴に追加
            self.history[i].append(axis.position)

        self.frame += 1



    def update_sim(self):

        self.update_motion()
        self.update_graph()
        if not getattr(self, "running", False):
            return
        
        # ここに CANopen の処理を入れる
        self.frame += 1

        # ① 軌道生成
        targets = self.traj.generate(self.frame)
        for axis, target in zip(self.axes, targets):
            axis.node.object_dictionary[0x607A].value = target

        # ② PDO受信
        while True:
            msg = self.rx_bus.recv(timeout=0.001)
            if msg is None:
             break

            if 0x180 <= msg.arbitration_id <= 0x185:
                nid = msg.arbitration_id - 0x180
                actual = int.from_bytes(msg.data[:4], 'little', signed=True)
                self.history[nid].append(actual)

        # ③ SYNC送信
        sync = can.Message(arbitration_id=0x80, data=[1,0], is_extended_id=False)
        self.network.bus.send(sync)

        # ④ 各軸 on_sync()
        for axis in self.axes:
            axis.on_sync()

        # ⑤ グラフ更新(低速版)
        #self.ax.clear()
        #for nid in range(1, 6):
        #    self.ax.plot(self.history[nid][-200:], label=f"Axis {nid}")
        # --- グラフ更新（高速版） ---


        self.canvas.draw()


        self.ax.legend()
        self.ax.set_ylim(-1500, 1500)
        self.canvas.draw()

        # ⑥ 数値ラベル更新
        for axis in self.axes:
            pos = axis.position
            self.pos_labels[axis.node_id - 1].setText(f"Axis {axis.node_id}: {pos}")

        # グリッド追加
        self.ax.grid(True)

    def update_graph(self):
        for nid in range(1, 6):
            y = self.history[nid]
            x = list(range(len(y)))
            self.lines[nid].set_data(x, y)

        # 自動スケール
        self.ax.relim()
        self.ax.autoscale_view()

    #5軸分の位置を1つにまとめる

    def send_multi_axis_pdo(self):
        arb_id = 0x300  # 任意の PDO 番号
        data = bytearray()

        for axis in self.axes:
            pos = axis.node.object_dictionary[0x6064].value
            data += int(pos).to_bytes(4, 'little', signed=True)

        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    #TPDO make fanction
    def create_pdo_row(self, axis, pdo_num):
        row = QHBoxLayout()

        label = QLabel(f"TPDO{pdo_num}")
        row.addWidget(label)

        # 既存マッピングを取得
        od_list = axis.tpdo_map.get(pdo_num, [])

        # OD コンボボックスを並べる
        for od in od_list:
            combo = QComboBox()
            for cand in self.od_candidates:
                combo.addItem(hex(cand))
            combo.setCurrentText(hex(od))

            # 変更時に tpdo_map を更新
            combo.currentTextChanged.connect(
                lambda val, a=axis, p=pdo_num, c=combo: self.update_pdo_map(a, p)
            )

            row.addWidget(combo)

        # [+] ボタン
        add_btn = QPushButton("+")
        add_btn.clicked.connect(lambda _, a=axis, p=pdo_num: self.add_pdo_entry(a, p))
        row.addWidget(add_btn)

        return row

     #TPDO mapping update function
    def update_pdo_map(self, axis, pdo_num):
        row_widgets = self.pdo_layout.itemAt(pdo_num - 1).layout()

        new_list = []
        for i in range(1, row_widgets.count() - 1):  # 最初はラベル、最後は [+]
            widget = row_widgets.itemAt(i).widget()
            if isinstance(widget, QComboBox):
                od = int(widget.currentText(), 16)
                new_list.append(od)

        axis.tpdo_map[pdo_num] = new_list
  
    #OD function
    def add_pdo_entry(self, axis, pdo_num):
        row = self.pdo_layout.itemAt(pdo_num - 1).layout()

        combo = QComboBox()
        for cand in self.od_candidates:
            combo.addItem(hex(cand))

        combo.currentTextChanged.connect(
            lambda val, a=axis, p=pdo_num: self.update_pdo_map(a, p)
        )

        # [+] ボタンの直前に追加
        row.insertWidget(row.count() - 1, combo)

        # マッピング更新
        self.update_pdo_map(axis, pdo_num)



        #pass
class Axis:
    def __init__(self, node, node_id, network):
        self.node = node
        self.node_id = node_id
        self.network = network
        # --- OD の追加（ここが重要） ---

        # Actual Position（既にあるかもしれない）
        self.node.object_dictionary[0x6064] = Variable("Actual Position", 0x6064, 0)
            
        # Actual Velocity（新規追加）
        self.node.object_dictionary[0x606C] = Variable("Actual Velocity", 0x606C, 0)

        # Actual Torque（新規追加）
        self.node.object_dictionary[0x6077] = Variable("Actual Torque",0x6077, 0)

        self.position = 0
        self.velocity = 0
        self.torque = 0
        self.tpdo_map = {
            1: [0x6064],                # TPDO1: 位置だけ
            2: [0x6064, 0x606C, 0x6077] # TPDO2: pos+vel+torque
        }

    def update(self):
        """1ステップ分モーターを更新する"""

        # 目標位置を取得
        target = self.node.object_dictionary[0x607A].value

        #---None対策
        if target is None:
            target = self.position

        # 簡易的な追従モデル（PD制御風）
        error = target - self.position

        # 速度更新
        self.velocity = error * 0.1

        # 位置更新
        self.position += self.velocity

        # トルクは速度に比例（簡易モデル）
        self.torque = self.velocity * 0.5

        # OD に反映
        self.node.object_dictionary[0x6064].value = int(self.position)
        self.node.object_dictionary[0x606C].value = int(self.velocity)
        self.node.object_dictionary[0x6077].value = int(self.torque)
   


    def update_motor(self):
        target = self.node.object_dictionary[0x607A].value
        error = target - self.position

        #---P制御---
        Kp = 0.5 #Gain
        self.velocity = Kp * error

        # 速度制限
        vmax = 2000
        self.velocity = max(min(self.velocity, vmax), -vmax)
        accel = 10

        #---Vlimit---
        #if self.velocity < vmax:
        #    self.velocity = vmax
        #elif self.velocity < -vmax:
        #        self.velocity = -vmax
        # 位置更新

        self.position += int(self.velocity)

        #---torque
        self.torque = self.velocity * 0.5

        # --- od に反映 ---
        self.node.object_dictionary[0x6064].value = self.position        #Actual Possition
        self.node.object_dictionary[0X606C].value = int(self.velocity)          #Actual Velocity
        self.node.object_dictionary[0X6077].value = int(self.velocity * 0.1)    #Actual Torque

    def on_sync(self):
        self.update_motor()

        # 各TPDO を送信
        for pdo_num, od_list in self.tpdo_map.items():


        #pos = self.node.object_dictionary[0x6064].value

        # 送信するODのリスト（TPDOマッピング)
        #od_list = [0x6064, 0x606C, 0x6077]

            arb_id = 0x180 + self.node_id + (pdo_num - 1) * 0x100
            data = bytearray()

        for index in od_list:
            val = self.node.object_dictionary[index].value
            data += int(val).to_bytes(4, 'little', signed=True)
            #arb_id = 0x180 + self.node_id
            #data = int(pos).to_bytes(4, 'little', signed=True)

        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.network.bus.send(msg)

    def slow_stop(self):
        if self.velocity > 0:
            self.velocity -= 20
        elif self.velocity <0:
            self.velocity += 20

        if abs(self.velocity) < 20:
            self.velocity = 0

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
