import can
import canopen
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================
# Trajectory Generator（軌道生成器）
# ============================================
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


# ============================================
# Axis クラス（S字加速付き）
# ============================================
class Axis:
    def __init__(self, node, node_id, network):
        self.node = node
        self.node_id = node_id
        self.network = network

        self.actual_position = 0
        self.velocity = 0

    def update_motor(self):
        target = self.node.object_dictionary[0x607A].value
        error = target - self.actual_position

        vmax = 200
        accel = 10

        # 加速
        if abs(error) > 200:
            if self.velocity < vmax:
                self.velocity += accel
        # 減速
        else:
            if self.velocity > 0:
                self.velocity -= accel

        # 速度制限
        self.velocity = max(min(self.velocity, vmax), -vmax)

        # 位置更新
        self.actual_position += int(self.velocity)
        self.node.object_dictionary[0x6064].value = self.actual_position

    def on_sync(self):
        self.update_motor()

        pos = self.node.object_dictionary[0x6064].value
        arb_id = 0x180 + self.node_id
        data = int(pos).to_bytes(4, 'little', signed=True)

        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.network.bus.send(msg)


# ============================================
# CANopen Network 初期化
# ============================================
network = canopen.Network()
network.connect(bustype='virtual')

# 受信用バス
rx_bus = can.interface.Bus(bustype='virtual', receive_own_messages=True)

# 5軸ノード
axes = []

from canopen.objectdictionary import Variable, ObjectDictionary

for nid in range(1, 6):
    od = ObjectDictionary()

    # 6064h Actual Position
    var_6064 = Variable("Position actual value", 0x6064, 0)
    var_6064.data_type = 0x0007 # INTEGER32
    od[0x6064] = var_6064
    #od.add_object(var_6064)

    # 607Ah Target Position
    var_607A = Variable("Target position", 0x607A, 0)
    var_607A.data_type = 0x0007 # INTEGER32
    od[0x607A] = var_607A

    #od.add_object(var_607A)

    node = network.add_node(nid, od)

    # ★ Object Dictionary を手動で追加
    #node.sdo.add_variable(0x6064, 0, 'Position actual value', 'i32')
    #node.sdo.add_variable(0x607A, 0, 'Target position', 'i32')
    
    axes.append(Axis(node, nid, network))

# 軌道生成器
traj = TrajectoryGenerator()

# 履歴
history = {i: [] for i in range(1, 6)}

# ============================================
# SYNC 送信
# ============================================
def send_sync():
    msg = can.Message(arbitration_id=0x80, data=[1,0], is_extended_id=False)
    network.bus.send(msg)


# ============================================
# update()（1周期処理）
# ============================================
def update(frame):
    # ① 軌道生成
    targets = traj.generate(frame)
    for axis, target in zip(axes, targets):
        #axis.node.sdo[0x607A].raw = target
        axis.node.object_dictionary[0x607A].value = target
    # ② PDO受信
    while True:
        msg = rx_bus.recv(timeout=0.001)
        if msg is None:
            break

        if 0x180 <= msg.arbitration_id <= 0x185:
            nid = msg.arbitration_id - 0x180
            actual = int.from_bytes(msg.data[:4], 'little', signed=True)
            history[nid].append(actual)

    # ③ SYNC送信
    send_sync()

    # ④ 各軸 on_sync()
    for axis in axes:
        axis.on_sync()

    # ⑤ グラフ更新
    ax.clear()
    for nid in range(1, 6):
        ax.plot(history[nid][-200:], label=f"Axis {nid}")

    ax.legend()
    ax.set_ylim(-1500, 1500)


# ============================================
# グラフ描画
# ============================================
fig, ax = plt.subplots()
ani = FuncAnimation(fig, update, interval=20)
plt.show()
