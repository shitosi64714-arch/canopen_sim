import can

from canopen.objectdictionary import Variable, ObjectDictionary

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
