import mujoco
import mujoco.viewer
import numpy as np
from pynput import keyboard

# 初始化按键状态
key_states = {
    # 移动控制键
    keyboard.Key.up: False,    # 前 (X+)
    keyboard.Key.down: False,  # 后 (X-)
    keyboard.Key.left: False,  # 左 (Y-)
    keyboard.Key.right: False, # 右 (Y+)
    ',': False,  # 上 (Z+)
    '.': False,  # 下 (Z-)
    
    # 旋转控制键
    '1': False,  # X轴正向旋转
    '2': False,  # X轴负向旋转
    '3': False,  # Y轴正向旋转
    '4': False,  # Y轴负向旋转
    '5': False,  # Z轴正向旋转
    '6': False   # Z轴负向旋转
}

def on_press(key):
    try:
        # 处理特殊按键
        if key in key_states:
            key_states[key] = True
        # 处理数字键 - 使用更可靠的字符检测方式
        elif hasattr(key, 'char') and key.char in key_states:
            key_states[key.char] = True
    except Exception as e:
        print(f"按键错误: {e}")

def on_release(key):
    try:
        # 处理特殊按键
        if key in key_states:
            key_states[key] = False
        # 处理数字键
        elif hasattr(key, 'char') and key.char in key_states:
            key_states[key.char] = False
    except Exception as e:
        print(f"释放错误: {e}")

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

XML = """
<mujoco>
  <option gravity="0 0 0"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="140" elevation="-30"/>
  </visual>

  <asset>
    <texture type="2d" name="ball_texture" builtin="checker" rgb1="0.8 0.2 0.2" rgb2="0.2 0.8 0.8" width="100" height="100"/>
    <material name="ball_material" texture="ball_texture" texuniform="true"/>
    <!-- 增强地面纹理 -->
    <texture type="2d" name="floor_texture" builtin="checker" width="512" height="512" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"/>
    <material name="floor_material" texture="floor_texture" texrepeat="6 6" texuniform="true" reflectance="0.2"/>
  </asset>

  <worldbody>
    <!-- 优化地面效果 -->
    <geom name="floor" type="plane" size="3 3 0.1" rgba="0.8 0.9 1 1" material="floor_material"/>
    
    <body name="ball" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.2" material="ball_material" mass="5"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(XML)
data = mujoco.MjData(model)

class BallControl:
    def __init__(self):
        self.rotation_angle = np.pi / 18  # 10度旋转角度
        self.move_step = 0.2  # 移动步长
        self.last_key_states = dict(key_states)
        
    def apply_control(self, key_states):
        """处理键盘控制，更新球的位置和旋转"""
        changed = False
        # 对于自由关节，qpos前3个是位置，后4个是四元数
        current_pos = data.qpos[:3].copy()
        current_quat = data.qpos[3:7].copy()
        
        # ==================== 处理位置移动 ====================
        # 上箭头键: 向前移动 (沿X轴正方向)
        if key_states[keyboard.Key.up] and not self.last_key_states.get(keyboard.Key.up, False):
            current_pos[0] += self.move_step
            changed = True
            print("上箭头: 向前移动 (X+)")
            
        # 下箭头键: 向后移动 (沿X轴负方向)
        if key_states[keyboard.Key.down] and not self.last_key_states.get(keyboard.Key.down, False):
            current_pos[0] -= self.move_step
            changed = True
            print("下箭头: 向后移动 (X-)")
            
        # 左箭头键: 向左移动 (沿Y轴负方向)
        if key_states[keyboard.Key.left] and not self.last_key_states.get(keyboard.Key.left, False):
            current_pos[1] -= self.move_step
            changed = True
            print("左箭头: 向左移动 (Y-)")
            
        # 右箭头键: 向右移动 (沿Y轴正方向)
        if key_states[keyboard.Key.right] and not self.last_key_states.get(keyboard.Key.right, False):
            current_pos[1] += self.move_step
            changed = True
            print("右箭头: 向右移动 (Y+)")
            
        # 逗号键: 向上移动 (沿Z轴正方向)
        if key_states[','] and not self.last_key_states.get(',', False):
            current_pos[2] += self.move_step
            changed = True
            print("逗号键: 向上移动 (Z+)")
            
        # 句号键: 向下移动 (沿Z轴负方向)
        if key_states['.'] and not self.last_key_states.get('.', False):
            current_pos[2] -= self.move_step
            changed = True
            print("句号键: 向下移动 (Z-)")
        
        # ==================== 处理旋转 ====================
        # 1键: X轴正向旋转
        if key_states['1'] and not self.last_key_states.get('1', False):
            axis = np.array([1, 0, 0])  # X轴
            rot_quat = np.array([np.cos(self.rotation_angle/2), *np.sin(self.rotation_angle/2)*axis])
            mujoco.mju_mulQuat(current_quat, rot_quat, current_quat)
            changed = True
            print("1键: 绕X轴正向旋转+10度")
            
        # 2键: X轴负向旋转
        if key_states['2'] and not self.last_key_states.get('2', False):
            axis = np.array([1, 0, 0])  # X轴
            rot_quat = np.array([np.cos(-self.rotation_angle/2), *np.sin(-self.rotation_angle/2)*axis])
            mujoco.mju_mulQuat(current_quat, rot_quat, current_quat)
            changed = True
            print("2键: 绕X轴负向旋转-10度")
            
        # 3键: Y轴正向旋转
        if key_states['3'] and not self.last_key_states.get('3', False):
            axis = np.array([0, 1, 0])  # Y轴
            rot_quat = np.array([np.cos(self.rotation_angle/2), *np.sin(self.rotation_angle/2)*axis])
            mujoco.mju_mulQuat(current_quat, rot_quat, current_quat)
            changed = True
            print("3键: 绕Y轴正向旋转+10度")
            
        # 4键: Y轴负向旋转
        if key_states['4'] and not self.last_key_states.get('4', False):
            axis = np.array([0, 1, 0])  # Y轴
            rot_quat = np.array([np.cos(-self.rotation_angle/2), *np.sin(-self.rotation_angle/2)*axis])
            mujoco.mju_mulQuat(current_quat, rot_quat, current_quat)
            changed = True
            print("4键: 绕Y轴负向旋转-10度")
            
        # 5键: Z轴正向旋转
        if key_states['5'] and not self.last_key_states.get('5', False):
            axis = np.array([0, 0, 1])  # Z轴
            rot_quat = np.array([np.cos(self.rotation_angle/2), *np.sin(self.rotation_angle/2)*axis])
            mujoco.mju_mulQuat(current_quat, rot_quat, current_quat)
            changed = True
            print("5键: 绕Z轴正向旋转+10度")
            
        # 6键: Z轴负向旋转
        if key_states['6'] and not self.last_key_states.get('6', False):
            axis = np.array([0, 0, 1])  # Z轴
            rot_quat = np.array([np.cos(-self.rotation_angle/2), *np.sin(-self.rotation_angle/2)*axis])
            mujoco.mju_mulQuat(current_quat, rot_quat, current_quat)
            changed = True
            print("6键: 绕Z轴负向旋转-10度")
            
        # 更新按键状态
        self.last_key_states = dict(key_states)
        
        if changed:
            # 更新位置和姿态
            data.qpos[:3] = current_pos
            data.qpos[3:7] = current_quat
            # 重置速度以避免物理抖动
            data.qvel[:] = 0
            # 更新物理状态
            mujoco.mj_forward(model, data)
            
            # 打印当前位置和四元数
            print("当前状态:")
            print(f"位置坐标 (x, y, z): ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
            print(f"四元数 (w, x, y, z): ({current_quat[0]:.3f}, {current_quat[1]:.3f}, {current_quat[2]:.3f}, {current_quat[3]:.3f})")
            print("-" * 40)
            
            return True
        return False

ball_control = BallControl()

with mujoco.viewer.launch_passive(model, data) as viewer:
    print("=" * 50)
    print("球体控制说明:")
    print("=" * 50)
    print("【位置移动控制】")
    print("  上箭头: 向前移动 (X+)")
    print("  下箭头: 向后移动 (X-)")
    print("  左箭头: 向左移动 (Y-)")
    print("  右箭头: 向右移动 (Y+)")
    print("  ,: 向上移动 (Z+)")
    print("  .: 向下移动 (Z-)")
    print("【旋转控制】")
    print("  1/2: 绕X轴 正/负 旋转")
    print("  3/4: 绕Y轴 正/负 旋转")
    print("  5/6: 绕Z轴 正/负 旋转")
    print("=" * 50)
    print("每次操作后将显示当前位置坐标和四元数")
    print("=" * 50)

    
    while viewer.is_running():
        # 处理控制
        ball_control.apply_control(key_states)
        
        # 推进仿真
        mujoco.mj_step(model, data)
        viewer.sync()
    