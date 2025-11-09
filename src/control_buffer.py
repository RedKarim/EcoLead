# -- BEGIN LICENSE BLOCK ----------------------------------------------
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
#    and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# -- END LICENSE BLOCK ------------------------------------------------
# ---------------------------------------------------------------------
# !\ control_buffer.py
#
# \date    2025-11-09
#
# レイテンシー対応制御信号バッファ
# ---------------------------------------------------------------------

from collections import deque
from dataclasses import dataclass
import time
import numpy as np

@dataclass
class ControlSignal:
    """タイムスタンプ付き制御信号"""
    timestamp: float  # 適用予定時刻
    delta: float  # ステアリング角度 (rad)
    acceleration: float  # 加速度 (m/s^2)
    computation_time: float  # 計算時刻

class ControlBuffer:
    """MPC計算結果を保存し、レイテンシーに対応する制御信号バッファ"""
    
    def __init__(self, buffer_size=20, dt=0.1):
        self.buffer = deque(maxlen=buffer_size)
        self.dt = dt
        self.last_computation_time = 0.0
        self.latency = 0.0
        
    def add_control_sequence(self, delta_sequence, accel_sequence, current_time, latency):
        """
        MPCから計算された制御信号シーケンスをバッファに追加
        
        Args:
            delta_sequence: ステアリング角度の配列
            accel_sequence: 加速度の配列
            current_time: 現在時刻（計算開始時刻）
            latency: レイテンシー時間 (秒)
        """
        self.latency = latency
        self.last_computation_time = current_time
        
        # バッファをクリア（新しい計算結果で上書き）
        self.buffer.clear()
        
        # 各制御信号にタイムスタンプを付けてバッファに追加
        # 信号は計算時刻から各ステップ分先の時刻用
        for i, (delta, accel) in enumerate(zip(delta_sequence, accel_sequence)):
            # 適用時刻 = 計算開始時刻 + 各ステップのオフセット
            # レイテンシーは別途、取得時にチェック
            application_time = current_time + (i * self.dt)
            
            signal = ControlSignal(
                timestamp=application_time,
                delta=float(delta),
                acceleration=float(accel),
                computation_time=current_time
            )
            self.buffer.append(signal)
        
        # デバッグ: 最初の数個の信号を表示
        delta_vals = [f"{d:.4f}" for d in delta_sequence[:3]]
        print(f"[ControlBuffer] {len(self.buffer)} 個の制御信号をバッファに追加 (latency={latency*1000:.0f}ms)")
        print(f"[ControlBuffer]   最初のdelta値: {delta_vals}, comp_time={current_time:.2f}, apply_times: [{current_time:.2f}..{current_time+(len(delta_sequence)-1)*self.dt:.2f}]")
    
    def get_control_signal(self, current_time):
        """
        現在時刻に対応する制御信号を取得（レイテンシー分古い信号を使用）
        
        Args:
            current_time: 現在時刻
            
        Returns:
            tuple: (delta, acceleration, is_valid)
        """
        if not self.buffer:
            print("[ControlBuffer] バッファが空です")
            return 0.0, 0.0, False
        
        # レイテンシー分過去の時刻の信号を探す
        # 現在t=1.0sなら、t=0.2s(1.0-0.8)に計算された信号を使う
        target_time = current_time - self.latency
        
        best_signal = None
        min_time_diff = float('inf')
        
        for signal in self.buffer:
            # 信号の計算時刻がtarget_time付近のものを探す
            time_diff = abs(signal.computation_time - target_time)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                best_signal = signal
        
        if best_signal is None:
            # フォールバック: 最初の信号を使用
            best_signal = self.buffer[0]
            print("[ControlBuffer] 適切な信号が見つからず、最初の信号を使用")
            return best_signal.delta, best_signal.acceleration, False
        
        # レイテンシーチェック: 信号が十分古いかチェック
        signal_age = current_time - best_signal.computation_time
        if signal_age < self.latency:
            # まだレイテンシー期間中（信号が到着していない）
            # しかし、初期化期間中は最新の信号を使う（車両を動かすため）
            print(f"[ControlBuffer] 信号未到着 (age={signal_age:.3f}s < latency={self.latency}s) - 最新信号を使用")
            # 最新の信号を探す
            latest_signal = max(self.buffer, key=lambda s: s.computation_time)
            print(f"[ControlBuffer] 最新信号使用: comp_t={latest_signal.computation_time:.2f}, delta={latest_signal.delta:.4f}")
            return latest_signal.delta, latest_signal.acceleration, True
        
        # 使用済み信号を削除（過去の信号）
        while self.buffer and self.buffer[0].computation_time < current_time - self.latency - 1.0:
            old_signal = self.buffer.popleft()
        
        print(f"[ControlBuffer] 制御信号取得: current_t={current_time:.2f}, target_t={target_time:.2f}, " 
              f"signal_comp_t={best_signal.computation_time:.2f}, signal_apply_t={best_signal.timestamp:.2f}, "
              f"delta={best_signal.delta:.4f}, a={best_signal.acceleration:.3f}, "
              f"age={signal_age:.3f}s, buffer_size={len(self.buffer)}")
        
        return best_signal.delta, best_signal.acceleration, True
    
    def has_valid_signal(self, current_time):
        """
        現在時刻に対して有効な制御信号があるかチェック
        
        Args:
            current_time: 現在時刻
            
        Returns:
            bool: 有効な信号が存在するか
        """
        if not self.buffer:
            return False
        
        # 最も古い信号と新しい信号のタイムスタンプをチェック
        oldest = self.buffer[0].timestamp
        newest = self.buffer[-1].timestamp
        
        # 現在時刻がバッファの範囲内にあるか
        return oldest <= current_time <= newest + self.dt
    
    def get_buffer_status(self):
        """バッファの状態を取得（デバッグ用）"""
        if not self.buffer:
            return {
                'size': 0,
                'time_range': (0, 0),
                'latency': self.latency
            }
        
        return {
            'size': len(self.buffer),
            'time_range': (self.buffer[0].timestamp, self.buffer[-1].timestamp),
            'latency': self.latency,
            'last_computation': self.last_computation_time
        }

