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
# !\ split_decision_buffer.py
#
# \date    2025-11-09
#
# スプリット決定のレイテンシー対応バッファ
# ---------------------------------------------------------------------

from collections import deque
from dataclasses import dataclass
import time

@dataclass
class SplitDecision:
    """タイムスタンプ付きスプリット決定"""
    timestamp: float  # 実行予定時刻
    decision: dict  # スプリット決定内容
    computation_time: float  # 計算時刻

class SplitDecisionBuffer:
    """スプリット決定をレイテンシーに対応して管理するバッファ"""
    
    def __init__(self, latency=0.8):
        self.buffer = deque(maxlen=10)
        self.latency = latency
        self.last_decision = None
        
    def add_decision(self, decision, current_time):
        """
        スプリット決定をバッファに追加
        
        Args:
            decision: スプリット決定の辞書
            current_time: 現在時刻（計算時刻）
        """
        # 計算時刻を記録（実行は後でlatency分遅延）
        split_decision = SplitDecision(
            timestamp=current_time,  # 計算時刻
            decision=decision,
            computation_time=current_time
        )
        self.buffer.append(split_decision)
        
        print(f"[SplitBuffer] 決定追加: mode={decision.get('mode')}, comp_time={current_time:.2f}")
    
    def get_decision(self, current_time):
        """
        現在時刻に実行すべきスプリット決定を取得（レイテンシー分古い決定を使用）
        
        Args:
            current_time: 現在時刻
            
        Returns:
            tuple: (decision_dict, should_execute)
        """
        if not self.buffer:
            return None, False
        
        # レイテンシー分過去の決定を探す
        target_time = current_time - self.latency
        
        # 実行すべき決定を探す
        for decision_obj in self.buffer:
            # 計算時刻がtarget_time付近で、まだ実行していない決定
            decision_age = current_time - decision_obj.computation_time
            
            # 決定が十分古い（レイテンシー経過）かつ未実行
            if decision_age >= self.latency and decision_obj != self.last_decision:
                self.last_decision = decision_obj
                print(f"[SplitBuffer] 決定実行: mode={decision_obj.decision.get('mode')}, " 
                      f"comp_time={decision_obj.computation_time:.2f}, age={decision_age:.3f}s")
                
                # 古い決定を削除
                while self.buffer and self.buffer[0].computation_time < current_time - self.latency - 1.0:
                    old_decision = self.buffer.popleft()
                
                return decision_obj.decision, True
        
        # レイテンシー期間中または実行すべき決定なし
        return None, False
    
    def clear(self):
        """バッファをクリア"""
        self.buffer.clear()
        self.last_decision = None

