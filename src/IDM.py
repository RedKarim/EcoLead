# Copyright (c) 2025, MAHDYAR KARIMI
# Author: MAHDYAR KARIMI, 2025-08-31 

def IDM(Xh: float, Vh: float, Xp: float, Vp: float) -> float:
    """
    Intelligent Driver Model (IDM) 関数
    Xh: 現在車両の位置
    Vh: 現在車両の速度  
    Xp: 前車の位置（または停止位置）
    Vp: 前車の速度（または停止速度）
    
    MATLABコードの参考実装に基づく信号機対応版
    """
    # IDMパラメータ（より積極的な設定）
    T = 0.8    # 希望時間間隔 (s) - さらに短縮
    Vd = 16    # 希望速度 (m/s) - 増加
    S0 = 0.5   # 最小車間距離 (m) - さらに短縮
    a = 2.0    # 最大加速度 (m/s²) - 増加
    b = 4.0    # 快適減速度 (m/s²) - さらに増加（より強いブレーキ）
    L = 1.5    # 車長 (m) - さらに短縮（より近くで停止）
    
    # 車間距離を計算
    DXh = Xp - Xh - L
    
    # 最小車間距離を保証
    if DXh < S0:
        DXh = S0
    
    # 希望車間距離を計算
    if (a * b) > 0:
        Rd = S0 + Vh * T + (Vh * (Vh - Vp)) / (2 * (a * b) ** 0.5)
    else:
        Rd = S0 + Vh * T
    
    # IDM加速度を計算
    if Vd > 0 and DXh > 0:
        speed_term = (Vh / Vd) ** 4
        distance_term = (Rd / DXh) ** 2
        acc = a * (1 - speed_term - distance_term)
        
        print(f"[IDM DEBUG] Xh={Xh}, Vh={Vh:.1f}, Xp={Xp}, Vp={Vp}")
        print(f"[IDM DEBUG] DXh={DXh:.1f}, Rd={Rd:.1f}, speed_term={speed_term:.3f}, distance_term={distance_term:.3f}")
        print(f"[IDM DEBUG] Raw acceleration: {acc:.2f} m/s²")
    else:
        acc = -b  # 緊急ブレーキ
        print(f"[IDM DEBUG] Emergency brake: Vd={Vd}, DXh={DXh}, acc={acc}")
    
    # 加速度を制限
    if acc < -5:
        acc = -5
    elif acc > 4:
        acc = 4
    
    print(f"[IDM DEBUG] Final acceleration: {acc:.2f} m/s²")
    return acc
