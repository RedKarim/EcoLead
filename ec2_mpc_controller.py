#!/usr/bin/env python3
"""
EC2上でのMPCコントローラー
CasADiを使用してMPC計算を実行し、AWS IoT Coreを介してCARLAと通信
"""

import json
import time
import logging
import os
import ssl
from datetime import datetime
import paho.mqtt.client as mqtt
from dotenv import load_dotenv
import numpy as np
import casadi as ca

# ロギング設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/mpc_controller.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class EC2MPCController:
    def __init__(self):
        """EC2 MPC Controller初期化"""
        # 環境変数をロード
        load_dotenv()
        
        # AWS IoT設定
        self.aws_endpoint = os.getenv("AWS_ENDPOINT")
        self.client_id = "ec2-mpc-controller"
        self.ca_path = os.getenv("AWS_CA_PATH", "/home/ec2-user/certs/AmazonRootCA1.pem")
        self.cert_path = os.getenv("AWS_CERT_PATH", "/home/ec2-user/certs/device.pem.crt")
        self.key_path = os.getenv("AWS_KEY_PATH", "/home/ec2-user/certs/private.pem.key")
        
        # MQTT設定
        self.mqtt_client = None
        self.connected = False
        
        # MPC設定
        self.setup_mpc_parameters()
        
        # 統計とパケット追跡
        self.processed_requests = 0
        self.start_time = time.time()
        self.packet_loss_tracker = {}  # パケットIDごとの追跡
        self.expected_packets = set()  # 期待されるパケットID
        self.received_packets = set()  # 受信されたパケットID
        self.total_packets_sent = 0
        self.total_packets_received = 0
        
        logger.info("EC2 MPC Controller initialized")
    
    def setup_mpc_parameters(self):
        """MPC パラメータ設定 - distributed_mpc_controller.pyと同じ"""
        # 車両モデルパラメータ
        self.Lf = 2.0   # 前軸からCGまでの距離 (distributedと同じ)
        self.N = 10     # 予測ホライゾン
        self.dt = 0.1   # タイムステップ
        self.v_max = 15.0  # 最大速度 (distributedと同じ)
        
        # 重み - distributedと同じ値
        self.Q_cte = 1.0      # クロストラックエラー重み
        self.Q_epsi = 1.0     # 方向エラー重み
        self.Q_v = 2.0        # 速度エラー重み (distributedと同じ)
        self.R_delta = 2.0    # ステアリング入力重み (distributedと同じ)
        self.R_a = 5.0        # 加速度入力重み (distributedと同じ)
        self.R_ddelta = 10.0  # ステアリング変化重み (distributedと同じ)
        self.R_da = 1.0       # 加速度変化重み (distributedと同じ)
        
        # 制約
        self.delta_max = np.deg2rad(25)  # 最大ステアリング角 (distributedと同じ)
        self.a_max = 3.0      # 最大加速度
        self.a_min = -3.0     # 最小加速度
        
        logger.info("MPC parameters configured (distributed compatible)")
    

    
    def solve_mpc(self, state, coeffs, ref_v):
        """MPC最適化問題を解く - distributed_mpc_controller.pyと同じロジック"""
        try:
            opti = ca.Opti()
            N = self.N
            
            # 変数定義
            x = opti.variable(N)
            y = opti.variable(N)
            psi = opti.variable(N)
            v = opti.variable(N)
            cte = opti.variable(N)
            epsi = opti.variable(N)
            delta = opti.variable(N - 1)
            a = opti.variable(N - 1)
            
            # 参照速度パラメータ
            ref_v_param = opti.parameter(N)
            if not hasattr(ref_v, '__len__'):
                ref_v_array = np.full(N, ref_v)
            else:
                ref_v_array = ref_v
            opti.set_value(ref_v_param, ref_v_array)
            
            # 初期状態
            x0, y0, psi0, v0, cte0, epsi0 = state
            opti.subject_to(x[0] == x0)
            opti.subject_to(y[0] == y0)
            opti.subject_to(psi[0] == psi0)
            opti.subject_to(v[0] == v0)
            opti.subject_to(cte[0] == cte0)
            opti.subject_to(epsi[0] == epsi0)
            
            # コスト関数 - distributed版と同じ
            cost = 0
            for t in range(N):
                cost += self.Q_cte * ca.power(cte[t], 2)
                cost += self.Q_epsi * ca.power(epsi[t], 2)
                velocity_error = ref_v_param[t] - v[t]
                cost += self.Q_v * ca.power(velocity_error, 2)
            
            # スムーズな遷移のペナルティ
            for t in range(N - 2):
                cost += self.R_da * ca.power(a[t + 1] - a[t], 2)
                cost += self.R_ddelta * ca.power(delta[t + 1] - delta[t], 2)
            
            opti.minimize(cost)
            
            # 車両ダイナミクス制約 - distributed版と同じ
            M_h = 1500.0    # 車両質量 (kg)
            C_D = 0.3       # 抗力係数
            A_v = 2.2       # 前面投影面積 (m^2)
            mu = 0.01       # 転がり抵抗係数
            g = 9.81        # 重力加速度 (m/s^2)
            rho_a = 1.225   # 空気密度 (kg/m^3)
            
            for t in range(1, N):
                x_t0 = x[t - 1]
                y_t0 = y[t - 1]
                psi_t0 = psi[t - 1]
                v_t0 = v[t - 1]
                delta_t0 = delta[t - 1]
                a_t0 = a[t - 1]
                
                # 空気抵抗と転がり抵抗を考慮
                a_drag = (0.5 * C_D * rho_a * A_v * (v_t0**2)) / M_h
                a_roll = mu * g
                a_net = a_t0 - a_drag - a_roll
                
                v_t1 = v_t0 + a_net * self.dt
                x_t1 = x_t0 + v_t0 * ca.cos(psi_t0) * self.dt
                y_t1 = y_t0 + v_t0 * ca.sin(psi_t0) * self.dt
                psi_t1 = psi_t0 + (v_t0 * delta_t0 / self.Lf) * self.dt
                
                # 参照パス
                f_x = coeffs[0] + coeffs[1] * x_t1 + coeffs[2] * (x_t1**2) + coeffs[3] * (x_t1**3)
                psi_des = ca.arctan(coeffs[1] + 2 * coeffs[2] * x_t1 + 3 * coeffs[3] * (x_t1**2))
                
                opti.subject_to(x[t] == x_t1)
                opti.subject_to(y[t] == y_t1)
                opti.subject_to(psi[t] == psi_t1)
                opti.subject_to(v[t] == v_t1)
                opti.subject_to(cte[t] == (f_x - y_t1) + v_t1 * ca.sin(epsi[t-1]) * self.dt)
                opti.subject_to(epsi[t] == (psi_t1 - psi_des) + v_t1 * delta_t0 / self.Lf * self.dt)
            
            # 制約
            opti.subject_to(opti.bounded(-ca.inf, x, ca.inf))
            opti.subject_to(opti.bounded(-ca.inf, y, ca.inf))
            opti.subject_to(opti.bounded(-ca.inf, psi, ca.inf))
            opti.subject_to(opti.bounded(-self.v_max, v, self.v_max))
            opti.subject_to(opti.bounded(-ca.inf, cte, ca.inf))
            opti.subject_to(opti.bounded(-ca.inf, epsi, ca.inf))
            opti.subject_to(opti.bounded(-self.delta_max, delta, self.delta_max))
            opti.subject_to(opti.bounded(self.a_min, a, self.a_max))
            
            # ソルバー設定
            opts = {
                'ipopt.print_level': 0,
                'ipopt.sb': 'yes',
                'print_time': 0,
                'ipopt.max_iter': 1000,
                'ipopt.tol': 1e-6,
                'ipopt.acceptable_tol': 1e-5
            }
            opti.solver('ipopt', opts)
            
            # 求解
            sol = opti.solve()
            
            # 結果取得
            optimal_delta = float(sol.value(delta[0]))
            optimal_a = [float(sol.value(a[k])) for k in range(N-1)]
            mpc_x = [float(sol.value(x[k])) for k in range(N)]
            mpc_y = [float(sol.value(y[k])) for k in range(N)]
            
            return optimal_delta, optimal_a, mpc_x, mpc_y
            
        except Exception as e:
            logger.error(f"MPC solver error: {str(e)}")
            # フォールバック: シンプルPID制御
            return self.fallback_control(state, ref_v)
    
    def fallback_control(self, state, ref_v):
        """MPCが失敗した場合のフォールバック制御"""
        x, y, psi, v, cte, epsi = state
        
        # シンプルPID制御
        kp_cte = 0.5
        kp_epsi = 1.0
        kp_speed = 0.3
        
        optimal_delta = -(kp_cte * cte + kp_epsi * epsi)
        optimal_delta = max(-self.delta_max, min(self.delta_max, optimal_delta))
        
        target_acceleration = kp_speed * (ref_v - v)
        target_acceleration = max(self.a_min, min(self.a_max, target_acceleration))
        
        optimal_a = [target_acceleration] * (self.N - 1)
        mpc_x = [x + i * v * self.dt for i in range(1, self.N + 1)]
        mpc_y = [y + i * cte * self.dt for i in range(1, self.N + 1)]
        
        logger.warning("Using fallback PID control")
        return optimal_delta, optimal_a, mpc_x, mpc_y
    
    def setup_mqtt(self):
        """MQTT接続設定"""
        try:
            self.mqtt_client = mqtt.Client(client_id=self.client_id, protocol=mqtt.MQTTv311)
            
            # SSL設定
            context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
            context.load_verify_locations(self.ca_path)
            context.load_cert_chain(self.cert_path, self.key_path)
            context.check_hostname = False
            context.verify_mode = ssl.CERT_REQUIRED
            
            self.mqtt_client.tls_set_context(context)
            self.mqtt_client.on_connect = self.on_mqtt_connect
            self.mqtt_client.on_message = self.on_mqtt_message
            self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
            
            # 接続
            logger.info(f"Connecting to AWS IoT: {self.aws_endpoint}")
            self.mqtt_client.connect(self.aws_endpoint, 8883, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            logger.error(f"MQTT setup error: {str(e)}")
            raise
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT接続コールバック"""
        if rc == 0:
            self.connected = True
            logger.info("Connected to AWS IoT Core")
            client.subscribe("mpc/request", qos=1)
            logger.info("Subscribed to mpc/request topic")
        else:
            logger.error(f"Failed to connect to AWS IoT, return code {rc}")
    
    def on_mqtt_message(self, client, userdata, msg):
        """MQTT メッセージ受信コールバック"""
        try:
            start_time = time.time()
            data = json.loads(msg.payload.decode())
            
            packet_id = data.get('packet_id', None)
            carla_timestamp = data.get('timestamp', start_time)
            
            # パケット追跡を更新
            self.total_packets_received += 1
            if packet_id is not None:
                self.received_packets.add(packet_id)
                self.packet_loss_tracker[packet_id] = {
                    'received_time': start_time,
                    'carla_timestamp': carla_timestamp,
                    'processed': False
                }
            
            logger.info(f"Received MPC request (Packet ID: {packet_id}) - Total received: {self.total_packets_received}")
            
            # 遅延計算
            receive_latency = (start_time - carla_timestamp) * 1000
            logger.info(f"CARLA to EC2 latency: {receive_latency:.2f} ms")
            
            # MPC計算開始時刻
            mpc_start_time = time.time()
            
            # MPC計算
            state = data['state']
            coeffs = data['coeffs']
            ref_v = data['ref_v']
            prev_delta = data.get('prev_delta', 0.0)
            prev_a = data.get('prev_a', 0.0)
            
            optimal_delta, optimal_a, mpc_x, mpc_y = self.solve_mpc(state, coeffs, ref_v)
            
            # MPC計算時間
            mpc_processing_time = (time.time() - mpc_start_time) * 1000
            total_processing_time = (time.time() - start_time) * 1000
            
            # パケット処理完了をマーク
            if packet_id is not None and packet_id in self.packet_loss_tracker:
                self.packet_loss_tracker[packet_id]['processed'] = True
                self.packet_loss_tracker[packet_id]['mpc_processing_time'] = mpc_processing_time
            
            # 結果送信
            result = {
                'packet_id': packet_id,
                'timestamp': time.time(),
                'carla_timestamp': carla_timestamp,  # 元のCARLAタイムスタンプを含める
                'optimal_delta': optimal_delta,
                'optimal_a': optimal_a,
                'mpc_x': mpc_x,
                'mpc_y': mpc_y,
                'processing_time_ms': total_processing_time,
                'mpc_processing_time_ms': mpc_processing_time,
                'receive_latency_ms': receive_latency,
                'algorithm': 'casadi_mpc',
                'ec2_instance_id': 'i-07a7ae42958b716de'  # EC2インスタンスID
            }
            
            # 結果を送信
            publish_start = time.time()
            client.publish("mpc/result", json.dumps(result), qos=1)
            publish_time = (time.time() - publish_start) * 1000
            
            self.processed_requests += 1
            self.total_packets_sent += 1
            
            logger.info(f"Sent MPC result (Packet ID: {packet_id}) - "
                       f"MPC: {mpc_processing_time:.2f}ms, "
                       f"Total: {total_processing_time:.2f}ms, "
                       f"Publish: {publish_time:.2f}ms - "
                       f"Processed: {self.processed_requests}")
            
            # パケットロス統計を定期的にログ
            if self.processed_requests % 50 == 0:
                self.log_packet_statistics()
            
        except Exception as e:
            logger.error(f"Error processing MPC request: {str(e)}")
            # エラー時もパケット統計を更新
            if 'packet_id' in locals() and packet_id is not None:
                self.packet_loss_tracker[packet_id] = {
                    'received_time': start_time,
                    'processed': False,
                    'error': str(e)
                }
    
    def log_packet_statistics(self):
        """パケットロス統計をログ出力"""
        try:
            if self.total_packets_received > 0:
                processed_count = sum(1 for p in self.packet_loss_tracker.values() if p.get('processed', False))
                error_count = sum(1 for p in self.packet_loss_tracker.values() if 'error' in p)
                
                # 平均遅延時間計算
                latencies = []
                mpc_times = []
                for packet_data in self.packet_loss_tracker.values():
                    if packet_data.get('processed', False):
                        receive_time = packet_data.get('received_time', 0)
                        carla_time = packet_data.get('carla_timestamp', 0)
                        if receive_time > 0 and carla_time > 0:
                            latencies.append((receive_time - carla_time) * 1000)
                        
                        mpc_time = packet_data.get('mpc_processing_time', 0)
                        if mpc_time > 0:
                            mpc_times.append(mpc_time)
                
                avg_latency = sum(latencies) / len(latencies) if latencies else 0
                avg_mpc_time = sum(mpc_times) / len(mpc_times) if mpc_times else 0
                
                # パケットロス率計算（簡単な近似）
                packet_loss_rate = (error_count / self.total_packets_received) * 100 if self.total_packets_received > 0 else 0
                
                logger.info(f"=== Packet Statistics ===")
                logger.info(f"Total Received: {self.total_packets_received}")
                logger.info(f"Successfully Processed: {processed_count}")
                logger.info(f"Errors: {error_count}")
                logger.info(f"Packet Loss Rate: {packet_loss_rate:.2f}%")
                logger.info(f"Average CARLA→EC2 Latency: {avg_latency:.2f} ms")
                logger.info(f"Average MPC Processing Time: {avg_mpc_time:.2f} ms")
                logger.info(f"========================")
                
        except Exception as e:
            logger.error(f"Error logging packet statistics: {str(e)}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT切断コールバック"""
        self.connected = False
        if rc != 0:
            logger.warning(f"Unexpected MQTT disconnection: {rc}")
        else:
            logger.info("MQTT disconnected")
    
    def run(self):
        """メインループ"""
        logger.info("Starting EC2 MPC Controller")
        
        try:
            self.setup_mqtt()
            
            # 接続待機
            while not self.connected:
                time.sleep(0.1)
            
            logger.info("EC2 MPC Controller is ready")
            
            # メインループ
            while True:
                time.sleep(10)
                uptime = time.time() - self.start_time
                logger.info(f"Uptime: {uptime/3600:.1f}h, Processed: {self.processed_requests} requests, "
                           f"Received: {self.total_packets_received}, Sent: {self.total_packets_sent}")
                
                # 5分ごとに詳細統計をログ出力
                if int(uptime) % 300 == 0 and self.processed_requests > 0:
                    self.log_packet_statistics()
                
        except KeyboardInterrupt:
            logger.info("Shutting down EC2 MPC Controller")
        except Exception as e:
            logger.error(f"Fatal error: {str(e)}")
        finally:
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()

if __name__ == "__main__":
    controller = EC2MPCController()
    controller.run()
