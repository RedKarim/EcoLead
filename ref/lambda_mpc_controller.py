import json
import numpy as np
import casadi as ca
import boto3
import os
import logging
import time
from datetime import datetime

# ロギング設定
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# IoTクライアント初期化
iot_client = boto3.client('iot-data')

class LambdaMPCController:
    def __init__(self, mpc_config, weights, fuel_coeffs):
        """AWS Lambda用MPCコントローラー初期化"""
        
        # MPCパラメータ
        self.N = mpc_config.get('N', 10)
        self.dt = mpc_config.get('dt', 0.1)
        self.ref_v = mpc_config.get('ref_v', 11.1)
        self.a_max = mpc_config.get('a_max', 3.0)
        steer_max_deg = mpc_config.get('steer_max_deg', 25)
        self.steer_max = np.deg2rad(steer_max_deg)
        self.v_max = mpc_config.get('v_max', 15.0)

        # コスト関数重み
        self.weight_cte = weights.get('cte', 1.0)
        self.weight_epsi = weights.get('epsi', 1.0)
        self.weight_v = weights.get('v', 2.0)
        self.weight_delta = weights.get('delta', 2.0)
        self.weight_a = weights.get('a', 5.0)
        self.weight_diff_delta = weights.get('diff_delta', 10.0)
        self.weight_diff_a = weights.get('diff_a', 1.0)
        self.weight_fuel = weights.get('fuel', 1000.0)
        self.weight_diff_fuel = weights.get('diff_fuel', 10.0)
        self.weight_distance = weights.get('dist', 1.0)

        # 燃費係数
        self.b0 = fuel_coeffs.get('b0', 0.0)
        self.b1 = fuel_coeffs.get('b1', 0.0)
        self.b2 = fuel_coeffs.get('b2', 0.0)
        self.b3 = fuel_coeffs.get('b3', 0.0)
        self.c0 = fuel_coeffs.get('c0', 0.0)
        self.c1 = fuel_coeffs.get('c1', 0.0)
        self.c2 = fuel_coeffs.get('c2', 0.0)
        self.F_idle = fuel_coeffs.get('F_idle', 0.0)

        # 物理パラメータ
        self.M_h = 1500.0    # 車両重量 (kg)
        self.C_D = 0.3       # 空気抵抗係数
        self.A_v = 2.2       # 前面投影面積 (m^2)
        self.mu = 0.01       # 転がり抵抗係数
        self.g = 9.81        # 重力加速度 (m/s^2)
        self.rho_a = 1.225   # 空気密度 (kg/m^3)
        self.theta = 0.0     # 路面勾配角 (radians)
        self.Lf = 2          # 前輪軸距離

        # 前回解の保存（ウォームスタート用）
        self.previous_solution = {
            'x': None, 'y': None, 'psi': None, 'v': None,
            'cte': None, 'epsi': None, 'delta': None, 'a': None,
        }

        logger.info("Lambda MPC Controller initialized")

    def setup_cost_function(self, opti, variables):
        """コスト関数設定"""
        x, y, psi, v, cte, epsi, delta, a, ref_v = variables
        N = self.N
        cost = 0
        
        # エラーと基準速度からの偏差にペナルティ
        for t in range(N):
            cost += self.weight_cte * ca.power(cte[t], 2)
            cost += self.weight_epsi * ca.power(epsi[t], 2)
            velocity_error = ref_v[t] - v[t]
            cost += self.weight_v * ca.power(velocity_error, 2)

        # スムーズな遷移にペナルティ
        for t in range(N - 2):
            cost += self.weight_diff_a * ca.power(a[t + 1] - a[t], 2)
            cost += self.weight_diff_delta * ca.power(delta[t + 1] - delta[t], 2)

        opti.minimize(cost)
        logger.info("Cost function set up")

    def setup_constraints(self, opti, variables, state, coeffs):
        """制約条件設定"""
        x, y, psi, v, cte, epsi, delta, a, ref_v = variables
        N = self.N
        x0, y0, psi0, v0, cte0, epsi0 = state

        # 初期条件
        opti.subject_to(x[0] == x0)
        opti.subject_to(y[0] == y0)
        opti.subject_to(psi[0] == psi0)
        opti.subject_to(v[0] == v0)
        opti.subject_to(cte[0] == cte0)
        opti.subject_to(epsi[0] == epsi0)

        for t in range(1, N):
            x_t0 = x[t - 1]
            y_t0 = y[t - 1]
            psi_t0 = psi[t - 1]
            v_t0 = v[t - 1]
            delta_t0 = delta[t - 1]
            a_t0 = a[t - 1]

            # 空気抵抗と転がり抵抗を考慮した動力学
            a_drag = (0.5 * self.C_D * self.rho_a * self.A_v * (v_t0**2)) / self.M_h
            a_roll = self.mu * self.g
            a_net = a_t0 - a_drag - a_roll

            v_t1 = v_t0 + a_net * self.dt
            x_t1 = x_t0 + v_t0 * ca.cos(psi_t0) * self.dt
            y_t1 = y_t0 + v_t0 * ca.sin(psi_t0) * self.dt
            psi_t1 = psi_t0 + (v_t0 * delta_t0 / self.Lf) * self.dt

            # 基準パスの多項式フィット
            f_x = coeffs[0] + coeffs[1] * x_t1 + coeffs[2] * (x_t1**2) + coeffs[3] * (x_t1**3)
            psi_des = ca.arctan(coeffs[1] + 2 * coeffs[2] * x_t1 + 3 * coeffs[3] * (x_t1**2))

            opti.subject_to(x[t] == x_t1)
            opti.subject_to(y[t] == y_t1)
            opti.subject_to(psi[t] == psi_t1)
            opti.subject_to(v[t] == v_t1)
            opti.subject_to(cte[t] == (f_x - y_t1) + v_t1 * ca.sin(epsi[t-1]) * self.dt)
            opti.subject_to(epsi[t] == (psi_t1 - psi_des) + v_t1 * delta_t0 / (2 * self.Lf) * self.dt)
        
        logger.info("Constraints set up")

    def solve(self, state, coeffs, prev_delta, prev_a, ref_v):
        """MPC問題を解く"""
        opti = ca.Opti()

        N = self.N
        x = opti.variable(N)
        y = opti.variable(N)
        psi = opti.variable(N)
        v = opti.variable(N)
        cte = opti.variable(N)
        epsi = opti.variable(N)
        delta = opti.variable(N - 1)
        a = opti.variable(N - 1)
        
        # 基準速度パラメータ配列作成
        ref_v_param = opti.parameter(N)
        if not hasattr(ref_v, '__len__'):
            ref_v_array = np.full(N, ref_v)
        else:
            ref_v_array = ref_v
        opti.set_value(ref_v_param, ref_v_array)

        # 変数グループ化
        variables = (x, y, psi, v, cte, epsi, delta, a, ref_v_param)
        self.setup_cost_function(opti, variables)
        self.setup_constraints(opti, variables, state, coeffs)

        # 境界条件
        opti.subject_to(opti.bounded(-ca.inf, x, ca.inf))
        opti.subject_to(opti.bounded(-ca.inf, y, ca.inf))
        opti.subject_to(opti.bounded(-ca.inf, psi, ca.inf))
        opti.subject_to(opti.bounded(-self.v_max, v, self.v_max))
        opti.subject_to(opti.bounded(-ca.inf, cte, ca.inf))
        opti.subject_to(opti.bounded(-ca.inf, epsi, ca.inf))

        # アクチュエータ制約
        opti.subject_to(opti.bounded(-self.steer_max, delta, self.steer_max))
        opti.subject_to(opti.bounded(-self.a_max, a, self.a_max))

        # ウォームスタート使用
        if self.previous_solution['x'] is not None:
            opti.set_initial(x, self.previous_solution['x'])
            opti.set_initial(y, self.previous_solution['y'])
            opti.set_initial(psi, self.previous_solution['psi'])
            opti.set_initial(v, self.previous_solution['v'])
            opti.set_initial(cte, self.previous_solution['cte'])
            opti.set_initial(epsi, self.previous_solution['epsi'])
            opti.set_initial(delta, self.previous_solution['delta'])
            opti.set_initial(a, self.previous_solution['a'])
            logger.info("Warm start used")
        else:
            logger.info("No warm start available")
        
        opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': False,
            'ipopt.max_iter': 1000,
            'ipopt.tol': 1e-6,
            'ipopt.acceptable_tol': 1e-5
        }
        opti.solver('ipopt', opts)

        try:
            sol = opti.solve()
        except RuntimeError:
            logger.error("Solver failed!")
            return None
            
        # 次回用に解を保存
        self.previous_solution['x'] = sol.value(x)
        self.previous_solution['y'] = sol.value(y)
        self.previous_solution['psi'] = sol.value(psi)
        self.previous_solution['v'] = sol.value(v)
        self.previous_solution['cte'] = sol.value(cte)
        self.previous_solution['epsi'] = sol.value(epsi)
        self.previous_solution['delta'] = sol.value(delta)
        self.previous_solution['a'] = sol.value(a)

        opt_delta = sol.value(delta[0])
        opt_a = sol.value(a)
        mpc_x = sol.value(x)
        mpc_y = sol.value(y)

        return opt_delta, opt_a, mpc_x, mpc_y

def lambda_handler(event, context):
    """AWS Lambda メインハンドラー"""
    try:
        start_time = time.time()
        logger.info("Lambda MPC function started")
        
        # 環境変数からMPC設定を取得
        mpc_config = {
            'N': int(os.environ.get('MPC_N', '10')),
            'dt': float(os.environ.get('MPC_DT', '0.1')),
            'ref_v': float(os.environ.get('MPC_REF_V', '11.1')),
            'a_max': float(os.environ.get('MPC_A_MAX', '3.0')),
            'steer_max_deg': float(os.environ.get('MPC_STEER_MAX_DEG', '25')),
            'v_max': float(os.environ.get('MPC_V_MAX', '15.0'))
        }
        
        weights = {
            'cte': float(os.environ.get('WEIGHT_CTE', '1.0')),
            'epsi': float(os.environ.get('WEIGHT_EPSI', '1.0')),
            'v': float(os.environ.get('WEIGHT_V', '2.0')),
            'delta': float(os.environ.get('WEIGHT_DELTA', '2.0')),
            'a': float(os.environ.get('WEIGHT_A', '5.0')),
            'diff_delta': float(os.environ.get('WEIGHT_DIFF_DELTA', '10.0')),
            'diff_a': float(os.environ.get('WEIGHT_DIFF_A', '1.0')),
            'fuel': float(os.environ.get('WEIGHT_FUEL', '1000.0')),
            'diff_fuel': float(os.environ.get('WEIGHT_DIFF_FUEL', '10.0')),
            'dist': float(os.environ.get('WEIGHT_DIST', '1.0'))
        }
        
        fuel_coeffs = {
            'b0': float(os.environ.get('FUEL_B0', '0.0')),
            'b1': float(os.environ.get('FUEL_B1', '0.0')),
            'b2': float(os.environ.get('FUEL_B2', '0.0')),
            'b3': float(os.environ.get('FUEL_B3', '0.0')),
            'c0': float(os.environ.get('FUEL_C0', '0.0')),
            'c1': float(os.environ.get('FUEL_C1', '0.0')),
            'c2': float(os.environ.get('FUEL_C2', '0.0')),
            'F_idle': float(os.environ.get('FUEL_F_IDLE', '0.0'))
        }

        # MPCコントローラー初期化
        controller = LambdaMPCController(mpc_config, weights, fuel_coeffs)

        # IoT Coreからリクエストデータを取得
        try:
            # IoT shadowまたはtopic経由でデータを受信する場合
            if 'Records' in event and event['Records']:
                # S3トリガーの場合
                s3 = boto3.client('s3')
                bucket = event['Records'][0]['s3']['bucket']['name']
                key = event['Records'][0]['s3']['object']['key']
                
                response = s3.get_object(Bucket=bucket, Key=key)
                request_data = json.loads(response['Body'].read().decode('utf-8'))
            else:
                # 直接IoT経由の場合
                request_data = event
            
            # MPC計算に必要なデータを抽出
            packet_id = request_data.get('packet_id', None)
            state = request_data['state']
            coeffs = request_data['coeffs']
            prev_delta = request_data['prev_delta']
            prev_a = request_data['prev_a']
            ref_v = request_data['ref_v']
            
            # CARLA timestampを抽出（遅延測定用）
            carla_timestamp = request_data.get('timestamp', start_time)
            aws_receive_time = time.time()
            carla_to_aws_latency = (aws_receive_time - carla_timestamp) * 1000
            logger.info(f"CARLA to AWS latency: {carla_to_aws_latency:.2f} ms")
            
            logger.info(f"Processing MPC request (Packet ID: {packet_id})")
            
        except Exception as e:
            logger.error(f"Error parsing request data: {str(e)}")
            return {
                'statusCode': 400,
                'body': json.dumps('Invalid request data format')
            }

        # MPC問題を解く
        try:
            result = controller.solve(state, coeffs, prev_delta, prev_a, ref_v)
            
            if result is not None:
                optimal_delta, optimal_a, mpc_x, mpc_y = result
                
                # 結果データを準備
                mpc_result = {
                    'packet_id': packet_id,
                    'timestamp': time.time(),
                    'optimal_delta': float(optimal_delta),
                    'optimal_a': [float(a) for a in optimal_a],
                    'mpc_x': [float(x) for x in mpc_x],
                    'mpc_y': [float(y) for y in mpc_y],
                    'processing_time_ms': (time.time() - start_time) * 1000
                }
                
                logger.info(f"MPC solved: delta={optimal_delta:.3f}, a[0]={optimal_a[0]:.3f}")
                
            else:
                # ソルバー失敗時のフォールバック
                logger.error("MPC solver failed, using safe fallback")
                N = mpc_config['N']
                mpc_result = {
                    'packet_id': packet_id,
                    'timestamp': time.time(),
                    'optimal_delta': 0.0,
                    'optimal_a': [0.0] * (N - 1),
                    'mpc_x': [0.0] * N,
                    'mpc_y': [0.0] * N,
                    'processing_time_ms': (time.time() - start_time) * 1000
                }
                
        except Exception as e:
            logger.error(f"Error solving MPC: {str(e)}")
            return {
                'statusCode': 500,
                'body': json.dumps('Error solving MPC problem')
            }

        # 結果をIoT Coreに送信
        try:
            iot_client.publish(
                topic='mpc/result',
                qos=1,
                payload=json.dumps(mpc_result)
            )
            logger.info(f"Published MPC result (Packet ID: {packet_id})")
            
        except Exception as e:
            logger.error(f"Error publishing result: {str(e)}")
            return {
                'statusCode': 500,
                'body': json.dumps('Error publishing MPC result')
            }

        # 総処理時間を計算
        total_time = (time.time() - start_time) * 1000
        logger.info(f"Total processing time: {total_time:.2f} ms")

        return {
            'statusCode': 200,
            'body': json.dumps({
                'message': 'MPC calculation completed successfully',
                'packet_id': packet_id,
                'processing_time_ms': total_time
            })
        }

    except Exception as e:
        logger.error(f"Error in lambda_handler: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps('Error processing MPC request')
        }
