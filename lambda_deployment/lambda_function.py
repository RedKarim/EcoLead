import json
import time
import boto3
import logging
from datetime import datetime

# ロギング設定
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# IoTクライアント初期化
iot_client = boto3.client('iot-data')

def simple_mpc_calculation(state, coeffs, prev_delta, prev_a, ref_v):
    """
    簡単なMPC計算（CasADiなし）
    実際のMPC最適化の代わりに、シンプルなPIDベースの制御を実装
    """
    try:
        # 状態変数を展開
        x, y, psi, v, cte, epsi = state
        
        # PIDゲイン（調整可能）
        kp_cte = 0.5
        kd_cte = 0.1
        kp_epsi = 1.0
        kp_speed = 0.3
        
        # ステアリング制御（クロストラックエラーと方向エラーに基づく）
        optimal_delta = -(kp_cte * cte + kp_epsi * epsi)
        optimal_delta = max(-0.4, min(0.4, optimal_delta))  # ステアリング制限
        
        # 速度制御
        speed_error = ref_v - v
        target_acceleration = kp_speed * speed_error
        target_acceleration = max(-3.0, min(3.0, target_acceleration))  # 加速度制限
        
        # 予測ホライゾン用の配列を作成
        N = 10
        optimal_a = [target_acceleration] * (N - 1)
        mpc_x = [x + i * v * 0.1 for i in range(N)]  # 簡単な予測
        mpc_y = [y + i * cte * 0.1 for i in range(N)]
        
        logger.info(f"Simple MPC: delta={optimal_delta:.3f}, a={target_acceleration:.3f}")
        
        return optimal_delta, optimal_a, mpc_x, mpc_y
        
    except Exception as e:
        logger.error(f"Error in simple MPC calculation: {str(e)}")
        return 0.0, [0.0] * 9, [0.0] * 10, [0.0] * 10

def lambda_handler(event, context):
    """AWS Lambda メインハンドラー"""
    try:
        start_time = time.time()
        logger.info("Lambda MPC function started (Simple Version)")
        logger.info(f"Event: {json.dumps(event)}")
        
        # リクエストデータを抽出
        packet_id = event.get('packet_id', None)
        state = event['state']
        coeffs = event['coeffs']
        prev_delta = event['prev_delta']
        prev_a = event['prev_a']
        ref_v = event['ref_v']
        
        # CARLA timestampを抽出（遅延測定用）
        carla_timestamp = event.get('timestamp', start_time)
        aws_receive_time = time.time()
        carla_to_aws_latency = (aws_receive_time - carla_timestamp) * 1000
        logger.info(f"CARLA to AWS latency: {carla_to_aws_latency:.2f} ms")
        
        logger.info(f"Processing MPC request (Packet ID: {packet_id})")
        
        # 簡単なMPC計算を実行
        optimal_delta, optimal_a, mpc_x, mpc_y = simple_mpc_calculation(
            state, coeffs, prev_delta, prev_a, ref_v
        )
        
        # 結果データを準備
        mpc_result = {
            'packet_id': packet_id,
            'timestamp': time.time(),
            'optimal_delta': float(optimal_delta),
            'optimal_a': [float(a) for a in optimal_a],
            'mpc_x': [float(x) for x in mpc_x],
            'mpc_y': [float(y) for y in mpc_y],
            'processing_time_ms': (time.time() - start_time) * 1000,
            'algorithm': 'simple_pid'  # CasADiの代わりにシンプルPID
        }
        
        logger.info(f"MPC calculated: delta={optimal_delta:.3f}, a[0]={optimal_a[0]:.3f}")
        
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
                'message': 'Simple MPC calculation completed successfully',
                'packet_id': packet_id,
                'processing_time_ms': total_time,
                'algorithm': 'simple_pid'
            })
        }

    except Exception as e:
        logger.error(f"Error in lambda_handler: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps('Error processing MPC request')
        }