#!/bin/bash
# EC2インスタンス上でMPCコントローラーをセットアップするスクリプト

set -e

echo "=== EC2 MPC Controller Setup ==="

# システム更新
echo "Updating system packages..."
sudo dnf update -y

# Python開発環境インストール
echo "Installing Python development environment..."
sudo dnf install -y python3 python3-pip python3-devel gcc gcc-c++ cmake git

# 必要なシステムライブラリをインストール
echo "Installing system libraries..."
sudo dnf install -y lapack-devel blas-devel

# Pythonパッケージインストール
echo "Installing Python packages..."
pip3 install --user numpy scipy casadi paho-mqtt python-dotenv boto3

# 作業ディレクトリ作成
echo "Creating working directories..."
mkdir -p /home/ec2-user/mpc_controller
mkdir -p /home/ec2-user/certs
mkdir -p /home/ec2-user/logs

# ログディレクトリ設定
sudo mkdir -p /var/log
sudo chown ec2-user:ec2-user /var/log

# 環境変数ファイル作成
echo "Creating environment file..."
cat > /home/ec2-user/.env << EOF
# AWS IoT Core設定
AWS_ENDPOINT=YOUR_IOT_ENDPOINT
CLIENT_ID=ec2-mpc-controller
AWS_CA_PATH=/home/ec2-user/certs/AmazonRootCA1.pem
AWS_CERT_PATH=/home/ec2-user/certs/device.pem.crt
AWS_KEY_PATH=/home/ec2-user/certs/private.pem.key
EOF

# systemdサービスファイル作成
echo "Creating systemd service..."
sudo tee /etc/systemd/system/mpc-controller.service > /dev/null << EOF
[Unit]
Description=CARLA MPC Controller
After=network.target
Wants=network.target

[Service]
Type=simple
User=ec2-user
Group=ec2-user
WorkingDirectory=/home/ec2-user/mpc_controller
Environment=PATH=/home/ec2-user/.local/bin:/usr/local/bin:/usr/bin:/bin
ExecStart=/usr/bin/python3 /home/ec2-user/mpc_controller/ec2_mpc_controller.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# systemdリロード
sudo systemctl daemon-reload

echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Upload your AWS IoT certificates to /home/ec2-user/certs/"
echo "2. Update /home/ec2-user/.env with your AWS IoT endpoint"
echo "3. Upload ec2_mpc_controller.py to /home/ec2-user/mpc_controller/"
echo "4. Start the service: sudo systemctl start mpc-controller"
echo "5. Enable auto-start: sudo systemctl enable mpc-controller"
echo ""
echo "Monitor logs with: sudo journalctl -u mpc-controller -f"
