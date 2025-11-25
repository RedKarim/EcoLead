# EC2 MPC Controller Deployment Guide

This guide provides all the commands needed to deploy and manage the MPC controller on AWS EC2.

## 📋 Prerequisites

- AWS EC2 instance running (current: `52.198.213.153`)
- SSH key file: `carla-mpc-key.pem` in the project root
- EC2 instance has systemd service configured

## 🚀 Quick Deployment Commands

### 1. Deploy Updated Controller

```bash
# From the EcoLead project root directory
scp -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2_mpc_controller.py ec2-user@52.198.213.153:/home/ec2-user/mpc_controller/
```

### 2. Restart the Service

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl restart mpc-controller"
```

### 3. Check Service Status

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl status mpc-controller --no-pager -n 10"
```

## 🔄 Complete Deployment Workflow

When you modify `ec2_mpc_controller.py`, run these commands in sequence:

```bash
# 1. Deploy the updated file
scp -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2_mpc_controller.py ec2-user@52.198.213.153:/home/ec2-user/mpc_controller/

# 2. Restart the service
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl restart mpc-controller"

# 3. Verify it's running
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl status mpc-controller --no-pager -n 8"
```

## 🔍 Monitoring & Debugging

### View Real-time Logs

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo journalctl -u mpc-controller -f"
```

### View Recent Logs

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo journalctl -u mpc-controller -n 50 --no-pager"
```

### Check Log File

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "tail -f /var/log/mpc_controller.log"
```

### Check Service Configuration

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl show mpc-controller"
```

## 🛠️ Service Management Commands

### Start Service

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl start mpc-controller"
```

### Stop Service

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl stop mpc-controller"
```

### Enable Auto-start (on boot)

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl enable mpc-controller"
```

### Disable Auto-start

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl disable mpc-controller"
```

## 📁 File Locations on EC2

- **Controller Script**: `/home/ec2-user/mpc_controller/ec2_mpc_controller.py`
- **Environment File**: `/home/ec2-user/mpc_controller/.env`
- **Certificates**: `/home/ec2-user/certs/`
- **Service File**: `/etc/systemd/system/mpc-controller.service`
- **Log File**: `/var/log/mpc_controller.log`

## 🔧 Troubleshooting Commands

### Check Python Dependencies

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "python3 -c 'import casadi, numpy, paho.mqtt.client; print(\"All dependencies OK\")'"
```

### Test AWS IoT Connection

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "cd /home/ec2-user/mpc_controller && python3 -c 'from dotenv import load_dotenv; load_dotenv(); import os; print(f\"Endpoint: {os.getenv(\"AWS_ENDPOINT\")}\")"
```

### Check Certificate Files

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "ls -la /home/ec2-user/certs/"
```

### Manual Test Run (Debug Mode)

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "cd /home/ec2-user/mpc_controller && python3 ec2_mpc_controller.py"
```

## ⚡ One-Line Deployment

For quick updates, use this single command:

```bash
scp -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2_mpc_controller.py ec2-user@52.198.213.153:/home/ec2-user/mpc_controller/ && ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "sudo systemctl restart mpc-controller && sudo systemctl status mpc-controller --no-pager -n 5"
```

## 📊 Performance Monitoring

### Check System Resources

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "top -bn1 | grep python3"
```

### Memory Usage

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "free -h"
```

### Disk Usage

```bash
ssh -i carla-mpc-key.pem -o StrictHostKeyChecking=no ec2-user@52.198.213.153 "df -h"
```

## 🔐 Security Notes

- The SSH key `carla-mpc-key.pem` should have permissions `400`
- AWS IoT certificates are stored securely on the EC2 instance
- The service runs with appropriate user permissions

## 📝 Environment Variables

The EC2 controller uses these environment variables (stored in `/home/ec2-user/mpc_controller/.env`):

```bash
AWS_ENDPOINT=a3ejb1zncumcih-ats.iot.ap-northeast-1.amazonaws.com
CLIENT_ID=carla-vehicle-1
AWS_CA_PATH=/home/ec2-user/certs/AmazonRootCA1.pem
AWS_CERT_PATH=/home/ec2-user/certs/device.pem.crt
AWS_KEY_PATH=/home/ec2-user/certs/private.pem.key
```

## 🎯 Testing the Deployment

After deployment, test with your CARLA simulation:

```bash
cd src
python3 main.py MPC
```

The EC2 controller should receive MPC requests and send back optimal control commands through AWS IoT Core.

---

## 📞 Quick Reference

| Action  | Command                                                                                                 |
| ------- | ------------------------------------------------------------------------------------------------------- |
| Deploy  | `scp -i carla-mpc-key.pem ec2_mpc_controller.py ec2-user@52.198.213.153:/home/ec2-user/mpc_controller/` |
| Restart | `ssh -i carla-mpc-key.pem ec2-user@52.198.213.153 "sudo systemctl restart mpc-controller"`              |
| Status  | `ssh -i carla-mpc-key.pem ec2-user@52.198.213.153 "sudo systemctl status mpc-controller"`               |
| Logs    | `ssh -i carla-mpc-key.pem ec2-user@52.198.213.153 "sudo journalctl -u mpc-controller -f"`               |

**Current EC2 Instance**: `52.198.213.153` (Amazon Linux 2)
**Service Name**: `mpc-controller`
**Region**: `ap-northeast-1`
