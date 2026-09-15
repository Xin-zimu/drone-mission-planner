#!/usr/bin/env bash
set -euo pipefail

python -m dmp_crazyflie_bridge.protocol_server --backend sim

