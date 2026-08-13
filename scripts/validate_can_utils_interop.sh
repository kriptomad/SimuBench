#!/usr/bin/env bash
set -euo pipefail

# Linux bench validation script for SocketCAN + ISO-TP interoperability.
# Requires can-utils installed and a vcan interface available.

CAN_IF="${CAN_IF:-vcan0}"
SRC_ID="${SRC_ID:-7E0}"
DST_ID="${DST_ID:-7E8}"
TIMEOUT_SEC="${TIMEOUT_SEC:-5}"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing command: $1" >&2
    exit 1
  }
}

need_cmd ip
need_cmd candump
need_cmd isotpsend
need_cmd isotprecv

echo "[1/5] Checking CAN interface ${CAN_IF}"
if ! ip link show "${CAN_IF}" >/dev/null 2>&1; then
  echo "interface ${CAN_IF} not found" >&2
  exit 1
fi

echo "[2/5] Starting capture window"
TMP_LOG="$(mktemp /tmp/autobreaking_canutils_XXXXXX.log)"
(candump "${CAN_IF}" >"${TMP_LOG}" 2>&1 &) 
CANDUMP_PID=$!
trap 'kill ${CANDUMP_PID} >/dev/null 2>&1 || true; rm -f "${TMP_LOG}"' EXIT
sleep 0.2

echo "[3/5] Launching ISO-TP receiver"
TMP_RX="$(mktemp /tmp/autobreaking_isotp_rx_XXXXXX.log)"
(timeout "${TIMEOUT_SEC}" isotprecv -s "${DST_ID}" -d "${SRC_ID}" "${CAN_IF}" >"${TMP_RX}" 2>&1 &) 
RX_PID=$!
sleep 0.2

echo "[4/5] Sending UDS TesterPresent over ISO-TP"
printf '3E 00\n' | isotpsend -s "${SRC_ID}" -d "${DST_ID}" "${CAN_IF}"

wait "${RX_PID}" || true

kill "${CANDUMP_PID}" >/dev/null 2>&1 || true

if grep -Eq '3E 00|3e 00' "${TMP_RX}"; then
  echo "[5/5] PASS: ISO-TP payload observed on receiver"
else
  echo "[5/5] FAIL: payload not observed by isotprecv" >&2
  echo "--- isotprecv output ---" >&2
  cat "${TMP_RX}" >&2 || true
  echo "--- candump output ---" >&2
  cat "${TMP_LOG}" >&2 || true
  exit 1
fi

echo "Validation logs:"
echo "  isotprecv: ${TMP_RX}"
echo "  candump:   ${TMP_LOG}"
