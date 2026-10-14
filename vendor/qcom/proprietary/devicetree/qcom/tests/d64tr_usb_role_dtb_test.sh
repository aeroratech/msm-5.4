#!/bin/sh

set -eu

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <qrb5165-iot-openq.dtb>" >&2
	exit 2
fi

dtb=$1
usb_node=/soc/ssusb@a600000
dwc3_node=$usb_node/dwc3@a600000

role=$(fdtget -t s "$dtb" "$dwc3_node" dr_mode)
if [ "$role" != "otg" ]; then
	echo "FAIL: expected $dwc3_node dr_mode=otg, got '$role'" >&2
	exit 1
fi

if fdtget "$dtb" "$usb_node" qcom,disable-dev-mode-pm >/dev/null 2>&1; then
	echo "FAIL: $usb_node must allow leaving peripheral mode" >&2
	exit 1
fi

echo "PASS: D64TR primary USB controller supports host/device role switching"
