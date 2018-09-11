#!/bin/bash

# Get the absolute path to ESP32-BNO055's dir no matter where the script is called from
os=$(uname)  # Mac and Linux have different ways of following symlinks (`stat -f` vs `readlink -f`) -> Figure out which one to use
if [[ "$os" == "Darwin"* ]]; then
	PROTO_DIR="$(dirname "$(stat -f "$0")")"  # Get the folder where this script (build-protos.sh) is (also where sensors.proto is)
else  # Linux
	PROTO_DIR="$(dirname "$(readlink -f "$0")")"
fi
ROOT_DIR="$PROTO_DIR/.."  # Path to `ESP32-BNO055`

# Use nanopb to compile sensors.proto (for the ESP32)
PROTO_IN_FILES=("$PROTO_DIR/sensor_data.proto" "$PROTO_DIR/google/protobuf/timestamp.proto")
PROTO_OUT_DIR="$ROOT_DIR/arduino-submodules/libraries/IoTpairingProtos"
NANOPB_OUT_DIR="$PROTO_OUT_DIR/src"
mkdir -p "$NANOPB_OUT_DIR"
protoc -I "$PROTO_DIR" --nanopb_out="$NANOPB_OUT_DIR" --plugin=protoc-gen-nanopb=`which protoc-gen-nanopb` "${PROTO_IN_FILES[@]}"

# Make the repo an Arduino-compatible library (add a library.properties and a .h header that includes other headers)
echo "#include \"sensor_data.pb.h\"" > "$NANOPB_OUT_DIR/IoTpairingProtos.h"
printf "name=IoT Pairing Protos\nversion=1.0.0\nauthor=Carlos Ruiz\nmaintainer=Carlos Ruiz <carlosrd@cmu.edu>\nsentence=Nanopb (C++) implementation of our paper's IoT pairing protobufs\nparagraph=\nurl=\ncategory=Communication" > "$PROTO_OUT_DIR/library.properties"
echo "Done! Protos saved to '$PROTO_OUT_DIR'"
