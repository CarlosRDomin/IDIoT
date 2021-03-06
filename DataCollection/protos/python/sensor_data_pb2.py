# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensor_data.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import nanopb_pb2 as nanopb__pb2
from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='sensor_data.proto',
  package='IoT.pairing',
  syntax='proto3',
  serialized_pb=_b('\n\x11sensor_data.proto\x12\x0bIoT.pairing\x1a\x0cnanopb.proto\x1a\x1fgoogle/protobuf/timestamp.proto\"\xac\x02\n\x10GenericDataArray\x12\x15\n\x06\x64\x61ta_x\x18\x01 \x03(\x02\x42\x05\x92?\x02\x10\x32\x12\x15\n\x06\x64\x61ta_y\x18\x02 \x03(\x02\x42\x05\x92?\x02\x10\x32\x12\x15\n\x06\x64\x61ta_z\x18\x03 \x03(\x02\x42\x05\x92?\x02\x10\x32\x12:\n\tmagnitude\x18\x04 \x01(\x0e\x32\'.IoT.pairing.GenericDataArray.ArrayType\x12,\n\x08t_latest\x18\x05 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12\x0e\n\x06\x46_samp\x18\x06 \x01(\x02\"Y\n\tArrayType\x12\x0f\n\x0bORIENTATION\x10\x00\x12\x10\n\x0cLINEAR_ACCEL\x10\x01\x12\x0b\n\x07GRAVITY\x10\x02\x12\t\n\x05\x41\x43\x43\x45L\x10\x03\x12\x08\n\x04GYRO\x10\x04\x12\x07\n\x03MAG\x10\x05\"A\n\x10GenericDataBlock\x12-\n\x06values\x18\x01 \x03(\x0b\x32\x1d.IoT.pairing.GenericDataArray\"@\n\x08XYZArray\x12\x10\n\x01x\x18\x01 \x03(\x02\x42\x05\x92?\x02\x10\x32\x12\x10\n\x01y\x18\x02 \x03(\x02\x42\x05\x92?\x02\x10\x32\x12\x10\n\x01z\x18\x03 \x03(\x02\x42\x05\x92?\x02\x10\x32\"S\n\tQuatArray\x12\x10\n\x01w\x18\x04 \x03(\x02\x42\x05\x92?\x02\x10\x32\x12\x10\n\x01x\x18\x01 \x03(\x02\x42\x05\x92?\x02\x10\x32\x12\x10\n\x01y\x18\x02 \x03(\x02\x42\x05\x92?\x02\x10\x32\x12\x10\n\x01z\x18\x03 \x03(\x02\x42\x05\x92?\x02\x10\x32\"N\n\x11\x43\x61librationStatus\x12\x0f\n\x07overall\x18\x01 \x01(\r\x12\r\n\x05\x61\x63\x63\x65l\x18\x02 \x01(\r\x12\x0c\n\x04gyro\x18\x03 \x01(\r\x12\x0b\n\x03mag\x18\x04 \x01(\r\"\xfa\x02\n\tDataBlock\x12+\n\x0borientation\x18\x01 \x01(\x0b\x32\x16.IoT.pairing.QuatArray\x12*\n\x0blinearAccel\x18\x02 \x01(\x0b\x32\x15.IoT.pairing.XYZArray\x12&\n\x07gravity\x18\x03 \x01(\x0b\x32\x15.IoT.pairing.XYZArray\x12$\n\x05\x61\x63\x63\x65l\x18\x04 \x01(\x0b\x32\x15.IoT.pairing.XYZArray\x12#\n\x04gyro\x18\x05 \x01(\x0b\x32\x15.IoT.pairing.XYZArray\x12\"\n\x03mag\x18\x06 \x01(\x0b\x32\x15.IoT.pairing.XYZArray\x12,\n\x08t_latest\x18\x07 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12\x0e\n\x06\x46_samp\x18\x08 \x01(\x02\x12\n\n\x02id\x18\t \x01(\r\x12\x33\n\x0b\x63\x61libStatus\x18\n \x01(\x0b\x32\x1e.IoT.pairing.CalibrationStatusb\x06proto3')
  ,
  dependencies=[nanopb__pb2.DESCRIPTOR,google_dot_protobuf_dot_timestamp__pb2.DESCRIPTOR,])



_GENERICDATAARRAY_ARRAYTYPE = _descriptor.EnumDescriptor(
  name='ArrayType',
  full_name='IoT.pairing.GenericDataArray.ArrayType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='ORIENTATION', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LINEAR_ACCEL', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GRAVITY', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ACCEL', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='GYRO', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MAG', index=5, number=5,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=293,
  serialized_end=382,
)
_sym_db.RegisterEnumDescriptor(_GENERICDATAARRAY_ARRAYTYPE)


_GENERICDATAARRAY = _descriptor.Descriptor(
  name='GenericDataArray',
  full_name='IoT.pairing.GenericDataArray',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data_x', full_name='IoT.pairing.GenericDataArray.data_x', index=0,
      number=1, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='data_y', full_name='IoT.pairing.GenericDataArray.data_y', index=1,
      number=2, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='data_z', full_name='IoT.pairing.GenericDataArray.data_z', index=2,
      number=3, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='magnitude', full_name='IoT.pairing.GenericDataArray.magnitude', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='t_latest', full_name='IoT.pairing.GenericDataArray.t_latest', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='F_samp', full_name='IoT.pairing.GenericDataArray.F_samp', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _GENERICDATAARRAY_ARRAYTYPE,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=82,
  serialized_end=382,
)


_GENERICDATABLOCK = _descriptor.Descriptor(
  name='GenericDataBlock',
  full_name='IoT.pairing.GenericDataBlock',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='values', full_name='IoT.pairing.GenericDataBlock.values', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=384,
  serialized_end=449,
)


_XYZARRAY = _descriptor.Descriptor(
  name='XYZArray',
  full_name='IoT.pairing.XYZArray',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='IoT.pairing.XYZArray.x', index=0,
      number=1, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='IoT.pairing.XYZArray.y', index=1,
      number=2, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='z', full_name='IoT.pairing.XYZArray.z', index=2,
      number=3, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=451,
  serialized_end=515,
)


_QUATARRAY = _descriptor.Descriptor(
  name='QuatArray',
  full_name='IoT.pairing.QuatArray',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='w', full_name='IoT.pairing.QuatArray.w', index=0,
      number=4, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='x', full_name='IoT.pairing.QuatArray.x', index=1,
      number=1, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y', full_name='IoT.pairing.QuatArray.y', index=2,
      number=2, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='z', full_name='IoT.pairing.QuatArray.z', index=3,
      number=3, type=2, cpp_type=6, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202')), file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=517,
  serialized_end=600,
)


_CALIBRATIONSTATUS = _descriptor.Descriptor(
  name='CalibrationStatus',
  full_name='IoT.pairing.CalibrationStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='overall', full_name='IoT.pairing.CalibrationStatus.overall', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accel', full_name='IoT.pairing.CalibrationStatus.accel', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyro', full_name='IoT.pairing.CalibrationStatus.gyro', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mag', full_name='IoT.pairing.CalibrationStatus.mag', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=602,
  serialized_end=680,
)


_DATABLOCK = _descriptor.Descriptor(
  name='DataBlock',
  full_name='IoT.pairing.DataBlock',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='orientation', full_name='IoT.pairing.DataBlock.orientation', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='linearAccel', full_name='IoT.pairing.DataBlock.linearAccel', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gravity', full_name='IoT.pairing.DataBlock.gravity', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='accel', full_name='IoT.pairing.DataBlock.accel', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gyro', full_name='IoT.pairing.DataBlock.gyro', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='mag', full_name='IoT.pairing.DataBlock.mag', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='t_latest', full_name='IoT.pairing.DataBlock.t_latest', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='F_samp', full_name='IoT.pairing.DataBlock.F_samp', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='id', full_name='IoT.pairing.DataBlock.id', index=8,
      number=9, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='calibStatus', full_name='IoT.pairing.DataBlock.calibStatus', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=683,
  serialized_end=1061,
)

_GENERICDATAARRAY.fields_by_name['magnitude'].enum_type = _GENERICDATAARRAY_ARRAYTYPE
_GENERICDATAARRAY.fields_by_name['t_latest'].message_type = google_dot_protobuf_dot_timestamp__pb2._TIMESTAMP
_GENERICDATAARRAY_ARRAYTYPE.containing_type = _GENERICDATAARRAY
_GENERICDATABLOCK.fields_by_name['values'].message_type = _GENERICDATAARRAY
_DATABLOCK.fields_by_name['orientation'].message_type = _QUATARRAY
_DATABLOCK.fields_by_name['linearAccel'].message_type = _XYZARRAY
_DATABLOCK.fields_by_name['gravity'].message_type = _XYZARRAY
_DATABLOCK.fields_by_name['accel'].message_type = _XYZARRAY
_DATABLOCK.fields_by_name['gyro'].message_type = _XYZARRAY
_DATABLOCK.fields_by_name['mag'].message_type = _XYZARRAY
_DATABLOCK.fields_by_name['t_latest'].message_type = google_dot_protobuf_dot_timestamp__pb2._TIMESTAMP
_DATABLOCK.fields_by_name['calibStatus'].message_type = _CALIBRATIONSTATUS
DESCRIPTOR.message_types_by_name['GenericDataArray'] = _GENERICDATAARRAY
DESCRIPTOR.message_types_by_name['GenericDataBlock'] = _GENERICDATABLOCK
DESCRIPTOR.message_types_by_name['XYZArray'] = _XYZARRAY
DESCRIPTOR.message_types_by_name['QuatArray'] = _QUATARRAY
DESCRIPTOR.message_types_by_name['CalibrationStatus'] = _CALIBRATIONSTATUS
DESCRIPTOR.message_types_by_name['DataBlock'] = _DATABLOCK
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GenericDataArray = _reflection.GeneratedProtocolMessageType('GenericDataArray', (_message.Message,), dict(
  DESCRIPTOR = _GENERICDATAARRAY,
  __module__ = 'sensor_data_pb2'
  # @@protoc_insertion_point(class_scope:IoT.pairing.GenericDataArray)
  ))
_sym_db.RegisterMessage(GenericDataArray)

GenericDataBlock = _reflection.GeneratedProtocolMessageType('GenericDataBlock', (_message.Message,), dict(
  DESCRIPTOR = _GENERICDATABLOCK,
  __module__ = 'sensor_data_pb2'
  # @@protoc_insertion_point(class_scope:IoT.pairing.GenericDataBlock)
  ))
_sym_db.RegisterMessage(GenericDataBlock)

XYZArray = _reflection.GeneratedProtocolMessageType('XYZArray', (_message.Message,), dict(
  DESCRIPTOR = _XYZARRAY,
  __module__ = 'sensor_data_pb2'
  # @@protoc_insertion_point(class_scope:IoT.pairing.XYZArray)
  ))
_sym_db.RegisterMessage(XYZArray)

QuatArray = _reflection.GeneratedProtocolMessageType('QuatArray', (_message.Message,), dict(
  DESCRIPTOR = _QUATARRAY,
  __module__ = 'sensor_data_pb2'
  # @@protoc_insertion_point(class_scope:IoT.pairing.QuatArray)
  ))
_sym_db.RegisterMessage(QuatArray)

CalibrationStatus = _reflection.GeneratedProtocolMessageType('CalibrationStatus', (_message.Message,), dict(
  DESCRIPTOR = _CALIBRATIONSTATUS,
  __module__ = 'sensor_data_pb2'
  # @@protoc_insertion_point(class_scope:IoT.pairing.CalibrationStatus)
  ))
_sym_db.RegisterMessage(CalibrationStatus)

DataBlock = _reflection.GeneratedProtocolMessageType('DataBlock', (_message.Message,), dict(
  DESCRIPTOR = _DATABLOCK,
  __module__ = 'sensor_data_pb2'
  # @@protoc_insertion_point(class_scope:IoT.pairing.DataBlock)
  ))
_sym_db.RegisterMessage(DataBlock)


_GENERICDATAARRAY.fields_by_name['data_x'].has_options = True
_GENERICDATAARRAY.fields_by_name['data_x']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_GENERICDATAARRAY.fields_by_name['data_y'].has_options = True
_GENERICDATAARRAY.fields_by_name['data_y']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_GENERICDATAARRAY.fields_by_name['data_z'].has_options = True
_GENERICDATAARRAY.fields_by_name['data_z']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_XYZARRAY.fields_by_name['x'].has_options = True
_XYZARRAY.fields_by_name['x']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_XYZARRAY.fields_by_name['y'].has_options = True
_XYZARRAY.fields_by_name['y']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_XYZARRAY.fields_by_name['z'].has_options = True
_XYZARRAY.fields_by_name['z']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_QUATARRAY.fields_by_name['w'].has_options = True
_QUATARRAY.fields_by_name['w']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_QUATARRAY.fields_by_name['x'].has_options = True
_QUATARRAY.fields_by_name['x']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_QUATARRAY.fields_by_name['y'].has_options = True
_QUATARRAY.fields_by_name['y']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
_QUATARRAY.fields_by_name['z'].has_options = True
_QUATARRAY.fields_by_name['z']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\222?\002\0202'))
# @@protoc_insertion_point(module_scope)
