import struct
from .ltparams_def import *

float_struct = struct.Struct('f')
uint32_struct = struct.Struct('I')
int32_struct = struct.Struct('i')


param_struct_dict = {
    LTParamType.LTPARAMS_TYPE_FLOAT: float_struct,
    LTParamType.LTPARAMS_TYPE_UINT32: uint32_struct,
    LTParamType.LTPARAMS_TYPE_INT32: int32_struct,
}


def encode_value(param, value):
    if not isinstance(param, LTParams):
        raise ValueError('param must be an LTParams')
    if param not in param_type_dict:
        raise ValueError('param not found in param_type_dict')
    param_type = param_type_dict[param]
    if param_type not in param_struct_dict:
        raise ValueError('param_type not found in param_struct_dict')
    return param_struct_dict[param_type].pack(value)


def decode_value(param, data):
    if not isinstance(param, LTParams):
        raise ValueError('param must be an LTParams')
    if param not in param_type_dict:
        raise ValueError('param not found in param_type_dict')
    param_type = param_type_dict[param]
    if param_type not in param_struct_dict:
        raise ValueError('param_type not found in param_struct_dict')
    return param_struct_dict[param_type].unpack(data)[0]


def default_params():
    return param_default_dict.copy()


def default_param(param):
    if not isinstance(param, LTParams):
        raise ValueError('param must be an LTParams')
    if param not in param_default_dict:
        raise ValueError('param not found in param_default_dict')
    return param_default_dict[param]


def get_param_type(param):
    if not isinstance(param, LTParams):
        raise ValueError('param must be an LTParams')
    if param not in param_type_dict:
        raise ValueError('param not found in param_type_dict')
    return param_type_dict[param]


if __name__ == '__main__':
    import pprint
    params = default_params()
    pprint.pprint(params)
    print('parameter', LTParams.QCOMP_ALPHA)
    print('default', default_param(LTParams.QCOMP_ALPHA))
    byts = encode_value(LTParams.QCOMP_ALPHA, 0.15)
    print('encoded', byts)
    print('decoded', decode_value(LTParams.QCOMP_ALPHA, byts))
    print('type', get_param_type(LTParams.QCOMP_ALPHA))

    print('parameter', LTParams.IMU_FILTER_TYPE)
    print('default', default_param(LTParams.IMU_FILTER_TYPE))
    byts = encode_value(LTParams.IMU_FILTER_TYPE, 3)
    print('encoded', byts)
    print('decoded', decode_value(LTParams.IMU_FILTER_TYPE, byts))
    print('type', get_param_type(LTParams.IMU_FILTER_TYPE))
