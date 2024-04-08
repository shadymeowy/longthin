import struct
from .ltparams_def import *
from .helpers import *


param_packet_dict = {
    LTParamValueType.LTPARAMS_TYPE_FLOAT: Setparam,
    LTParamValueType.LTPARAMS_TYPE_UINT32: Setparamu,
    LTParamValueType.LTPARAMS_TYPE_INT32: Setparami,
}


def setparam(param, value):
    if not isinstance(param, LTParamType):
        raise ValueError('param must be an LTParamType')
    if param not in param_type_dict:
        raise ValueError('param not found in param_type_dict')
    param_type = param_type_dict[param]
    if param_type not in param_packet_dict:
        raise ValueError('param_type not found in param_packet_dict')
    return param_packet_dict[param_type](param.value, value)


def default_params():
    return param_default_dict.copy()


def default_param(param):
    if not isinstance(param, LTParamType):
        raise ValueError('param must be an LTParamType')
    if param not in param_default_dict:
        raise ValueError('param not found in param_default_dict')
    return param_default_dict[param]


def get_param_type(param):
    if not isinstance(param, LTParamType):
        raise ValueError('param must be an LTParamType')
    if param not in param_type_dict:
        raise ValueError('param not found in param_type_dict')
    return param_type_dict[param]


if __name__ == '__main__':
    import pprint
    params = default_params()
    pprint.pprint(params)
    print('parameter', LTParamType.QCOMP_ALPHA)
    print('default', default_param(LTParamType.QCOMP_ALPHA))
    packet = setparam(LTParamType.QCOMP_ALPHA, 0.15)
    byts = encode(packet)
    print('encoded', byts)
    print('decoded', decode(byts))
    print('type', get_param_type(LTParamType.QCOMP_ALPHA))

    print('parameter', LTParamType.IMU_FILTER_TYPE)
    print('default', default_param(LTParamType.IMU_FILTER_TYPE))
    packet = setparam(LTParamType.IMU_FILTER_TYPE, 3)
    byts = encode(packet)
    print('encoded', byts)
    print('decoded', decode(byts))
    print('type', get_param_type(LTParamType.IMU_FILTER_TYPE))
