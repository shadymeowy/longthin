import yaml

types_c = {
    'f32': 'float',
    'u32': 'uint32_t',
    'i32': 'int32_t'
}

types_py = {
    'f32': 'float',
    'u32': 'int',
    'i32': 'int'
}

types_struct = {
    'f32': 'f',
    'u32': 'I',
    'i32': 'i'
}

types_enum = {
    'f32': 'LTPARAMS_TYPE_FLOAT',
    'u32': 'LTPARAMS_TYPE_UINT32',
    'i32': 'LTPARAMS_TYPE_INT32'
}

types_field = {
    'f32': 'f',
    'u32': 'ui',
    'i32': 'i'
}


def generate_c(defs):
    def_lines = [
        f"#define LTPARAMS_COUNT 0x{len(defs):x}"
    ]
    for name, value in defs.items():
        namel = name.lower()
        id_ = value['id']
        typ = types_enum[value['type']]
        default = value['default']
        field = types_field[value['type']]
        def_line = f'LTDEF(0x{(id_):x}, LTPARAMS_{namel.upper()}, {typ}, {field}, {default})'
        def_lines.append(def_line)

    def_lines = '\n'.join(def_lines)
    return def_lines


def generate_py(defs):
    def_lines = [
        "from enum import Enum\n\n",
        f"LTPARAMS_COUNT = 0x{len(defs):x}\n\n",
        "class LTParamType(Enum):",
        "    LTPARAMS_TYPE_FLOAT = 0",
        "    LTPARAMS_TYPE_UINT32 = 1",
        "    LTPARAMS_TYPE_INT32 = 2\n\n",
        "class LTParams(Enum):"
    ]
    for name, value in defs.items():
        namel = name.lower()
        id_ = value['id']
        def_line = f"    {namel.upper()} = 0x{id_:x}"
        def_lines.append(def_line)
    def_lines.append("\n\nparam_type_dict = {")
    for name, value in defs.items():
        namel = name.lower()
        id_ = value['id']
        typ = types_enum[value['type']]
        def_line = f"    LTParams.{namel.upper()}: LTParamType.{typ},"
        def_lines.append(def_line)
    def_lines.append("}\n\nparam_default_dict = {")
    for name, value in defs.items():
        namel = name.lower()
        id_ = value['id']
        default = value['default']
        def_line = f"    LTParams.{namel.upper()}: {default},"
        def_lines.append(def_line)
    def_lines.append("}")
    def_lines = '\n'.join(def_lines)
    return def_lines


if __name__ == "__main__":
    with open('parameters.yaml') as file:
        parameters = yaml.load(file, Loader=yaml.FullLoader)
    c_defs = generate_c(parameters)
    py_defs = generate_py(parameters)
    with open('../firmware/ltpacket/ltparams_def.h', 'w') as file:
        file.write(c_defs)
    with open('../longthin/ltpacket/ltparams_def.py', 'w') as file:
        file.write(py_defs)
