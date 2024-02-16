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
        print(name, value)
        namel = name.lower()
        id_ = value['id']
        typ = types_enum[value['type']]
        default = value['default']
        field = types_field[value['type']]
        def_line = f'LTDEF(0x{(id_):x}, LTPARAMS_{namel.upper()}, {typ}, {field}, {default})'
        def_lines.append(def_line)

    def_lines = '\n'.join(def_lines)
    return def_lines


if __name__ == "__main__":
    with open('parameters.yaml') as file:
        parameters = yaml.load(file, Loader=yaml.FullLoader)
    c_defs = generate_c(parameters)
    with open('../firmware/ltpacket/ltparams_def.h', 'w') as file:
        file.write(c_defs)
