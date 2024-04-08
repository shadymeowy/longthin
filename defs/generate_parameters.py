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
        "from enum import Enum",
        "from dataclasses import dataclass, asdict\n\n",
        f"LTPARAMS_COUNT = 0x{len(defs):x}\n\n",
        "class LTParamValueType(Enum):",
        "    LTPARAMS_TYPE_FLOAT = 0",
        "    LTPARAMS_TYPE_UINT32 = 1",
        "    LTPARAMS_TYPE_INT32 = 2\n\n",
        "class LTParamType(Enum):"
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
        def_line = f"    LTParamType.{namel.upper()}: LTParamValueType.{typ},"
        def_lines.append(def_line)
    def_lines.append("}\n\nparam_default_dict = {")
    for name, value in defs.items():
        namel = name.lower()
        id_ = value['id']
        default = value['default']
        def_line = f"    LTParamType.{namel.upper()}: {default},"
        def_lines.append(def_line)
    def_lines.append("}\n\n")

    def_lines.append("@dataclass")
    def_lines.append("class LTParameters:")
    for name, value in defs.items():
        namel = name.lower()
        typ = types_py[value['type']]
        def_line = f"    {namel}: {typ}"
        def_lines.append(def_line)
    def_lines.append("")
    def_lines.append(r"""    @classmethod
    def from_dict(cls, d):
        new_dict = {}
        for key, item in d.items():
            if key not in param_default_dict:
                raise ValueError(f"Unknown parameter {key}")
            key = key.name.lower()
            new_dict[key] = item
        return cls(**new_dict)

    @classmethod
    def from_default(cls):
        return cls.from_dict(param_default_dict)

    def to_dict(self):
        dct = asdict(self)
        new_dict = {}
        for key, item in dct.items():
            key = key.upper()
            new_dict[LTParamType[key]] = item
        return new_dict

    def __getitem__(self, key):
        key = key.name.lower()
        return getattr(self, key)

    def __setitem__(self, key, value):
        key = key.name.lower()
        setattr(self, key, value)""")
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
