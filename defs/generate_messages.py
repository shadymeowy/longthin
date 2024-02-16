import yaml

types_c = {
    'i8': 'int8_t',
    'i16': 'int16_t',
    'i32': 'int32_t',
    'i64': 'int64_t',
    'u8': 'uint8_t',
    'u16': 'uint16_t',
    'u32': 'uint32_t',
    'u64': 'uint64_t',
    'f32': 'float',
    'f64': 'double',
    'str': 'char*',
    'bool': 'bool'
}

types_py = {
    'i8': 'int',
    'i16': 'int',
    'i32': 'int',
    'i64': 'int',
    'u8': 'int',
    'u16': 'int',
    'u32': 'int',
    'u64': 'int',
    'f32': 'float',
    'f64': 'float',
    'str': 'str',
    'bool': 'bool'
}

types_struct = {
    'i8': 'c',
    'i16': 'h',
    'i32': 'i',
    'i64': 'q',
    'u8': 'B',
    'u16': 'H',
    'u32': 'I',
    'u64': 'Q',
    'f32': 'f',
    'f64': 'd',
    'str': 's',
    'bool': '?'
}


def generate_c(defs):
    def_lines = []
    type_lines = [
        '#include <stdint.h>',
        '#include <stdbool.h>\n'
    ]
    for typ, value in defs.items():
        typu = typ.upper()
        typl = typ.lower()
        def_line = f'LTMESSAGE({value["id"]}, LTPACKET_TYPE_{typu}, ltpacket_{typl}_t, {typl})'
        type_line = f'struct ltpacket_{typl}_t {{'
        for field, field_typ in value['fields'].items():
            type_line += f'\n    {types_c[field_typ]} {field};'
        type_line += '\n};\n'
        def_lines.append(def_line)
        type_lines.append(type_line)
    def_lines = '\n'.join(def_lines)
    type_lines = '\n'.join(type_lines)
    return def_lines, type_lines


def generate_py(defs):
    def0_lines = [
        'import struct',
        'from dataclasses import dataclass',
        'from enum import Enum\n\n',
    ]
    def1_lines = []
    def2_lines = []
    def3_lines = [
        'class LTPACKET_TYPE(Enum):'
    ]
    def4_lines = [
        'type_map = {'
    ]
    for typ, value in defs.items():
        typu = typ.upper()
        typl = typ.lower()
        struct_str = []
        for field, field_typ in value['fields'].items():
            struct_str.append(types_struct[field_typ])
        struct_str = ''.join(struct_str)
        camel = "".join(map(str.capitalize, typl.split('_')))
        def2_lines.append(f'{typl}_struct = struct.Struct(\'{struct_str}\')')
        def1_lines.append(f'@dataclass')
        def1_lines.append(f'class {camel}:')
        for field, field_typ in value['fields'].items():
            def1_lines.append(f'    {field}: {types_py[field_typ]}')
        def1_lines.append(f'\n    @staticmethod')
        def1_lines.append(f'    def from_bytes(data):')
        def1_lines.append(f'        return {camel}(*{typl}_struct.unpack(data))\n')
        def1_lines.append(f'    def to_bytes(self):')
        def1_lines.append(f'        return {typl}_struct.pack(')
        for field in value['fields']:
            def1_lines.append(f'            self.{field},')
        def1_lines.append(f'        )\n')
        def1_lines.append(f'    @property')
        def1_lines.append(f'    def type(self):')
        def1_lines.append(f'        return LTPACKET_TYPE.{typu}\n')
        def1_lines.append('\n')
        def3_lines.append(f'    {typu} = {int(value["id"])}')
        def4_lines.append(f'    LTPACKET_TYPE.{typu}: {camel},')
    def2_lines.append('')
    def3_lines.append('')
    def4_lines.append('}')
    def4_lines.append(f'type_map_rev = {{v: k for k, v in type_map.items()}}')
    def0_lines.extend(def1_lines)
    def0_lines.extend(def2_lines)
    def0_lines.extend(def3_lines)
    def0_lines.extend(def4_lines)
    def_lines = '\n'.join(def0_lines)
    return def_lines


if __name__ == "__main__":
    with open('messages.yaml') as file:
        messages = yaml.load(file, Loader=yaml.FullLoader)
    c_defs, c_types = generate_c(messages)
    py_defs = generate_py(messages)
    with open('../firmware/ltpacket/ltpacket_def.h', 'w') as file:
        file.write(c_defs)
    with open('../firmware/ltpacket/ltpacket_type.h', 'w') as file:
        file.write(c_types)
    with open('../longthin/ltpacket/ltpacket_def.py', 'w') as file:
        file.write(py_defs)
