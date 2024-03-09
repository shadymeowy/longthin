import yaml
from collections import namedtuple


def load_config(path):
    with open(path) as f:
        dct = yaml.safe_load(f)
    return to_namedtuple('Config', dct)


def to_namedtuple(name, dct):
    keys = []
    values = []
    for k, v in dct.items():
        keys.append(k)
        if isinstance(v, dict):
            k = k.capitalize()
            values.append(to_namedtuple(k, v))
        elif isinstance(v, list):
            lst = []
            for e in v:
                if isinstance(e, dict):
                    lst.append(to_namedtuple(k, e))
                else:
                    lst.append(e)
            values.append(lst)
        else:
            values.append(v)
    return namedtuple(name, keys)(*values)


if __name__ == '__main__':
    config = load_config('default.yaml')
    print(config)
