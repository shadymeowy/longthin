import os
import yaml
from collections import namedtuple
from importlib.resources import read_text

PATH_DEFAULT = 'default.yaml'


def load_config():
    if 'LTCONFIG' in os.environ:
        return load_config_file_merged(os.environ['LTCONFIG'])
    else:
        return load_default_config()


def load_config_file(path):
    if os.path.exists(path):
        with open(path) as f:
            dct = yaml.safe_load(f)
    else:
        txt = read_text('longthin.config', path)
        dct = yaml.safe_load(txt)

    return to_namedtuple('Config', dct)


def load_default_config():
    return load_config_file(PATH_DEFAULT)


def load_config_file_merged(path):
    default = load_default_config()
    user = load_config_file(path)
    return merge_namedtuples(default, user)


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


def merge_namedtuples(a, b):
    dct = {}
    for k in a._fields:
        v = getattr(a, k)
        dct[k] = v
    for k in b._fields:
        v = getattr(b, k)
        dct[k] = v
    return namedtuple('Config', dct.keys())(*dct.values())


if __name__ == '__main__':
    config = load_config()
    print(config)
