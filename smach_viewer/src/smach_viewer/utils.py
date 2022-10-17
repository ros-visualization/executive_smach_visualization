# Helper Functions
def graph_attr_string(attrs):
    """Generate an xdot graph attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k, v in attrs.items()]
    return ';\n'.join(attrs_strs)+';\n'


def attr_string(attrs):
    """Generate an xdot node attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k, v in attrs.items()]
    return ' ['+(', '.join(attrs_strs))+']'


def get_parent_path(path):
    """Get the parent path of an xdot node."""
    path_tokens = path.split('/')
    if len(path_tokens) > 2:
        parent_path = '/'.join(path_tokens[0:-1])
    else:
        parent_path = '/'.join(path_tokens[0:1])
    return parent_path


def get_label(path):
    """Get the label of an xdot node."""
    path_tokens = path.split('/')
    return path_tokens[-1]


def hex2t(color_str):
    """Convert a hexadecimal color strng into a color tuple."""
    color_tuple = [
        int(color_str[i:i+2], 16)/255.0 for i in range(1, len(color_str), 2)]
    return color_tuple
