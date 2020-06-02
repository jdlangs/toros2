import click
import collections
import re

ROSInterface = collections.namedtuple('ROSInterface', ['package', 'name', 'type'])

def identify_interfaces(data: str):
    """Get a list of all ROS interfaces that are being included in this file"""
    interfaces = list()

    pattern = '#include [<"](\w+)/(\w+)\.h[>"]'
    for match in re.finditer(pattern, data):
        filename = f'{match.group(1)}/{match.group(2)}.h'
        print(f'Included file: {filename}')

        #TODO: check if known interface

        filetype = click.prompt(
                'message / service / action / not an interface',
                default='n',
                type=click.Choice(['m','s', 'a', 'n']))
        if filetype == 'm':
            interfaces.append(
                ROSInterface(package = match.group(1), name = match.group(2), type = 'msg'))
        elif filetype == 's':
            interfaces.append(
                ROSInterface(package = match.group(1), name = match.group(2), type = 'srv'))
        elif filetype == 'a':
            interfaces.append(
                ROSInterface(package = match.group(1), name = match.group(2), type = 'action'))
        elif filetype == 'n':
            print('Not an interface')
    return interfaces
