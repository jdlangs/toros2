import argparse
import click
import collections
import inflection
import re
import os
import yaml

Sub = collections.namedtuple('Switch', ['pattern', 'replace', 'save'])

LOGGER = ''
CACHEFILE = '.toros2.cache'

logging_subs = [
    #printf vanilla: ROS_DEBUG({args...}) -> RCLCPP_DEBUG({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)\((.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({logger_expr()}, {m.group(2)});',
        save = False,
    ),
    #printf cond: ROS_INFO_COND({condition}, {args...}) -> RCLCPP_INFO_EXPRESSION({logger}, {condition}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_COND\((.*?),(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}_EXPRESSION({logger_expr()}, {m.group(2)},{m.group(3)});',
        save = False,
    ),
    #printf once: ROS_INFO_ONCE({args...}) -> RCLCPP_INFO_ONCE({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_ONCE\((.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}_ONCE({logger_expr()}, {m.group(2)});',
        save = False,
    ),
    #printf throttle: ROS_INFO_THROTTLE(period, {args...}) -> RCLCPP_INFO({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_THROTTLE\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({logger_expr()}, {m.group(2)});',
        save = False,
    ),
    #printf delayed throttle: ROS_INFO_DELAYED_THROTTLE(period, {args...}) -> RCLCPP_INFO({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_DELAYED_THROTTLE\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({logger_expr()}, {m.group(2)});',
        save = False,
    ),
    #printf filter: ROS_INFO_FILTER(filter, {args...}) -> RCLCPP_INFO({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_FILTER\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({logger_expr()}, {m.group(2)});',
        save = False,
    ),
    #stream vanilla
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM\((.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({logger_expr()}, {stream_to_printf(m.group(2))});',
        save = False,
    ),
    #stream cond
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_COND\((.*?),(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}_EXPRESSION({logger_expr()}, {m.group(2)},{stream_to_printf(m.group(3))});',
        save = False,
    ),
    #stream once
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_ONCE\((.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}_ONCE({logger_expr()}, {stream_to_printf(m.group(2))});',
        save = False,
    ),
    #stream throttle
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_THROTTLE\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({logger_expr()}, {stream_to_printf(m.group(2))});',
        save = False,
    ),
    #stream delayed throttle
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_DELAYED_THROTTLE\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({logger_expr()}, {stream_to_printf(m.group(2))});',
        save = False,
    ),
    #stream filter
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_FILTER\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({logger_expr()}, {stream_to_printf(m.group(2))});',
        save = False,
    ),
]

rclcpp_subs = [
    Sub(
        pattern = 'ros::Duration',
        replace = lambda m: 'rclcpp::Duration',
        save = False,
    ),
    Sub(
        pattern = 'ros::Time',
        replace = lambda m: 'rclcpp::Time',
        save = False,
    ),
]

def logger_expr():
    global LOGGER
    if len(LOGGER) == 0:
        LOGGER = click.prompt('Enter expression to use for logger', type=str, default=LOGGER)
    return LOGGER

def stream_to_printf(contents: str):
    elems = map(lambda s: s.strip(), contents.split('<<')) # whitespace cleaned list of contents separated by '<<'
    exprs = list()
    printf_list = list()
    for elem in elems:
        # non-string items turn into a format specifier, string items are kept as-is
        if not (elem.startswith('"') and elem.endswith('"')):
            exprs.append(elem)
            printf_list.append(guess_format_type(elem))
        else:
            printf_list.append(elem[1:-1])

    fmt_str = ''.join(printf_list)
    fmt_args = ''.join(map(lambda s: ', '+s, exprs))
    printf_contents = '"{}"{}'.format(fmt_str, fmt_args)
    return printf_contents

def guess_format_type(expr: str):
    try:
        int(expr)
        return '%i'
    except ValueError:
        pass
    try:
        float(expr)
        return '%f'
    except ValueError:
        pass
    fmt = click.prompt(f"    Enter format specified for expression '{expr}'", type=str, default='%s')
    return fmt

def apply_sub(data: str, sub: Sub, cache: dict, *, confirm=False):
    print('Checking for pattern: ', sub.pattern)
    def print_and_replace(m):
        matched = m.group(0)
        msgwidth = 20
        print(f"  {'pattern found':<{msgwidth}}: '{matched}'")
        repl = ''
        if matched in cache:
            repl = cache[matched]
            print(f"  {'cached replacement':<{msgwidth}}: '{repl}'")
        else:
            repl = sub.replace(m)
            print(f"  {'replacing with':<{msgwidth}}: '{repl}'")
        action = 'y'
        if confirm:
            action = click.prompt('yes/no/rewrite', default='y', type=click.Choice(['y','n','r']))
        if action == 'y':
            if sub.save:
                cache[matched] = repl
            return repl
        elif action == 'n':
            return None
        elif action == 'r':
            newrepl = click.prompt('  Input alternate replacement:', type=str)
            if sub.save:
                cache[matched] = newrepl

    data = re.sub(sub.pattern, print_and_replace, data)
    return data

def substitutions():
    """
    Make a list of all substitutions to apply
    """
    # '#include <geometry_msgs/Point.h>' -> '#include <geometry_msgs/msg/point.hpp'
    include_sub = Sub(
        pattern = r'#include <([a-z_]+_msgs)/([a-zA-Z_]+).h>',
        replace = lambda m: f'#include <{m.group(1)}/msg/{inflection.underscore(m.group(2))}.hpp>',
        save = True,
    )
    # 'geometry_msgs::Point' -> 'geometry_msgs::msg::Point'
    msg_sub = Sub(
        pattern = r'([a-z_]+_msgs)::([a-zA-z]+)',
        replace = lambda m: f'{m.group(1)}::msg::{m.group(2)}',
        save = True
    )
    all_subs = list()
    all_subs.append(include_sub)
    all_subs.append(msg_sub)
    all_subs.extend(logging_subs)
    all_subs.extend(rclcpp_subs)
    return all_subs

def load_cache(cachefile: str):
    cache = dict()
    if not os.path.exists(cachefile):
        print('No cache found')
        return cache
    with open(cachefile, 'r') as fs:
        cache = yaml.load(fs)
    print(f'Cache found, loaded {len(cache)} items')
    return cache

def save_cache(cachefile: str, cache: dict):
    with open(cachefile, 'w') as fs:
        fs.write(yaml.dump(cache))

def readfile(filestr: str):
    with open(filestr, 'r') as fs:
        return fs.read()

def writefile(data: str, name: str, args: dict):
    with open(name, 'w') as fs:
        fs.write(data)

@click.command()
@click.argument('filename', type=click.Path(exists=True, dir_okay=False, writable=True), nargs=-1)
@click.option('-c', '--confirm', type=bool, is_flag=True, help='Confirm each replacement, even when the pattern is found in the replacement history cache')
@click.option('-n', '--nobackup', type=bool, is_flag=True, help='Do not write a backup of the original file')
def main(**args):
    """
    Convert ROS1 C++ files to ROS2 equivalents (mostly) by replacing certain patterns
    """
    cache = load_cache(CACHEFILE)
    subs = substitutions()

    files = args['filename']
    print('Converting files: ', files)
    for f in files:
        # read file
        orig_contents = readfile(f)

        # send through subs
        contents = orig_contents
        for sub in subs:
            contents = apply_sub(contents, sub, cache, confirm=args['confirm'])

        # save backup
        if not args['nobackup']:
            writefile(orig_contents, f + '.ros1', args)
        # overwrite original file
        writefile(contents, f, args)

    save_cache(CACHEFILE, cache)
