import argparse
import click
import collections
import inflection
import re
import os

Sub = collections.namedtuple('Switch', ['pattern', 'replace'])

LOGGER = 'node_->get_logger()'

logging_subs = [
    #printf vanilla: ROS_DEBUG({args...}) -> RCLCPP_DEBUG({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)\((.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({LOGGER}, {m.group(2)});',
    ),
    #printf cond: ROS_INFO_COND({condition}, {args...}) -> RCLCPP_INFO_EXPRESSION({logger}, {condition}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_COND\((.*?),(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}_EXPRESSION({LOGGER}, {m.group(2)},{m.group(3)});',
    ),
    #printf once: ROS_INFO_ONCE({args...}) -> RCLCPP_INFO_ONCE({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_ONCE\((.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}_ONCE({LOGGER}, {m.group(2)});',
    ),
    #printf throttle: ROS_INFO_THROTTLE(period, {args...}) -> RCLCPP_INFO({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_THROTTLE\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({LOGGER}, {m.group(2)});',
    ),
    #printf delayed throttle: ROS_INFO_DELAYED_THROTTLE(period, {args...}) -> RCLCPP_INFO({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_DELAYED_THROTTLE\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({LOGGER}, {m.group(2)});',
    ),
    #printf filter: ROS_INFO_FILTER(filter, {args...}) -> RCLCPP_INFO({logger}, {args...})
    Sub(
        pattern = r'ROS_([A-Z]+)_FILTER\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({LOGGER}, {m.group(2)});',
    ),
    #stream vanilla
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM\((.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({LOGGER}, {stream_to_printf(m.group(2))});'
    ),
    #stream cond
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_COND\((.*?),(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}_EXPRESSION({LOGGER}, {m.group(2)},{stream_to_printf(m.group(3))});'
    ),
    #stream once
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_ONCE\((.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}_ONCE({LOGGER}, {stream_to_printf(m.group(2))});'
    ),
    #stream throttle
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_THROTTLE\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({LOGGER}, {stream_to_printf(m.group(2))});',
    ),
    #stream delayed throttle
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_DELAYED_THROTTLE\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({LOGGER}, {stream_to_printf(m.group(2))});',
    ),
    #stream filter
    Sub(
        pattern = r'ROS_([A-Z]+)_STREAM_FILTER\(.*?,(.*)\);?',
        replace = lambda m: f'RCLCPP_{m.group(1)}({LOGGER}, {stream_to_printf(m.group(2))});',
    ),
]

def stream_to_printf(contents: str):
    elems = map(lambda s: s.strip(), contents.split('<<'))
    exprs = list()
    printf_list = list()
    for elem in elems:
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
    return '%Q'

def apply_sub(data: str, sub: Sub, *, confirm: bool):
    print('Checking for pattern: ', sub.pattern)
    def print_and_replace(m):
        print(f"  replacing: '{m.group(0)}'")
        print(f"       with: '{sub.replace(m)}'")
        if confirm:
            if not click.confirm('', default=True):
                return None
        return sub.replace(m)

    data = re.sub(sub.pattern, print_and_replace, data)
    return data

def do_includes(data: str, *, confirm: bool=False):
    sub = Sub(
        pattern = r'#include <([a-z_]+_msgs)/([a-zA-Z_]+).h>',
        replace = lambda m: f'#include <{m.group(1)}/msg/{inflection.underscore(m.group(2))}.hpp>'
    )
    return apply_sub(data, sub, confirm=confirm)

def do_logging(data: str, *, confirm: bool=False):
    for sub in logging_subs:
        data = apply_sub(data, sub, confirm=confirm)
    return data

def do_msg_uses(data: str, *, confirm: bool=False):
    sub = Sub(
        pattern = r'([a-z_]+_msgs)::([a-zA-z]+)',
        replace = lambda m: f'{m.group(1)}::msg::{m.group(2)}'
    )
    return apply_sub(data, sub, confirm=confirm)

def make_filelist(args: dict):
    return [args.file]

def readfile(filestr: str):
    with open(filestr, 'r') as fs:
        return fs.read()

def writefile(data: str, name: str, args: dict):
    with open(name, 'w') as fs:
        fs.write(data)

def make_argparser():
    parser = argparse.ArgumentParser(description='Convert a C++ file to use ROS2 functionality')
    parser.add_argument('file', help='Relative path to file to process')
    parser.add_argument('-c', '--confirm', action='store_true', help='Confirm each replacement')
    return parser

@click.command()
def main():
    parser = make_argparser()
    args = parser.parse_args()

    actions = [
        do_includes,
        do_msg_uses,
        do_logging,
        #do_nodehandle_calls,
    ]

    files = make_filelist(args)
    for f in files:
        # read file
        orig = readfile(f)

        # send through actions
        processed = orig
        for action in actions:
            processed = action(processed, confirm=args.confirm)

        # save backup
        writefile(orig, f + '.ros1', args)
        # overwrite original file
        writefile(processed, f, args)
