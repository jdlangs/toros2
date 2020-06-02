from .substitute import Sub

LOGGER = ''

def logger_expr():
    global LOGGER
    if len(LOGGER) == 0:
        LOGGER = click.prompt('Enter expression to use for logger', type=str, default=LOGGER)
    return LOGGER

def logging_subs():
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
            replace = lambda m: f'RCLCPP_{m.group(1)}_STREAM({logger_expr()}, {m.group(2)});',
            save = False,
        ),
        #stream cond
        Sub(
            pattern = r'ROS_([A-Z]+)_STREAM_COND\((.*?),(.*)\);?',
            replace = lambda m: f'RCLCPP_{m.group(1)}_STREAM_EXPRESSION({logger_expr()}, {m.group(2)},{m.group(3)});',
            save = False,
        ),
        #stream once
        Sub(
            pattern = r'ROS_([A-Z]+)_STREAM_ONCE\((.*)\);?',
            replace = lambda m: f'RCLCPP_{m.group(1)}_STREAM_ONCE({logger_expr()}, {m.group(2)});',
            save = False,
        ),
        #stream throttle
        Sub(
            pattern = r'ROS_([A-Z]+)_STREAM_THROTTLE\(.*?,(.*)\);?',
            replace = lambda m: f'RCLCPP_{m.group(1)}_STREAM_THROTTLE({logger_expr()}, {m.group(2)});',
            save = False,
        ),
        #stream delayed throttle
        Sub(
            pattern = r'ROS_([A-Z]+)_STREAM_DELAYED_THROTTLE\(.*?,(.*)\);?',
            replace = lambda m: f'RCLCPP_{m.group(1)}_STREAM_THROTTLE({logger_expr()}, {m.group(2)});',
            save = False,
        ),
        #stream filter
        Sub(
            pattern = r'ROS_([A-Z]+)_STREAM_FILTER\(.*?,(.*)\);?',
            replace = lambda m: f'RCLCPP_{m.group(1)}_STREAM({logger_expr()}, {m.group(2)});',
            save = False,
        ),
    ]
    return logging_subs
