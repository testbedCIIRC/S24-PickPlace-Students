import logging

def setup_logging(name: str, logging_config) -> logging.Logger:
    log = logging.getLogger(name)

    hdl = logging.StreamHandler()
    if logging_config.level == 'DEBUG':
        log.setLevel(logging.DEBUG)
        hdl.setLevel(logging.DEBUG)
    elif logging_config.level == 'INFO':
        log.setLevel(logging.INFO)
        hdl.setLevel(logging.INFO)
    elif logging_config.level == 'WARNING':
        log.setLevel(logging.WARNING)
        hdl.setLevel(logging.WARNING)
    elif logging_config.level == 'ERROR':
        log.setLevel(logging.ERROR)
        hdl.setLevel(logging.ERROR)
    elif logging_config.level == 'CRITICAL':
        log.setLevel(logging.CRITICAL)
        hdl.setLevel(logging.CRITICAL)
    else:
        log.setLevel(logging.INFO)
        hdl.setLevel(logging.INFO)
    log_formatter = logging.Formatter(
        fmt='[{asctime}] | {name:^' + str(logging_config.name_char_length) + 's} | {levelname:^' + str(logging_config.level_char_length) + 's} | {message}',
        datefmt='%Y-%m-%d %H:%M:%S',
        style='{'
    )
    hdl.setFormatter(log_formatter)
    log.addHandler(hdl)

    return log
