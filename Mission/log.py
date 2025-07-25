import logging
import sys

log_filename = "mission_debug.log"
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler(log_filename, mode='a'),
        logging.StreamHandler(sys.__stdout__)
    ]
)

class LoggerWriter:
    def __init__(self, level):
        self.level = level
        self.buffer = ''

    def write(self, message):
        if message.strip() != "":
            self.buffer += message
        if '\n' in message:
            self.flush()

    def flush(self):
        if self.buffer:
            try:
                self.level(self.buffer)
            except Exception:
                pass
            self.buffer = ''

sys.stdout = LoggerWriter(logging.info)
sys.stderr = LoggerWriter(logging.error)