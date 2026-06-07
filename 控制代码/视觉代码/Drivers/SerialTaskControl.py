class SerialTaskControl:
    TASK_COMMANDS = {
        "1": 1,
        "2": 2,
        "3l": "3l",
        "3r": "3r",
        "4": 4,
        "5": 5,
        "6": 6,
    }

    def __init__(self, serial_dev):
        self.serial = serial_dev
        self.buffer = b""

    def poll(self):
        try:
            data = self.serial.read()
        except Exception:
            return None

        if not data:
            return None
        if isinstance(data, str):
            data = data.encode("utf-8")

        self.buffer += data
        line = None
        for sep in (b"\n", b"\r"):
            if sep in self.buffer:
                line, self.buffer = self.buffer.split(sep, 1)
                break
        if line is None:
            if len(self.buffer) > 64:
                self.buffer = b""
            return None

        try:
            text = line.decode("utf-8").strip().lower()
        except Exception:
            return None

        return self._parse(text)

    def _parse(self, text):
        if not text:
            return None
        task_id = self.TASK_COMMANDS.get(text)
        if task_id is not None:
            return ("run", task_id)
        return None
